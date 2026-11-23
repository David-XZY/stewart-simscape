param(
    [Parameter(Mandatory = $true)][string]$InputDocx,
    [Parameter(Mandatory = $true)][string]$OutputDir,
    [switch]$UpdateFields
)

$ErrorActionPreference = 'Stop'
$resolvedInput = (Resolve-Path -LiteralPath $InputDocx).Path
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
$resolvedOutput = (Resolve-Path -LiteralPath $OutputDir).Path
$pdfPath = Join-Path $resolvedOutput (([IO.Path]::GetFileNameWithoutExtension($resolvedInput)) + '.pdf')

$word = $null
$doc = $null
try {
    Write-Output "stage:create-word"
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0
    $word.Options.CheckSpellingAsYouType = $false
    $word.Options.CheckGrammarAsYouType = $false
    $word.Options.SaveNormalPrompt = $false
    Write-Output "stage:open"
    $doc = $word.Documents.Open($resolvedInput, $false, $true)
    Write-Output "stage:opened"
    if ($UpdateFields) {
        foreach ($toc in $doc.TablesOfContents) { $toc.Update() }
        foreach ($story in $doc.StoryRanges) {
            try { $story.Fields.Update() | Out-Null } catch { }
        }
    }
    Write-Output "stage:export"
    $doc.SaveAs2($pdfPath, 17)
    Write-Output "stage:exported"
}
finally {
    if ($doc -ne $null) { $doc.Close($false) }
    if ($word -ne $null) { $word.Quit() }
}

& pdftoppm -png -r 144 $pdfPath (Join-Path $resolvedOutput 'page')
if ($LASTEXITCODE -ne 0) { throw "pdftoppm failed with exit code $LASTEXITCODE" }
Write-Output $pdfPath
