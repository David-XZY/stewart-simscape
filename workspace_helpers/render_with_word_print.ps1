param(
    [Parameter(Mandatory = $true)][string]$InputDocx,
    [Parameter(Mandatory = $true)][string]$OutputDir
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
    $word.ActivePrinter = "Microsoft Print to PDF"
    Write-Output "stage:open"
    $doc = $word.Documents.Open($resolvedInput, $false, $true)
    Write-Output "stage:print"
    $doc.PrintOut($false, $false, 0, $pdfPath, "", "", 0, 1, "", 0, $true, $false)
    Write-Output "stage:printed"
}
finally {
    if ($doc -ne $null) { $doc.Close($false) }
    if ($word -ne $null) { $word.Quit() }
}

$deadline = (Get-Date).AddSeconds(60)
while (-not (Test-Path -LiteralPath $pdfPath)) {
    if ((Get-Date) -gt $deadline) { throw "Timed out waiting for PDF output" }
    Start-Sleep -Milliseconds 250
}
& pdftoppm -png -r 144 $pdfPath (Join-Path $resolvedOutput 'page')
if ($LASTEXITCODE -ne 0) { throw "pdftoppm failed with exit code $LASTEXITCODE" }
Write-Output $pdfPath
