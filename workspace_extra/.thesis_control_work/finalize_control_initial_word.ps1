param(
    [Parameter(Mandatory=$true)][string]$DocxPath,
    [Parameter(Mandatory=$true)][string]$EquationJson,
    [string]$PdfPath = ""
)
$ErrorActionPreference = "Stop"
$equations = Get-Content -Raw -Encoding UTF8 -LiteralPath $EquationJson | ConvertFrom-Json
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$word.Options.UpdateFieldsAtPrint = $true
$doc = $null
$equationDoc = $null
try {
    $doc = $word.Documents.Open($DocxPath, $false, $false)
    $equationDoc = $word.Documents.Add()
    $properties = @($equations.PSObject.Properties)
    for ($propertyIndex = $properties.Count - 1; $propertyIndex -ge 0; $propertyIndex--) {
        $property = $properties[$propertyIndex]
        $id = [string]$property.Name
        $entry = $property.Value
        $marker = "[[EQ:$id]]"
        $equationRange = $doc.Content.Duplicate
        $find = $equationRange.Find
        $find.ClearFormatting()
        $find.Text = $marker
        $find.Forward = $true
        $find.Wrap = 0
        $find.MatchCase = $true
        $find.MatchWildcards = $false
        $found = $find.Execute()
        if (-not $found) { throw "Equation marker not found: $id" }
        $linear = [string]$entry.linear
        $equationDoc.Content.Text = ""
        $equationParagraph = $equationDoc.Paragraphs.Item(1)
        $sourceRange = $equationParagraph.Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End = $sourceRange.End - 1 }
        $sourceRange.Text = $linear
        $equationParagraph = $equationDoc.Paragraphs.Item(1)
        $sourceRange = $equationParagraph.Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End = $sourceRange.End - 1 }
        $sourceRange.Font.Name = "Cambria Math"
        $sourceRange.Font.NameFarEast = "宋体"
        $sourceRange.Font.Size = 12
        $addedRange = $equationDoc.OMaths.Add($sourceRange)
        $addedRange.OMaths.Item(1).BuildUp()
        $equationParagraph = $equationDoc.Paragraphs.Item(1)
        $sourceRange = $equationParagraph.Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End = $sourceRange.End - 1 }
        $equationRange.FormattedText = $sourceRange.FormattedText
    }
    $equationDoc.Close($false)
    $equationDoc = $null
    [void]$doc.Fields.Update()
    for ($i = 1; $i -le $doc.TablesOfContents.Count; $i++) {
        $doc.TablesOfContents.Item($i).Update()
    }
    $doc.Repaginate()
    $doc.Save()
    if ($PdfPath -ne "") {
        $pdfDir = Split-Path -Parent $PdfPath
        if (-not (Test-Path -LiteralPath $pdfDir)) { New-Item -ItemType Directory -Force -Path $pdfDir | Out-Null }
        $doc.ExportAsFixedFormat($PdfPath, 17)
    }
    $doc.Close($false)
    $doc = $null
}
finally {
    if ($null -ne $equationDoc) {
        try { $equationDoc.Close($false) } catch { }
    }
    if ($null -ne $doc) {
        try { $doc.Close($false) } catch { }
    }
    $word.Quit()
    [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($word)
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
