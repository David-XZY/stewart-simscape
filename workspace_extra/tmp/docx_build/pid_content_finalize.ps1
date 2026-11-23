param(
    [Parameter(Mandatory=$true)][string]$DocxPath,
    [Parameter(Mandatory=$true)][string]$EquationJson
)
$ErrorActionPreference = "Stop"
$equations = Get-Content -Raw -Encoding UTF8 -LiteralPath $EquationJson | ConvertFrom-Json
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$doc = $null
$equationDoc = $null
try {
    $doc = $word.Documents.Open($DocxPath, $false, $false)
    $equationDoc = $word.Documents.Add()
    $properties = @($equations.PSObject.Properties)
    for ($propertyIndex = $properties.Count - 1; $propertyIndex -ge 0; $propertyIndex--) {
        $property = $properties[$propertyIndex]
        $id = [string]$property.Name
        $marker = "[[EQ:$id]]"
        $equationRange = $doc.Content.Duplicate
        $find = $equationRange.Find
        $find.Text = $marker
        $find.Forward = $true
        $find.Wrap = 0
        $find.MatchCase = $true
        if (-not $find.Execute()) { throw "Equation marker not found: $id" }
        $equationDoc.Content.Text = ""
        $sourceRange = $equationDoc.Paragraphs.Item(1).Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End-- }
        $sourceRange.Text = [string]$property.Value.linear
        $sourceRange = $equationDoc.Paragraphs.Item(1).Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End-- }
        $sourceRange.Font.Name = "Cambria Math"
        $sourceRange.Font.Size = 12
        $addedRange = $equationDoc.OMaths.Add($sourceRange)
        $addedRange.OMaths.Item(1).BuildUp()
        $sourceRange = $equationDoc.Paragraphs.Item(1).Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End-- }
        $equationRange.FormattedText = $sourceRange.FormattedText
    }
    $equationDoc.Close($false)
    $equationDoc = $null
    [void]$doc.Fields.Update()
    $doc.Repaginate()
    $doc.Save()
    Write-Output ("OMaths=" + $doc.OMaths.Count)
    Write-Output ("Pages=" + $doc.ComputeStatistics(2))
    $doc.Close($false)
    $doc = $null
}
finally {
    if ($null -ne $equationDoc) { try { $equationDoc.Close($false) } catch {} }
    if ($null -ne $doc) { try { $doc.Close($false) } catch {} }
    $word.Quit()
}
