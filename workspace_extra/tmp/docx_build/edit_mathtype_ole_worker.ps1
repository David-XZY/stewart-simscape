param(
    [Parameter(Mandatory=$true)][string]$DocumentPath,
    [int]$ShapeIndex = 1
)

$word = $null
$doc = $null
$shape = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $true
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Open($DocumentPath, $false, $false)
    $shape = $doc.InlineShapes.Item($ShapeIndex)
    $shape.Range.Select()
    $shape.OLEFormat.DoVerb(-1)
    # Allow the external UI driver to enter and close the equation.
    Start-Sleep -Seconds 25
    $doc.Save()
    Write-Output ('ProgID=' + $shape.OLEFormat.ProgID)
} finally {
    if ($null -ne $doc) { try { $doc.Close($false) } catch { } }
    if ($null -ne $word) { try { $word.Quit() } catch { } }
    if ($null -ne $shape) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($shape) | Out-Null }
    if ($null -ne $doc) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc) | Out-Null }
    if ($null -ne $word) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null }
}
