param(
    [Parameter(Mandatory=$true)][string]$DocumentPath,
    [Parameter(Mandatory=$true)][string]$Marker
)

$word = $null
$doc = $null
$shape = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $true
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Open($DocumentPath, $false, $false)
    $searchRange = $doc.Content
    $find = $searchRange.Find
    $find.Text = $Marker
    $find.Forward = $true
    $find.Wrap = 0
    if (-not $find.Execute()) {
        throw "Marker not found: $Marker"
    }
    $searchRange.Text = ''
    $shape = $doc.InlineShapes.AddOLEObject(
        'Equation.DSMT4',
        [Type]::Missing,
        $false,
        $false,
        [Type]::Missing,
        [Type]::Missing,
        [Type]::Missing,
        $searchRange)
    $doc.Save()
    Write-Output ('ProgID=' + $shape.OLEFormat.ProgID)
} finally {
    if ($null -ne $doc) { try { $doc.Close($false) } catch { } }
    if ($null -ne $word) { try { $word.Quit() } catch { } }
    if ($null -ne $shape) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($shape) | Out-Null }
    if ($null -ne $doc) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc) | Out-Null }
    if ($null -ne $word) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null }
}
