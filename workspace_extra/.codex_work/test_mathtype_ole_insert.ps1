$ErrorActionPreference = 'Stop'
$out = 'D:\NPU\Matlab_code\.codex_work\mathtype_ole_insert_test.docx'
if (Test-Path -LiteralPath $out) {
    Remove-Item -LiteralPath $out -Force
}
$word = $null
$document = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0
    $document = $word.Documents.Add()
    $range = $document.Range(0, 0)
    $shape = $document.InlineShapes.AddOLEObject(
        'Equation.DSMT4', '', $false, $false, '', 0, '', $range)
    $document.SaveAs([ref]$out, [ref]16)
    [pscustomobject]@{
        Saved = Test-Path -LiteralPath $out
        Type = $shape.Type
        ProgID = $shape.OLEFormat.ProgID
        Width = $shape.Width
        Height = $shape.Height
    } | ConvertTo-Json -Compress
}
catch {
    [pscustomobject]@{
        Message = $_.Exception.Message
        HResult = ('0x{0:X8}' -f ($_.Exception.HResult -band 0xffffffffL))
        Type = $_.Exception.GetType().FullName
        ScriptStack = $_.ScriptStackTrace
    } | ConvertTo-Json -Compress
    exit 1
}
finally {
    if ($null -ne $document) { $document.Close(0) }
    if ($null -ne $word) { $word.Quit() }
}
