$ErrorActionPreference = 'Stop'
$docx = (Get-ChildItem 'D:\NPU\Matlab_code' -Filter '*Simscape*.docx' |
    Where-Object { -not $_.Name.StartsWith('~$') } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1).FullName
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$document = $null
try {
    $document = $word.Documents.Open($docx, $false, $true, $false)
    $document.Repaginate()
    Write-Output ('PAGES=' + $document.ComputeStatistics(2))
    foreach ($paragraph in $document.Paragraphs) {
        $text = $paragraph.Range.Text.Replace("`r", '').Replace("`a", '')
        if ($paragraph.Range.InlineShapes.Count -gt 0 -or $text -match '^(第4章|4\.1|图4-|表4-)') {
            $page = $paragraph.Range.Information(3)
            $kind = if ($paragraph.Range.InlineShapes.Count -gt 0) { 'IMAGE' } else { 'TEXT' }
            Write-Output ("$kind`t$page`t$text")
        }
    }
    $i = 0
    foreach ($table in $document.Tables) {
        $i++
        $page = $table.Range.Information(3)
        Write-Output ("TABLE`t$i`t$page`trows=" + $table.Rows.Count)
    }
} finally {
    if ($null -ne $document) {
        $document.Close($false)
        [void][Runtime.InteropServices.Marshal]::ReleaseComObject($document)
    }
    $word.Quit()
    [void][Runtime.InteropServices.Marshal]::ReleaseComObject($word)
}
