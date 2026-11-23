$ErrorActionPreference = 'Stop'
$docx = (Get-ChildItem 'D:\NPU\Matlab_code' -Filter '*Simscape*.docx' |
    Where-Object { -not $_.Name.StartsWith('~$') } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1).FullName
$outDir = 'D:\NPU\Matlab_code\_docx_work\word-render'
$pdf = Join-Path $outDir 'simscape_section.pdf'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
Write-Output 'START_WORD'
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$document = $null
try {
    Write-Output 'OPEN_DOC'
    $document = $word.Documents.Open($docx, $false, $true, $false)
    Write-Output 'REPAGINATE'
    $document.Repaginate()
    $pages = $document.ComputeStatistics(2)
    Write-Output "PAGES=$pages"
    Write-Output 'EXPORT_PDF'
    $document.ExportAsFixedFormat($pdf, 17)
    Write-Output "PDF=$pdf"
} finally {
    if ($null -ne $document) {
        $document.Close($false)
        [void][Runtime.InteropServices.Marshal]::ReleaseComObject($document)
    }
    $word.Quit()
    [void][Runtime.InteropServices.Marshal]::ReleaseComObject($word)
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
