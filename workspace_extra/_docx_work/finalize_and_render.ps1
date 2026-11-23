$ErrorActionPreference = 'Stop'
$docx = (Get-ChildItem 'D:\NPU\Matlab_code' -Filter '*Simscape*.docx' |
    Where-Object { -not $_.Name.StartsWith('~$') } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1).FullName
$outDir = 'D:\NPU\Matlab_code\_docx_work\final-render'
$pdf = Join-Path $outDir 'simscape_section.pdf'
$normalizedDocx = Join-Path $outDir 'simscape_section_finalized.docx'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$document = $null
try {
    $document = $word.Documents.Open($docx, $false, $false, $false)
    $footerRange = $document.Sections.Item(1).Footers.Item(1).Range
    $footerRange.Text = ''
    $footerRange.ParagraphFormat.Alignment = 1
    $footerRange.Font.Name = 'Times New Roman'
    $footerRange.Font.NameFarEast = 'SimSun'
    $footerRange.Font.Size = 9
    $null = $document.Fields.Add($footerRange, 33)
    $document.Repaginate()
    $document.SaveAs2($normalizedDocx, 16)
    $pages = $document.ComputeStatistics(2)
    Write-Output "PAGES=$pages"
    $document.Close($false)
    [void][Runtime.InteropServices.Marshal]::ReleaseComObject($document)
    $document = $null
    $document = $word.Documents.Open($normalizedDocx, $false, $true, $false)
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
Copy-Item -LiteralPath $normalizedDocx -Destination $docx -Force
