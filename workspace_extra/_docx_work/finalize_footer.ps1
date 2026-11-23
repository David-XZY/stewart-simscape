$ErrorActionPreference = 'Stop'
$docx = (Get-ChildItem 'D:\NPU\Matlab_code' -Filter '*Simscape*.docx' |
    Where-Object { -not $_.Name.StartsWith('~$') } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1).FullName
$temp = 'D:\NPU\Matlab_code\_docx_work\footer_finalized.docx'
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$document = $null
try {
    $document = $word.Documents.Open($docx, $false, $false, $false)
    $footer = $document.Sections.Item(1).Footers.Item(1).Range
    $footer.Text = ''
    $footer.ParagraphFormat.Alignment = 1
    $footer.Font.Name = 'Times New Roman'
    $footer.Font.NameFarEast = 'SimSun'
    $footer.Font.Size = 9
    $null = $document.Fields.Add($footer, 33)
    $document.Repaginate()
    $pages = $document.ComputeStatistics(2)
    $document.SaveAs2($temp, 16)
    Write-Output "PAGES=$pages"
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
Copy-Item -LiteralPath $temp -Destination $docx -Force
