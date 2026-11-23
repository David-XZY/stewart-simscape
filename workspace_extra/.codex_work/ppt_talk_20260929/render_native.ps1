$ErrorActionPreference='Stop'
$source='D:\NPU\组会PPT\26.9.29组会_第一人称通俗讲稿.docx'
$render='D:\NPU\Matlab_code\.codex_work\ppt_talk_20260929\render'
New-Item -ItemType Directory -Force -Path $render | Out-Null
$pdf=Join-Path $render 'manuscript.pdf'
$app=New-Object -ComObject Word.Application
$app.Visible=$false
$app.DisplayAlerts=0
$document=$null
try {
  $document=$app.Documents.Open($source,$false,$true,$false)
  $document.Repaginate()
  $pages=$document.ComputeStatistics(2)
  $document.ExportAsFixedFormat($pdf,17)
  $map=@()
  for($i=1;$i -le $pages;$i++) {
    $start=$document.GoTo(1,1,$i).Start
    if($i -lt $pages){$end=$document.GoTo(1,1,($i+1)).Start} else {$end=$document.Content.End}
    $body=$document.Range($start,$end).Text
    $map += [pscustomobject]@{page=$i;start=$start;end=$end;characters=$body.Length;text=$body}
  }
  $map | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $render 'pages.json') -Encoding utf8
  Write-Output ('Word exported '+$pages+' pages')
} finally {
  if($document){$document.Close($false);[void][Runtime.InteropServices.Marshal]::ReleaseComObject($document)}
  $app.Quit()
  [void][Runtime.InteropServices.Marshal]::ReleaseComObject($app)
}
& 'C:\Users\David_X\.cache\codex-runtimes\codex-primary-runtime\dependencies\native\poppler\Library\bin\pdftoppm.exe' -png -r 120 $pdf (Join-Path $render 'page')
if($LASTEXITCODE -ne 0){throw 'Page rendering failed'}
