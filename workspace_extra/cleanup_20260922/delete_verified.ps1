$ErrorActionPreference='Stop'
$workspaceRoot=[IO.Path]::GetFullPath('D:\NPU\Matlab_code').TrimEnd('\')
$recordRoot=Join-Path $workspaceRoot 'cleanup_20260922'
$manifestPath=Join-Path $recordRoot 'deletion_manifest.json'
$check=Get-Content -LiteralPath (Join-Path $recordRoot 'preflight_passed.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if(-not $check.passed -or -not $check.latestArchiveUnchanged){throw 'Preflight failed'}
if((Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $check.manifestSHA256){throw 'Deletion manifest changed'}
$active=@(Get-CimInstance Win32_Process -Filter "Name='MATLAB.exe'" | Where-Object {$_.CommandLine -like '*Matlab_code*' -or -not $_.CommandLine})
if($active.Count){throw 'A MATLAB task may use this workspace; no files deleted'}
$manifest=Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
$protected=[Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach($p in $manifest.protectedPaths){[void]$protected.Add($p)}
function Assert-ContainedFile([string]$path){
 $absolute=[IO.Path]::GetFullPath($path)
 if(-not $absolute.StartsWith($workspaceRoot+'\',[StringComparison]::OrdinalIgnoreCase)){throw "Outside workspace: $absolute"}
 if($protected.Contains($absolute)){throw "Protected path: $absolute"}
 $item=Get-Item -LiteralPath $absolute -Force
 if($item.PSIsContainer){throw "Expected file: $absolute"}
 $cursor=$item
 while($null -ne $cursor){
  if(($cursor.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0){throw "Reparse point: $($cursor.FullName)"}
  if($cursor.FullName.Equals($workspaceRoot,[StringComparison]::OrdinalIgnoreCase)){break}
  if($cursor.Name -eq '.git'){throw 'Git content excluded'}
  $cursor=if($cursor -is [IO.FileInfo]){$cursor.Directory}else{$cursor.Parent}
 }
 if($null -eq $cursor){throw 'Workspace ancestor not found'}
 return $absolute
}
# All path checks run before the first removal. Files are removed individually;
# no recursive deletion and no shell-generated wildcard command is used.
foreach($entry in $manifest.files){[void](Assert-ContainedFile $entry.path)}
$log=Join-Path $recordRoot 'deletion_results.jsonl'
if(Test-Path -LiteralPath $log){throw 'Deletion log already exists; review it instead of rerunning blindly'}
$writer=[IO.StreamWriter]::new($log,$false,[Text.UTF8Encoding]::new($false));$writer.AutoFlush=$true
$deleted=0;$skipped=0;$bytes=[long]0
try {
 foreach($entry in $manifest.files){
  $status='skipped';$message=''
  try {
   $absolute=Assert-ContainedFile $entry.path
   $file=Get-Item -LiteralPath $absolute -Force
   if($file.Length -ne [long]$entry.bytes){$message='size changed'}
   elseif((Get-FileHash -LiteralPath $absolute -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.sha256){$message='content hash changed'}
   else {
    Remove-Item -LiteralPath $absolute -Force -ErrorAction Stop
    if(Test-Path -LiteralPath $absolute){throw 'File still exists after removal'}
    $status='deleted';$deleted++;$bytes+=[long]$entry.bytes
   }
  } catch {$message=$_.Exception.Message}
  if($status -ne 'deleted'){$skipped++}
  $writer.WriteLine((@{path=$entry.path;category=$entry.category;bytes=$entry.bytes;sha256=$entry.sha256;status=$status;message=$message;time=(Get-Date).ToString('o')} | ConvertTo-Json -Compress))
  if(($deleted+$skipped)%500 -eq 0){Write-Output "Processed $($deleted+$skipped) / $($manifest.files.Count), deleted $deleted, skipped $skipped"}
 }
} finally {$writer.Dispose()}
@{deleted=$deleted;skipped=$skipped;deletedBytes=$bytes;completedAt=(Get-Date).ToString('o')} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $recordRoot 'deletion_summary.json') -Encoding UTF8
Write-Output "DELETE_COMPLETE files=$deleted skipped=$skipped bytes=$bytes"
