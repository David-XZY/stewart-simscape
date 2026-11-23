$ErrorActionPreference='Stop'
$workspaceRoot=[IO.Path]::GetFullPath('D:\NPU\Matlab_code').TrimEnd('\')
$recordRoot=Join-Path $workspaceRoot 'cleanup_20260922'
$summary=Get-Content -LiteralPath (Join-Path $recordRoot 'deletion_summary.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$parents=[Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
Get-Content -LiteralPath (Join-Path $recordRoot 'deletion_results.jsonl') -Encoding UTF8 | ForEach-Object {
 $entry=$_ | ConvertFrom-Json
 if($entry.status -eq 'deleted'){
  $parent=[IO.Path]::GetDirectoryName([IO.Path]::GetFullPath($entry.path))
  while($parent -and $parent.StartsWith($workspaceRoot+'\',[StringComparison]::OrdinalIgnoreCase)){
   [void]$parents.Add($parent);$parent=[IO.Path]::GetDirectoryName($parent)
  }
 }
}
$removed=[Collections.Generic.List[string]]::new()
foreach($path in ($parents | Sort-Object Length -Descending)){
 if(-not $path.StartsWith($workspaceRoot+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Directory escaped workspace'}
 if([IO.Path]::GetDirectoryName($path) -eq $workspaceRoot -and [IO.Path]::GetFileName($path) -ne '__pycache__'){continue}
 if(-not (Test-Path -LiteralPath $path -PathType Container)){continue}
 $item=Get-Item -LiteralPath $path -Force
 if(($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0 -or $item.Name -eq '.git'){continue}
 if(@(Get-ChildItem -LiteralPath $path -Force | Select-Object -First 1).Count -ne 0){continue}
 # No -Recurse: only verified empty directories from deleted-file parents.
 Remove-Item -LiteralPath $path -Force -Confirm:$false -ErrorAction Stop
 $removed.Add($path)
}
@{removedEmptyDirectories=$removed.Count;paths=@($removed)} | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $recordRoot 'empty_directories_removed.json') -Encoding UTF8
Write-Output "EMPTY_DIRECTORIES_REMOVED $($removed.Count)"
