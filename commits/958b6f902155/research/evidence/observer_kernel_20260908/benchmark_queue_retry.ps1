$ErrorActionPreference='Stop';$ProgressPreference='SilentlyContinue'
$job='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/observer_kernel_benchmark_20260908';$source='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/observer_kernel_benchmark_20260908/source_42ff65e';$pipeline='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_interval_closedloop_20260908'
$manifest=Get-Content -Raw -LiteralPath ($job+'/manifest.json') | ConvertFrom-Json
$state=@{state='starting';queuePid=$PID;predecessorPid=46896;started=(Get-Date).ToString('o');scope=$manifest.scope}
function FileSha([string]$path){$s=[IO.File]::OpenRead($path);$a=[Security.Cryptography.SHA256]::Create();try{return [BitConverter]::ToString($a.ComputeHash($s)).Replace('-','').ToLowerInvariant()}finally{$s.Dispose();$a.Dispose()}}
function Publish{$state | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath ($job+'/queue_retry/status.json') -Encoding UTF8}
try{
 $identity=Get-CimInstance Win32_Process -Filter 'ProcessId=46896'
 if(-not $identity -or -not $identity.CommandLine.Replace([char]92,[char]47).Contains($manifest.predecessorScript.Replace([char]92,[char]47))){throw 'Predecessor is not the original complete-F dependency queue'}
 if((FileSha $manifest.predecessorScript) -ne $manifest.predecessorScriptHash){throw 'Predecessor script changed'}
 $previous=Get-Process -Id 46896;$null=$previous.Handle
 if([Math]::Abs(($previous.StartTime-$identity.CreationDate).TotalSeconds) -gt 1){throw 'Predecessor identity changed'}
 $state.state='waiting-for-complete-F-native-queue';$state.predecessorStartTime=$previous.StartTime.ToString('o');Publish
 $previous.WaitForExit();$state.predecessorExitCode=$previous.ExitCode
 if($previous.ExitCode -ne 0){throw 'The frozen F queue did not exit successfully; benchmark not launched'}
 $summary=Get-Content -Raw -LiteralPath ($pipeline+'/summary_complete.json') | ConvertFrom-Json
 $completed=Get-Content -Raw -LiteralPath ($pipeline+'/wave_02_queue_retry/status.json') | ConvertFrom-Json
 if($summary.state -ne 'complete' -or $summary.plannedCount -ne 432 -or $completed.state -ne 'complete'){throw 'Full F artifact audit is incomplete'}
 if(@(Get-CimInstance Win32_Process -Filter "Name='MATLAB.exe'").Count -ne 0){throw 'Another local MATLAB is active; benchmark not launched'}
 if((FileSha $manifest.inputFile) -ne $manifest.inputHash){throw 'Training input changed'}
 for($k=0;$k -lt $manifest.sourceFiles.Count;$k++){if((FileSha (Join-Path $source $manifest.sourceFiles[$k])) -ne $manifest.sourceHashes[$k]){throw 'Frozen benchmark source changed'}}
 $env:OMP_NUM_THREADS='1';$env:MKL_NUM_THREADS='1';$env:OPENBLAS_NUM_THREADS='1';$env:TEMP=$job+'/tmp';$env:TMP=$env:TEMP
 New-Item -ItemType Directory -Path $env:TEMP -ErrorAction Stop | Out-Null
 $child=Start-Process -FilePath 'D:/MATLAB/R2025b/bin/matlab.exe' -ArgumentList @('-wait','-singleCompThread','-batch',('"run('''+$job+'/run.m'')"')) -WorkingDirectory $source -WindowStyle Hidden -RedirectStandardOutput ($job+'/stdout.log') -RedirectStandardError ($job+'/stderr.log') -PassThru
 $null=$child.Handle;$state.launcherPid=$child.Id;$state.state='running-training-benchmark';Publish
 $child.WaitForExit();$state.exitCode=$child.ExitCode
 if($child.ExitCode -ne 0){throw 'Observer benchmark failed; deployed observer remains unchanged'}
 $result=Get-Content -Raw -LiteralPath ($job+'/batch_status.json') | ConvertFrom-Json
 if($result.state -ne 'complete' -or -not $result.passed){throw 'Observer equivalence benchmark did not pass'}
 $state.state='complete';$state.finished=(Get-Date).ToString('o');Publish
}catch{$state.state='failed';$state.message=$_.Exception.Message;$state.finished=(Get-Date).ToString('o');Publish;throw}
