$ErrorActionPreference='Stop';$ProgressPreference='SilentlyContinue'
$job='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/observer_kernel_simscape_20260908';$manifest=Get-Content -Raw -LiteralPath ($job+'/manifest.json') | ConvertFrom-Json
$source=$manifest.source;$benchmark=$manifest.benchmark
$state=@{state='starting';queuePid=$PID;predecessorPid=18312;started=(Get-Date).ToString('o');scope=$manifest.scope}
function FileSha([string]$path){$s=[IO.File]::OpenRead($path);$a=[Security.Cryptography.SHA256]::Create();try{return [BitConverter]::ToString($a.ComputeHash($s)).Replace('-','').ToLowerInvariant()}finally{$s.Dispose();$a.Dispose()}}
function Publish{$state | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath ($job+'/queue_status.json') -Encoding UTF8}
try{
 $identity=Get-CimInstance Win32_Process -Filter 'ProcessId=18312'
 if(-not $identity -or -not $identity.CommandLine.Replace([char]92,[char]47).Contains($manifest.predecessorScript.Replace([char]92,[char]47))){throw 'Predecessor is not the original observer benchmark queue'}
 if((FileSha $manifest.predecessorScript) -ne $manifest.predecessorScriptHash){throw 'Predecessor script changed'}
 $previous=Get-Process -Id 18312;$null=$previous.Handle
 if([Math]::Abs(($previous.StartTime-$identity.CreationDate).TotalSeconds) -gt 1){throw 'Predecessor identity changed'}
 $state.state='waiting-for-native-kernel-benchmark';$state.predecessorStartTime=$previous.StartTime.ToString('o');Publish
 $previous.WaitForExit();$state.predecessorExitCode=$previous.ExitCode
 if($previous.ExitCode -ne 0){throw 'Numerical benchmark exited unsuccessfully; full simulation not launched'}
 if((FileSha ($benchmark+'/manifest.json')) -ne $manifest.predecessorManifestHash){throw 'Benchmark manifest changed'}
 $completed=Get-Content -Raw -LiteralPath ($benchmark+'/queue_retry/status.json') | ConvertFrom-Json
 $result=Get-Content -Raw -LiteralPath ($benchmark+'/batch_status.json') | ConvertFrom-Json
 if($completed.state -ne 'complete' -or $result.state -ne 'complete' -or -not $result.passed -or $result.records.Count -ne 3){throw 'Three-model numerical benchmark is not accepted'}
 foreach($record in $result.records){if(-not $record.checks.passed -or $record.checks.comparisons -ne 48 -or $record.checks.refusals -ne 4){throw 'Required observer equivalence or refusal checks missing'}}
 $prior=Get-Content -Raw -LiteralPath ($benchmark+'/manifest.json') | ConvertFrom-Json
 for($k=0;$k -lt $manifest.kernelFiles.Count;$k++){if((FileSha (Join-Path $prior.source $manifest.kernelFiles[$k])) -ne $manifest.kernelHashes[$k]){throw 'Numerical benchmark used different observer code'}}
 if(@(Get-CimInstance Win32_Process -Filter "Name='MATLAB.exe'").Count -ne 0){throw 'Another local MATLAB is active; do not contaminate timing'}
 foreach($file in $manifest.inputFiles){if((FileSha $file.path) -ne $file.sha256){throw 'Frozen training input changed'}}
 for($k=0;$k -lt $manifest.sourceFiles.Count;$k++){if((FileSha (Join-Path $source $manifest.sourceFiles[$k])) -ne $manifest.sourceHashes[$k]){throw 'Frozen runtime source changed'}}
 foreach($file in $manifest.runtimeDependencies){if((FileSha (Join-Path $source $file.path)) -ne $file.sha256){throw 'Frozen runtime binary changed'}}
 $env:OMP_NUM_THREADS='1';$env:MKL_NUM_THREADS='1';$env:OPENBLAS_NUM_THREADS='1';$env:TEMP=$job+'/tmp';$env:TMP=$env:TEMP
 New-Item -ItemType Directory -Path $env:TEMP -ErrorAction Stop | Out-Null
 $child=Start-Process -FilePath 'D:/MATLAB/R2025b/bin/matlab.exe' -ArgumentList @('-wait','-singleCompThread','-batch',('"run('''+$job+'/run.m'')"')) -WorkingDirectory $source -WindowStyle Hidden -RedirectStandardOutput ($job+'/stdout.log') -RedirectStandardError ($job+'/stderr.log') -PassThru
 $null=$child.Handle;$state.launcherPid=$child.Id;$state.state='running-training-simscape-kernel-pair';Publish
 $child.WaitForExit();$state.exitCode=$child.ExitCode
 if($child.ExitCode -ne 0){throw 'Full Simscape kernel comparison failed to execute'}
 $result=Get-Content -Raw -LiteralPath ($job+'/batch_status.json') | ConvertFrom-Json
 if($result.state -ne 'complete' -or $result.comparison -ne 'kernel' -or $result.records.Count -ne 2){throw 'Full kernel comparison is incomplete'}
 $state.state='complete';$state.accepted=$result.passed;$state.equivalencePassed=$result.equivalence.passed;$state.finished=(Get-Date).ToString('o');Publish
}catch{$state.state='failed';$state.message=$_.Exception.Message;$state.finished=(Get-Date).ToString('o');Publish;throw}
