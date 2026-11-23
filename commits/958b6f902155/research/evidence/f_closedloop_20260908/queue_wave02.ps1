$ErrorActionPreference='Stop';$ProgressPreference='SilentlyContinue'
$queueRoot='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_interval_closedloop_20260908/wave_02_queue_retry'
$base='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_interval_closedloop_20260908'
$frozen='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_interval_closedloop_20260908/source_cddb795'
$inputs='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_full_delivery_20260908'
$snapshot='D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_holdout_closedloop_inputs_20260908_wave05'
$state=@{state='starting';queuePid=$PID;predecessorPid=4852;started=(Get-Date).ToString('o');scope='One-shot dependency queue; unchanged frozen F implementation; no concurrent local MATLAB'}
function FileSha([string]$path) {
 $stream=[System.IO.File]::OpenRead($path);$algorithm=[System.Security.Cryptography.SHA256]::Create()
 try{return [System.BitConverter]::ToString($algorithm.ComputeHash($stream)).Replace('-','').ToLowerInvariant()}
 finally{$stream.Dispose();$algorithm.Dispose()}
}
function Publish { $state | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath ($queueRoot+'/status.json') -Encoding UTF8 }
try {
 $identity=Get-CimInstance Win32_Process -Filter 'ProcessId=4852'
 $runner=$frozen+'/research/remote/run_f_closedloop_pipeline.py'
 if(-not $identity -or -not $identity.CommandLine.Replace('\','/').Contains($runner) -or -not $identity.CommandLine.Replace('\','/').Contains($base+'/wave_01')){throw 'Predecessor is not the recorded frozen F worker'}
 $predecessor=Get-Process -Id 4852;$null=$predecessor.Handle
 if([Math]::Abs(($predecessor.StartTime-$identity.CreationDate).TotalSeconds) -gt 1){throw 'Predecessor process identity changed'}
 if((FileSha ($base+'/wave_01/source_manifest.json')) -ne 'd55d4ac79a2c651e32bcac4c6c55c9aea2ddd2c2704b175e5f21fb7d23533520'){throw 'Previous freeze changed'}
 if((FileSha ($snapshot+'/transfer_manifest.json')) -ne 'f65d3a14abfaa633a958d0fb26d4fb03b356e5b987162ce779608c2c0b4a56a9'){throw 'New input manifest changed'}
 if(Test-Path -LiteralPath ($base+'/wave_02')){throw 'Fresh wave directory already exists'}
 $state.state='waiting-for-native-predecessor';$state.predecessorStartTime=$predecessor.StartTime.ToString('o');Publish
 $predecessor.WaitForExit();$state.predecessorExitCode=$predecessor.ExitCode
 if($predecessor.ExitCode -ne 0){throw 'Previous native Python worker failed; no dependent simulation started'}
 $previous=Get-Content -LiteralPath ($base+'/wave_01/pipeline_status.json') -Raw | ConvertFrom-Json
 if($previous.state -ne 'partial' -or $previous.workerFailures.Count -ne 0 -or $previous.completedTasks.Count -ne 9){throw 'Previous wave lacks all nine completed native tasks'}
 $expected=@('task_20260962','task_20260964','task_20260965')
 if(Compare-Object $expected $previous.awaitingTransfer){throw 'Remaining original task identities changed'}
 foreach($task in $previous.completedTasks){
  $native=Get-Content -LiteralPath ($base+'/wave_01/jobs/'+$task.taskId+'/status.json') -Raw | ConvertFrom-Json
  if($native.state -ne 'complete' -or $native.exitCode -ne 0){throw 'A previous MATLAB worker did not complete'}
 }
 if(@(Get-CimInstance Win32_Process -Filter "Name='MATLAB.exe'").Count -ne 0){throw 'Another local MATLAB is active; preserve timing protocol and do not start another worker'}
 if((FileSha $runner) -ne 'f4f78d60c327ef8c318ac30c85d4330dca8e7683f7d5a67ab4378fb81db2db20'){throw 'Frozen execution runner changed'}
 if((FileSha ($snapshot+'/transfer_manifest.json')) -ne 'f65d3a14abfaa633a958d0fb26d4fb03b356e5b987162ce779608c2c0b4a56a9'){throw 'Queued input manifest changed'}
 $arguments=@($runner,$inputs,($base+'/wave_02'),'--snapshot',$snapshot)
 $state.state='launching-wave-02';Publish
 $child=Start-Process -FilePath 'D:/Python313/python.exe' -ArgumentList $arguments -WorkingDirectory $frozen -WindowStyle Hidden -RedirectStandardOutput ($queueRoot+'/stdout.log') -RedirectStandardError ($queueRoot+'/stderr.log') -PassThru
 $null=$child.Handle;$state.workerPid=$child.Id;$state.state='running-wave-02';Publish
 $child.WaitForExit();$state.exitCode=$child.ExitCode
 if($child.ExitCode -ne 0){throw 'Dependent wave failed; all original outcomes retained'}
 $summaryArgs=@(($frozen+'/research/remote/summarize_f_closedloop.py'),$inputs,($base+'/summary_complete.json'),'--wave',($base+'/wave_01'),'--wave',($base+'/wave_02'))
 & 'D:/Python313/python.exe' @summaryArgs
 if($LASTEXITCODE -ne 0){throw 'Combined native artifact audit failed'}
 $summary=Get-Content -LiteralPath ($base+'/summary_complete.json') -Raw | ConvertFrom-Json
 if($summary.state -ne 'complete' -or $summary.plannedCount -ne 432){throw 'Combined full 432-case matrix is incomplete'}
 $state.state='complete';$state.acceptedCount=$summary.acceptedCount;$state.finished=(Get-Date).ToString('o');Publish
} catch {
 $state.state='failed';$state.message=$_.Exception.Message;$state.finished=(Get-Date).ToString('o');Publish;throw
}
