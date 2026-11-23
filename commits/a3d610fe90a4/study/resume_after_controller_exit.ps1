# One-time recovery of the observed third native crash and controller exit.
$ErrorActionPreference='Stop'
$ProgressPreference='SilentlyContinue'
$runRoot=[IO.Path]::GetFullPath('D:/XieZhenyu/stewart-simscape-diffusion/results/study/frozen_v1')
$running=@(Get-CimInstance Win32_Process | Where-Object { $_.Name -match 'python|MATLAB' -and $_.CommandLine -match 'sgWorker\(|study[/\\](run_study|launch_supervisor).py' })
if ($running.Count -gt 0) { throw 'Study processes still exist; duplicate launch refused.' }
$archive=Join-Path $runRoot 'environment_recovery_controller_20260916'
if (Test-Path -LiteralPath (Join-Path $archive 'incident.json')) { throw 'Recovery already applied.' }
New-Item -ItemType Directory -Force -Path $archive | Out-Null
foreach ($name in @('progress.json','run_control.json','active_1.json','active_2.json')) {
    Copy-Item -LiteralPath (Join-Path $runRoot $name) -Destination (Join-Path $archive $name)
}
$failed=Get-Content -Raw (Join-Path $runRoot 'active_1.json') | ConvertFrom-Json
$other=Get-Content -Raw (Join-Path $runRoot 'active_2.json') | ConvertFrom-Json
if ($failed.jobId -ne 'physical_a3_p20__diff_ihsid__k08__r1' -or $other.jobId -ne 'physical_a4_p19__diff_ihsid__k08__r1') { throw 'Unexpected active jobs.' }
$row=[ordered]@{}
foreach ($object in @($failed.metadata,$failed.job)) { foreach ($property in $object.PSObject.Properties) { $row[$property.Name]=$property.Value } }
$row.fingerprint=$failed.fingerprint; $row.completed=$true; $row.passed=$false
$row.status='worker_process_exit'; $row.reason='MATLAB native heap corruption (0xc0000374); controller exited before recording exact elapsed time.'
$row.onlineSeconds=$null; $row.timingUnavailable=$true
$progress=Get-Content -Raw (Join-Path $runRoot 'progress.json') | ConvertFrom-Json
$row.lastTelemetryElapsedSeconds=$progress.epoch-$failed.startedEpoch
$target=Join-Path $runRoot ('jobs/'+$failed.jobId+'/result.json')
if (Test-Path -LiteralPath $target) { throw 'Native failure already has a result.' }
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
[IO.File]::WriteAllText($target,($row | ConvertTo-Json -Depth 12),(New-Object Text.UTF8Encoding($false)))
$jobRoot=[IO.Path]::GetFullPath((Join-Path $runRoot ('jobs/'+$other.jobId)))
$jobArchive=[IO.Path]::GetFullPath((Join-Path $jobRoot 'interrupted_controller_20260916'))
if (-not $jobArchive.StartsWith($runRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Archive escapes run root.' }
New-Item -ItemType Directory -Force -Path $jobArchive | Out-Null
foreach ($name in @('result.json','result.mat','online.mat','online_progress.json','first_audit.mat')) {
    $source=[IO.Path]::GetFullPath((Join-Path $jobRoot $name))
    if (-not $source.StartsWith($runRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Source escapes run root.' }
    if (Test-Path -LiteralPath $source) { Move-Item -LiteralPath $source -Destination (Join-Path $jobArchive $name) }
}
@{reason='Native heap corruption followed by exhausted controller restart guard';failedJob=$failed.jobId;interruptedJob=$other.jobId;recoveredUtc=[DateTime]::UtcNow.ToString('o');action='Retain native failure; resume the other interrupted attempt and remaining frozen jobs';numericalImplementationUnchanged=$true;computeDeadlineUtc='2026-09-17T03:46:51Z'} | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $archive 'incident.json')
$startup=New-CimInstance -ClassName Win32_ProcessStartup -ClientOnly -Property @{ShowWindow=[uint16]0;CreateFlags=[uint32]16777216}
$created=Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{CommandLine='D:\XieZhenyu\envs\stewart-abfd\Scripts\python.exe D:\XieZhenyu\stewart-simscape-diffusion\study\launch_supervisor.py frozen_v1';CurrentDirectory='D:\XieZhenyu\stewart-simscape-diffusion';ProcessStartupInformation=$startup}
$created | Select-Object ReturnValue,ProcessId | ConvertTo-Json
if ($created.ReturnValue -ne 0) { throw 'Detached launch failed.' }
