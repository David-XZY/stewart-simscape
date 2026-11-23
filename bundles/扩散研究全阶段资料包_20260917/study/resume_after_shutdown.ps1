# One-time recovery of the observed 2026-09-16 workstation shutdown.
$ErrorActionPreference='Stop'
$ProgressPreference='SilentlyContinue'
$runRoot=[IO.Path]::GetFullPath('D:/XieZhenyu/stewart-simscape-diffusion/results/study/frozen_v1')
$running=@(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'study[/\\]run_study.py|sgWorker\(' })
if ($running.Count -gt 0) { throw 'A study process is still running; refusing duplicate launch.' }
$recovery=Join-Path $runRoot 'environment_recovery_20260916T111429Z'
if (Test-Path -LiteralPath (Join-Path $recovery 'incident.json')) { throw 'This interruption has already been archived.' }
New-Item -ItemType Directory -Force -Path $recovery | Out-Null
foreach ($name in @('progress.json','run_control.json','active_1.json','active_2.json','supervisor.lock')) {
    $source=Join-Path $runRoot $name
    if (Test-Path -LiteralPath $source) { Copy-Item -LiteralPath $source -Destination (Join-Path $recovery $name) -Force }
}
$interrupted=@('goal_hood_position_1_2__diff_ihsid__k08__r2','goal_hood_position_2_1__retrieve_fast__k64__r1','layout_simple_2__diff_direct__k32__r2','goal_simple_position_1_1__diff_fast__k64__r2')
foreach ($job in $interrupted) {
    $jobRoot=[IO.Path]::GetFullPath((Join-Path (Join-Path $runRoot 'jobs') $job))
    $archive=[IO.Path]::GetFullPath((Join-Path $jobRoot 'interrupted_shutdown_20260916T111429Z'))
    if (-not $archive.StartsWith($runRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Archive is outside study workspace.' }
    New-Item -ItemType Directory -Force -Path $archive | Out-Null
    foreach ($name in @('result.json','result.mat','online.mat','online_progress.json','first_audit.mat')) {
        $source=[IO.Path]::GetFullPath((Join-Path $jobRoot $name))
        if (-not $source.StartsWith($runRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Source is outside study workspace.' }
        if (Test-Path -LiteralPath $source) { Move-Item -LiteralPath $source -Destination (Join-Path $archive $name) }
    }
}
@{reason='Windows planned shutdown and reboot';bootUtc='2026-09-16T11:14:29Z';shutdownEventId=1074;interruptedJobs=$interrupted;action='Archive interrupted attempts and resume identical frozen jobs';numericalImplementationUnchanged=$true;computeDeadlineUtc='2026-09-17T03:46:51Z'} | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $recovery 'incident.json')
$lock=Join-Path $runRoot 'supervisor.lock'
if (Test-Path -LiteralPath $lock) { Remove-Item -LiteralPath $lock }
$startup=New-CimInstance -ClassName Win32_ProcessStartup -ClientOnly -Property @{ShowWindow=[uint16]0;CreateFlags=[uint32]16777216}
$line='D:\XieZhenyu\envs\stewart-abfd\Scripts\python.exe D:\XieZhenyu\stewart-simscape-diffusion\study\launch_supervisor.py frozen_v1'
$created=Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{CommandLine=$line;CurrentDirectory='D:\XieZhenyu\stewart-simscape-diffusion';ProcessStartupInformation=$startup}
$created | Select-Object ReturnValue,ProcessId | ConvertTo-Json
if ($created.ReturnValue -ne 0) { throw 'Detached supervisor launch failed.' }
