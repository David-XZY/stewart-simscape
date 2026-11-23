$ErrorActionPreference='Stop'
$ProgressPreference='SilentlyContinue'
$r=[IO.Path]::GetFullPath('D:/XieZhenyu/stewart-simscape-diffusion/results/study/frozen_v1')
$running=@(Get-CimInstance Win32_Process | Where-Object { $_.Name -match 'python|MATLAB' -and $_.CommandLine -match 'sgWorker\(|study[/\\](run_study|launch_supervisor).py' })
if ($running.Count) { throw 'Study processes remain.' }
$a=Join-Path $r 'environment_recovery_encoding_20260916'
if (Test-Path -LiteralPath (Join-Path $a 'incident.json')) { throw 'Already recovered.' }
New-Item -ItemType Directory -Force -Path $a | Out-Null
foreach($name in @('progress.json','run_control.json','active_2.json')) { Copy-Item -LiteralPath (Join-Path $r $name) -Destination (Join-Path $a $name) }
$f=Join-Path $r 'jobs/physical_a3_p20__diff_ihsid__k08__r1/result.json'
Copy-Item -LiteralPath $f -Destination (Join-Path $a 'original_bom_record.json')
$text=[IO.File]::ReadAllText($f)
$record=$text | ConvertFrom-Json
if($record.jobId -ne 'physical_a3_p20__diff_ihsid__k08__r1' -or $record.status -ne 'worker_process_exit') { throw 'Unexpected record.' }
[IO.File]::WriteAllText($f,$text,(New-Object Text.UTF8Encoding($false)))
$active=Get-Content -Raw (Join-Path $r 'active_2.json') | ConvertFrom-Json
if($active.jobId -ne 'physical_a3_p07__geo_ihsid__k08__r1') { throw 'Unexpected interrupted job.' }
$job=[IO.Path]::GetFullPath((Join-Path $r ('jobs/'+$active.jobId)))
$dest=[IO.Path]::GetFullPath((Join-Path $job 'interrupted_encoding_20260916'))
if(-not $dest.StartsWith($r+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Outside run root.' }
New-Item -ItemType Directory -Force -Path $dest | Out-Null
foreach($name in @('result.json','result.mat','online.mat','online_progress.json','first_audit.mat')) {
    $source=[IO.Path]::GetFullPath((Join-Path $job $name))
    if(-not $source.StartsWith($r+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Outside run root.' }
    if(Test-Path -LiteralPath $source) { Move-Item -LiteralPath $source -Destination (Join-Path $dest $name) }
}
$incident=@{reason='Recovery record had a UTF-8 BOM rejected by MATLAB R2019a jsondecode';action='Remove BOM without changing record content; archive and resume the other interrupted job';interruptedJob=$active.jobId;recoveredUtc=[DateTime]::UtcNow.ToString('o');numericalImplementationUnchanged=$true}
[IO.File]::WriteAllText((Join-Path $a 'incident.json'),($incident | ConvertTo-Json),(New-Object Text.UTF8Encoding($false)))
$startup=New-CimInstance -ClassName Win32_ProcessStartup -ClientOnly -Property @{ShowWindow=[uint16]0;CreateFlags=[uint32]16777216}
Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{CommandLine='D:\XieZhenyu\envs\stewart-abfd\Scripts\python.exe D:\XieZhenyu\stewart-simscape-diffusion\study\launch_supervisor.py frozen_v1';CurrentDirectory='D:\XieZhenyu\stewart-simscape-diffusion';ProcessStartupInformation=$startup} | Select-Object ReturnValue,ProcessId | ConvertTo-Json
