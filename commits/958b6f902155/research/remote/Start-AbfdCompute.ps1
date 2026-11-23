param(
    [Parameter(Mandatory=$true)][ValidateSet('F-pilot','F-data')][string]$Stage,
    [Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_-]+$')][string]$RunName,
    [ValidateRange(1,100000)][int]$SolverMaxIterations=2000,
    [ValidatePattern('^[A-Za-z0-9._-]+$')][string]$HostAlias='workstation-p3',
    [switch]$Resume
)
$ErrorActionPreference='Stop'
$project='D:/XieZhenyu/stewart-simscape-abfd'
$run="$project/results/abfd/compute/$RunName"
$attempt=[DateTime]::UtcNow.ToString('yyyyMMddTHHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8)
$attemptDir="$run/attempts/$attempt"
$job=@{executable='D:/matlab2019a/bin/matlab.exe'; workingDirectory=$project; outputDirectory=$attemptDir; lockFile="$run/compute.lock"; arguments=@('-batch',"run('$attemptDir/run_stage.m')")}|ConvertTo-Json -Depth 5 -Compress
$code=@'
$ErrorActionPreference='Stop'
$run='__RUN__'
if(Test-Path -LiteralPath $run) {
    if('__RESUME__' -ne 'True'){throw 'Run exists; choose a new RunName or explicit -Resume'}
    $old=Get-Content -LiteralPath ($run+'/compute_manifest.json') -Raw|ConvertFrom-Json
    if($old.stage -ne '__STAGE__' -or $old.solverMaxIterations -ne __ITER__){throw 'Compute resume configuration mismatch'}
    if(Test-Path -LiteralPath ($run+'/compute.lock')){throw 'Compute lock exists; inspect the running job before resuming'}
} else {
    if('__RESUME__' -eq 'True'){throw 'Cannot resume a missing compute run'}
    New-Item -ItemType Directory -Path $run|Out-Null
    @{stage='__STAGE__';solverMaxIterations=__ITER__}|ConvertTo-Json|Set-Content -LiteralPath ($run+'/compute_manifest.json') -Encoding UTF8
}
New-Item -ItemType Directory -Path '__ATTEMPT__'|Out-Null
"cd('D:/XieZhenyu/stewart-simscape-abfd'); addpath('research'); addpath('research/remote'); run_abfd_compute_stage('__STAGE__','__RUN__',__ITER__);"|Set-Content -LiteralPath '__ATTEMPT__/run_stage.m' -Encoding ascii
'__JOB__'|Set-Content -LiteralPath '__ATTEMPT__/job.json' -Encoding UTF8
& 'C:/Program Files/Bitvise SSH Server/bvRun.exe' -brj -det -ncw 'D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe' 'D:/XieZhenyu/stewart-simscape-abfd/research/remote/run_remote_job.py' '__ATTEMPT__/job.json'
if($LASTEXITCODE -ne 0){throw "Detached compute launch failed: $LASTEXITCODE"}
'__ATTEMPT__/status.json'
'@
$code=$code.Replace('__RUN__',$run).Replace('__STAGE__',$Stage).Replace('__ITER__',[string]$SolverMaxIterations).Replace('__RESUME__',[string][bool]$Resume).Replace('__ATTEMPT__',$attemptDir).Replace('__JOB__',$job.Replace("'","''"))
$encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($code))
& ssh -o BatchMode=yes -o ConnectTimeout=8 $HostAlias powershell.exe -NoProfile -NonInteractive -OutputFormat Text -EncodedCommand $encoded
if($LASTEXITCODE -ne 0){throw "Remote compute launch failed: $LASTEXITCODE"}
