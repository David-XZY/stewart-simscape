param(
    [ValidateSet('deploy','launch','train','status','collect')][string]$Action,
    [ValidateSet('audit','select','prepare','test','cache-test','pilot','run')][string]$Stage='run',
    [string]$RunName='chapter3_v3'
)
$ErrorActionPreference='Stop'
if ($RunName -notmatch '^[a-zA-Z0-9_-]+$') { throw 'Invalid run name' }
$localRoot=Split-Path $PSScriptRoot -Parent
$remoteRoot='D:/XieZhenyu/stewart-simscape-diffusion'
function Remote([string]$Script) {
    $Script="`$ProgressPreference='SilentlyContinue'; `$ErrorActionPreference='Stop'; "+$Script
    $encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($Script))
    & ssh -o BatchMode=yes -o ConnectTimeout=10 ws powershell.exe -NoProfile -NonInteractive -OutputFormat Text -EncodedCommand $encoded
    if ($LASTEXITCODE -ne 0) { throw "SSH exit $LASTEXITCODE" }
}
if ($Action -eq 'deploy') {
    & scp -r $PSScriptRoot "ws:$remoteRoot/"
    if ($LASTEXITCODE -ne 0) { throw 'Transfer failed' }
} elseif ($Action -eq 'launch') {
    Remote @"
`$out='$remoteRoot/results/chapter3/$RunName'
New-Item -ItemType Directory -Force -Path `$out | Out-Null
`$args=@('$remoteRoot/chapter3/run_chapter3.py','--run','$RunName','--stage','$Stage')
`$p=Start-Process 'D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe' -ArgumentList `$args -WorkingDirectory '$remoteRoot' -WindowStyle Hidden -PassThru -RedirectStandardOutput (`$out+'/$Stage-controller.log') -RedirectStandardError (`$out+'/$Stage-controller-error.log')
@{pid=`$p.Id;stage='$Stage';started=(Get-Date).ToString('o')} | ConvertTo-Json -Compress | Set-Content -Encoding UTF8 (`$out+'/$Stage-launch.json')
Get-Content (`$out+'/$Stage-launch.json')
"@
} elseif ($Action -eq 'train') {
    Remote @"
`$env:OMP_NUM_THREADS='1'; `$env:MKL_NUM_THREADS='1'
& 'D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe' '$remoteRoot/diffusion/train_diffusion.py' '$remoteRoot/results/chapter3/$RunName/dataset.json' '$remoteRoot/results/chapter3/$RunName/training' --device cuda
if (`$LASTEXITCODE -ne 0) { throw 'Training failed' }
"@
} elseif ($Action -eq 'status') {
    Remote @"
`$out='$remoteRoot/results/chapter3/$RunName'
foreach (`$name in @('progress.json','complete.json','core_tests.json','safety_stop.json')) { if(Test-Path (`$out+'/'+`$name)){ Get-Content (`$out+'/'+`$name) } }
foreach (`$name in @('$Stage-controller-error.log','${Stage}_process.log')) { if(Test-Path (`$out+'/'+`$name)){ Get-Content (`$out+'/'+`$name) -Tail 8 } }
if(Test-Path (`$out+'/active.json')) { `$a=Get-Content (`$out+'/active.json') -Raw | ConvertFrom-Json; @{job=`$a.job.jobId;phase=`$a.phase;started=`$a.startedEpoch} | ConvertTo-Json -Compress }
"@
} else {
    $target=Join-Path $localRoot 'results/chapter3'
    New-Item -ItemType Directory -Force -Path $target | Out-Null
    & scp -r "ws:$remoteRoot/results/chapter3/$RunName" $target
    if ($LASTEXITCODE -ne 0) { throw 'Collection failed' }
}
