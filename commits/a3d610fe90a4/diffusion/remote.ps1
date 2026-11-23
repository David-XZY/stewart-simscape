param(
    [ValidateSet('deploy','run','pipeline','status','collect')][string]$Action,
    [string]$Entry='test_diffusion_core',
    [string]$RunName='pilot_v1'
)
$ErrorActionPreference='Stop'
$localRoot=Split-Path $PSScriptRoot -Parent
$remoteRoot='D:/XieZhenyu/stewart-simscape-diffusion'
if ($Entry -notmatch '^[a-zA-Z0-9_]+$' -or $RunName -notmatch '^[a-zA-Z0-9_-]+$') { throw 'Invalid entry/run name' }
function Invoke-Remote([string]$Script) {
    $Script="`$ProgressPreference='SilentlyContinue'; "+$Script
    $encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($Script))
    & ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=30 -o ServerAliveCountMax=10 ws powershell.exe -NoProfile -NonInteractive -OutputFormat Text -EncodedCommand $encoded
    if ($LASTEXITCODE -ne 0) { throw "Remote operation failed: $LASTEXITCODE" }
}
if ($Action -eq 'deploy') {
    $archive=Join-Path $localRoot 'results/diffusion_source.zip'
    & python (Join-Path $PSScriptRoot 'source_archive.py') $localRoot $archive
    if ($LASTEXITCODE -ne 0) { throw 'Archive failed' }
    Invoke-Remote "New-Item -ItemType Directory -Force -Path '$remoteRoot' | Out-Null"
    & scp $archive "ws:$remoteRoot/source.zip"
    if ($LASTEXITCODE -ne 0) { throw 'Source transfer failed' }
    Invoke-Remote "Expand-Archive -LiteralPath '$remoteRoot/source.zip' -DestinationPath '$remoteRoot' -Force"
} elseif ($Action -eq 'pipeline') {
    $kind='pilot'; if ($RunName.StartsWith('formal')) { $kind='formal' }
    $script=@"
`$ErrorActionPreference='Stop'
`$env:STEWART_DEPENDENCY_ROOT='D:/XieZhenyu/stewart-simscape-abfd'
`$env:OMP_NUM_THREADS='2'; `$env:MKL_NUM_THREADS='2'
& 'D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe' '$remoteRoot/diffusion/run_pipeline.py' --kind '$kind' --run '$RunName'
exit `$LASTEXITCODE
"@
    Invoke-Remote $script
} elseif ($Action -eq 'run') {
    $script=@"
`$ErrorActionPreference='Stop'
`$env:STEWART_DEPENDENCY_ROOT='D:/XieZhenyu/stewart-simscape-abfd'
`$env:OMP_NUM_THREADS='2'; `$env:MKL_NUM_THREADS='2'
`$out='$remoteRoot/results/diffusion/$RunName'
New-Item -ItemType Directory -Force -Path `$out | Out-Null
`$matlabArgs=@('-wait','-batch', '"addpath(''$remoteRoot/diffusion''); $Entry"', '-logfile', '"'+`$out+'/console.log"')
`$p=Start-Process 'D:/matlab2019a/bin/matlab.exe' -ArgumentList `$matlabArgs -WorkingDirectory '$remoteRoot' -WindowStyle Hidden -PassThru -RedirectStandardOutput (`$out+'/process.log') -RedirectStandardError (`$out+'/error.log')
[pscustomobject]@{pid=`$p.Id;entry='$Entry';started=(Get-Date).ToString('o')} | ConvertTo-Json -Compress | Set-Content -Encoding UTF8 (`$out+'/launch.json')
Get-Content (`$out+'/launch.json')
`$p.WaitForExit()
exit `$p.ExitCode
"@
    Invoke-Remote $script
} elseif ($Action -eq 'status') {
    $script=@"
`$job='$remoteRoot/results/diffusion/$RunName'
Get-Content (`$job+'/pipeline_status.json') -ErrorAction SilentlyContinue
foreach (`$i in 1,2) {
    `$file=`$job+'/worker_'+`$i+'.json'
    if(Test-Path `$file){
        `$rows=Get-Content `$file -Raw | ConvertFrom-Json
        [pscustomobject]@{worker=`$i;attempts=@(`$rows).Count;passed=@(`$rows | Where-Object passed).Count;last=`$rows[-1].taskId;lastReason=`$rows[-1].reason} | ConvertTo-Json -Compress
    }
    Get-Content (`$job+'/worker_'+`$i+'_process.log') -Tail 2 -ErrorAction SilentlyContinue
}
if(Test-Path (`$job+'/benchmark.json')){
    `$rows=Get-Content (`$job+'/benchmark.json') -Raw | ConvertFrom-Json
    `$rows | Group-Object method,passed | Select-Object Name,Count | ConvertTo-Json -Compress
}
foreach (`$i in 1,2) {
    `$file=`$job+'/benchmark_worker_'+`$i+'.json'
    if(Test-Path `$file){
        `$rows=Get-Content `$file -Raw | ConvertFrom-Json
        [pscustomobject]@{benchmarkWorker=`$i;comparisons=@(`$rows).Count;passed=@(`$rows | Where-Object passed).Count;last=`$rows[-1].taskId;method=`$rows[-1].method;reason=`$rows[-1].reason} | ConvertTo-Json -Compress
    }
}
Get-Content (`$job+'/training_process.log') -Tail 2 -ErrorAction SilentlyContinue
Get-Content (`$job+'/error.log') -Tail 3 -ErrorAction SilentlyContinue
exit 0
"@
    Invoke-Remote $script
} else {
    $target=Join-Path $localRoot 'results/remote/diffusion'
    New-Item -ItemType Directory -Force -Path $target | Out-Null
    & scp -r "ws:$remoteRoot/results/diffusion/$RunName" $target
    if ($LASTEXITCODE -ne 0) { throw 'Result collection failed' }
}
