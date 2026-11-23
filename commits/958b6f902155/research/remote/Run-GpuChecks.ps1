$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$root = 'D:\XieZhenyu'
$project = Join-Path $root 'stewart-simscape-abfd'
$python = Join-Path $root 'envs\stewart-abfd\Scripts\python.exe'
$env:TEMP = Join-Path $root 'tmp'
$env:TMP = $env:TEMP
$checks = @(
    @{name='gpu'; args=@('research\remote\run_gpu_preflight.py',(Join-Path $root 'logs\gpu_preflight.json'))},
    @{name='contracts'; args=@('research\tests\test_training_contracts.py')},
    @{name='dependencies'; args=@('-m','pip','check')},
    @{name='B_cli'; args=@('research\b_residual_model\train_residual_wrench_model.py','--help')},
    @{name='F_cli'; args=@('research\f_trajectory_proposal\train_trajectory_proposal.py','--help')}
)
$results = @()
foreach ($check in $checks) {
    $p = Start-Process -FilePath $python -ArgumentList $check.args -WorkingDirectory $project -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput (Join-Path $root ('logs\check_'+$check.name+'.stdout.log')) -RedirectStandardError (Join-Path $root ('logs\check_'+$check.name+'.stderr.log'))
    $results += @{name=$check.name; exitCode=$p.ExitCode; passed=($p.ExitCode -eq 0)}
    $results | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $root 'logs\training_environment_checks.json') -Encoding UTF8
    if ($p.ExitCode -ne 0) { throw ('Environment check failed: '+$check.name) }
}
& $python -m pip freeze --all | Set-Content -LiteralPath (Join-Path $root 'logs\requirements-training.lock.txt') -Encoding ASCII
$results | ConvertTo-Json
