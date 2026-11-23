param([ValidateSet('validate','regression','teacher','train','inference','development','freeze','formal','report','all')][string]$Stage='all')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$python=Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
$matlab='D:\MATLAB\R2025b\bin\matlab.exe'
Push-Location $projectRoot
try {
    $freezeExists=Test-Path -LiteralPath (Join-Path $PSScriptRoot 'artifacts/freeze.json')
    if ($freezeExists -and $Stage -notin @('all','freeze','formal','report')) {
        throw 'Frozen prerequisites are immutable; start a new experiment version to change them.'
    }
    # Every entry uses the same dependency gates, durable lock and MATLAB
    # threading policy. Stage-specific shortcuts must not skip validation.
    & $python (Join-Path $PSScriptRoot 'campaign.py') --stage $Stage
    if ($LASTEXITCODE -ne 0) {throw "The campaign stopped; inspect chapter5/artifacts/campaign_status.json."}
} finally { Pop-Location }
