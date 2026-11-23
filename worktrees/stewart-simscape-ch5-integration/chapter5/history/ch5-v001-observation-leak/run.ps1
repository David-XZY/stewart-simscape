param([ValidateSet('validate','regression','teacher','train','inference','development','freeze','formal','report','all')][string]$Stage='all')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$python=Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
$matlab='D:\MATLAB\R2025b\bin\matlab.exe'
Push-Location $projectRoot
try {
    if ($Stage -eq 'all') {
        & $python (Join-Path $PSScriptRoot 'campaign.py') --stage all
        if ($LASTEXITCODE -ne 0) {throw "The campaign stopped; inspect chapter5/artifacts/campaign_status.json."}
        return
    }
    $stages=if ($Stage -eq 'all') {@('validate','regression','teacher','train','inference','development','freeze','formal','report')} else {@($Stage)}
    foreach ($currentStage in $stages) {
        switch ($currentStage) {
            'train' { & $python (Join-Path $PSScriptRoot 'train.py') }
            'freeze' { & $python (Join-Path $PSScriptRoot 'freeze.py') }
            'report' { & $python (Join-Path $PSScriptRoot 'report.py') }
            default {
                $expression = switch ($currentStage) {
                    'regression' { "addpath('chapter5');ch5Init;run_13_strict_clf_cbf_qp_simscape('', 'chapter5/artifacts/regression',struct('sampleTime',0.01));" }
                    'inference' { "addpath('chapter5');ch5ValidateInference;" }
                    default { "addpath('chapter5');ch5RunStage('$currentStage');" }
                }
                & $matlab -batch $expression -logfile (Join-Path $PSScriptRoot "$currentStage.log")
            }
        }
        if ($LASTEXITCODE -ne 0) { throw "Stage $currentStage failed with exit code $LASTEXITCODE; later stages were not executed." }
    }
} finally { Pop-Location }
