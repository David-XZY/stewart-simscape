param(
    [Parameter(Mandatory=$true)][ValidateSet('B','F')][string]$Kind,
    [Parameter(Mandatory=$true)][string]$DatasetPath,
    [Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_-]+$')][string]$RunName,
    [ValidateSet('nearest','mlp','diffusion')][string]$Mode='mlp',
    [string]$AuditPath,
    [string]$RemoteProject='D:/XieZhenyu/stewart-simscape-abfd',
    [ValidateRange(1,100000)][int]$Epochs=300,
    [ValidatePattern('^[A-Za-z0-9._-]+$')][string]$HostAlias='workstation-p3',
    [switch]$Resume
)
$ErrorActionPreference='Stop'
# A detached Python launcher can inherit a module path from another PowerShell.
# Load this host's bundled hashing/JSON commands without changing global settings.
Import-Module (Join-Path $PSHOME 'Modules/Microsoft.PowerShell.Utility/Microsoft.PowerShell.Utility.psd1') -ErrorAction Stop
$dataset=Get-Item -LiteralPath $DatasetPath
if($dataset.PSIsContainer -or $dataset.Extension -ne '.json'){throw 'A dataset JSON file is required'}
$audit=$null
$auditName='multimodality_audit.json'
if($Kind -eq 'B' -and $AuditPath) {
    $audit=Get-Item -LiteralPath $AuditPath
    $auditName='residual_dataset_audit.json'
}
if($Kind -eq 'F' -and $Mode -eq 'diffusion') {
    if(!$AuditPath){throw 'Diffusion requires the matching approved multimodality audit'}
    $audit=Get-Item -LiteralPath $AuditPath
}
$remoteRoot=$RemoteProject.Replace('\','/').TrimEnd('/')
if($remoteRoot -notmatch '^D:/XieZhenyu/[A-Za-z0-9_/-]+$' -or $remoteRoot.Contains('/../')) {
    throw 'RemoteProject must be an explicit project directory under D:/XieZhenyu'
}
$remoteRun="$remoteRoot/results/abfd/training/$RunName"
$hash=(Get-FileHash -LiteralPath $dataset.FullName -Algorithm SHA256).Hash
$auditHash=''; if($audit){$auditHash=(Get-FileHash -LiteralPath $audit.FullName -Algorithm SHA256).Hash}
function Invoke-RemoteScript([string]$Code) {
    # File transport avoids cmd.exe's length limit and Bitvise stdin EOF hangs.
    $directory=Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'results/abfd/remote_scripts'
    New-Item -ItemType Directory -Path $directory -Force | Out-Null
    $name=[guid]::NewGuid().ToString('N')+'.ps1'; $file=Join-Path $directory $name
    [IO.File]::WriteAllText($file,"`$ProgressPreference='SilentlyContinue'`r`n"+$Code,(New-Object Text.UTF8Encoding($true)))
    $mkdir=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes("`$ProgressPreference='SilentlyContinue'; New-Item -ItemType Directory -Path D:/XieZhenyu/deployments -Force | Out-Null"))
    & ssh -T -o BatchMode=yes -o ConnectTimeout=8 $HostAlias powershell.exe -NoProfile -NonInteractive -EncodedCommand $mkdir
    if($LASTEXITCODE -ne 0){throw 'Cannot prepare remote script directory'}
    $remoteScript='D:/XieZhenyu/deployments/'+$name
    & scp -q -o BatchMode=yes -o ConnectTimeout=8 $file ($HostAlias+':/'+$remoteScript)
    if($LASTEXITCODE -ne 0){throw 'Remote script transfer failed'}
    & ssh -T -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=15 $HostAlias powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -OutputFormat Text -File $remoteScript
    if($LASTEXITCODE -ne 0){throw "Remote command failed: $LASTEXITCODE"}
}
$meta=@{kind=$Kind; mode=$Mode; datasetSha256=$hash; auditSha256=$auditHash}|ConvertTo-Json -Compress
$guard=@'
$ErrorActionPreference='Stop'
$run='__RUN__'; $expected='__META__'|ConvertFrom-Json
if(Test-Path -LiteralPath $run) {
    if('__RESUME__' -ne 'True'){throw 'Run exists; use a new RunName or explicit -Resume'}
    $old=Get-Content -LiteralPath ($run+'/input_manifest.json') -Raw|ConvertFrom-Json
    foreach($name in @('kind','mode','datasetSha256','auditSha256')) {
        if($old.$name -ne $expected.$name){throw ('Resume input mismatch: '+$name)}
    }
    if(Test-Path -LiteralPath ($run+'/training.lock')){throw 'Training lock exists; inspect the running job before resuming'}
} else {
    if('__RESUME__' -eq 'True'){throw 'Cannot resume a missing run'}
    New-Item -ItemType Directory -Path $run | Out-Null
    '__META__'|Set-Content -LiteralPath ($run+'/input_manifest.json') -Encoding UTF8
}
'@
Invoke-RemoteScript $guard.Replace('__RUN__',$remoteRun).Replace('__META__',$meta).Replace('__RESUME__',[string][bool]$Resume)
# Unique staging files prevent overwriting the accepted input on resume.
$upload=[guid]::NewGuid().ToString('N')
& scp -q -o BatchMode=yes $dataset.FullName ($HostAlias+':/'+$remoteRun+'/dataset-'+$upload+'.json')
if($LASTEXITCODE -ne 0){throw 'Dataset transfer failed'}
if($audit) {
    & scp -q -o BatchMode=yes $audit.FullName ($HostAlias+':/'+$remoteRun+'/audit-'+$upload+'.json')
    if($LASTEXITCODE -ne 0){throw 'Audit transfer failed'}
}
$entry='research/b_residual_model/train_residual_wrench_model.py'
$extra=@('--device','cuda')
if($Kind -eq 'B' -and $audit){$extra+=@('--dataset-audit',"$remoteRun/$auditName")}
if($Kind -eq 'F') {
    $entry='research/f_trajectory_proposal/train_trajectory_proposal.py'
    $extra+=@('--mode',$Mode)
    if($Mode -eq 'diffusion'){$extra+=@('--audit',"$remoteRun/multimodality_audit.json")}
}
if($Resume){$extra+='--resume'}
$attempt=[DateTime]::UtcNow.ToString('yyyyMMddTHHmmss')+'-'+$upload.Substring(0,8)
$attemptDir="$remoteRun/attempts/$attempt"
$job=@{executable='D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe'; workingDirectory="$remoteRun/source"; outputDirectory=$attemptDir; lockFile="$remoteRun/training.lock"; arguments=@("$remoteRun/source/$entry","$remoteRun/input_dataset.json","$remoteRun/model",'--epochs',[string]$Epochs)+$extra}|ConvertTo-Json -Depth 5 -Compress
$code=@'
$ErrorActionPreference='Stop'
function Accept-Input($staging,$target,$hash) {
    if((Get-FileHash -LiteralPath $staging -Algorithm SHA256).Hash -ne $hash){throw 'Transferred input hash mismatch'}
    if(Test-Path -LiteralPath $target) {
        if((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $hash){throw 'Existing accepted input changed'}
        Remove-Item -LiteralPath $staging
    } else { Move-Item -LiteralPath $staging -Destination $target }
}
Accept-Input '__RUN__/dataset-__UPLOAD__.json' '__RUN__/input_dataset.json' '__HASH__'
if('__AUDITHASH__') { Accept-Input '__RUN__/audit-__UPLOAD__.json' '__RUN__/__AUDITNAME__' '__AUDITHASH__' }
# Freeze only the executable training sources; resume verifies these copies.
$sourceRoot='__RUN__/source'; $sourceManifest='__RUN__/source_manifest.json'
if(Test-Path -LiteralPath $sourceManifest) {
    $saved=Get-Content -LiteralPath $sourceManifest -Raw | ConvertFrom-Json
    foreach($item in $saved.files) {
        if((Get-FileHash -LiteralPath ($sourceRoot+'/'+$item.path) -Algorithm SHA256).Hash -ne $item.sha256){throw 'Frozen training source changed'}
    }
} else {
    if(Test-Path -LiteralPath $sourceRoot){throw 'Incomplete training source snapshot; inspect before resuming'}
    $sources=@('__ENTRY__','research/common/training_io.py')
    if('__KIND__' -eq 'B'){$sources+='research/b_residual_model/startup_inertia_context.py'}
    $items=foreach($relative in $sources) {
        $destination=$sourceRoot+'/'+$relative
        New-Item -ItemType Directory -Path (Split-Path -Parent $destination) -Force | Out-Null
        Copy-Item -LiteralPath ('__ROOT__/'+$relative) -Destination $destination
        @{path=$relative;sha256=(Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash}
    }
    @{files=@($items);repositoryRevision=(git -C '__ROOT__' rev-parse HEAD);repositoryDirty=[bool](git -C '__ROOT__' status --porcelain)} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $sourceManifest -Encoding UTF8
}
New-Item -ItemType Directory -Path '__ATTEMPT__' | Out-Null
$trainingJob='__JOB__'|ConvertFrom-Json
if('__RESUME__' -eq 'True' -and !(Test-Path -LiteralPath '__RUN__/model/checkpoint.pt')) {
    if((Test-Path -LiteralPath '__RUN__/model') -and (Get-ChildItem -LiteralPath '__RUN__/model' -Force | Measure-Object).Count){throw 'Model outputs exist without a resumable checkpoint'}
    # Resume interrupted deployment; no optimizer state has ever been saved.
    $trainingJob.arguments=@($trainingJob.arguments | Where-Object {$_ -ne '--resume'})
}
$trainingJob | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath '__ATTEMPT__/job.json' -Encoding UTF8
& 'C:/Program Files/Bitvise SSH Server/bvRun.exe' -brj -det -ncw 'D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe' '__ROOT__/research/remote/run_remote_job.py' '__ATTEMPT__/job.json'
if($LASTEXITCODE -ne 0){throw "Detached launch failed: $LASTEXITCODE"}
'__ATTEMPT__/status.json'
'@
$code=$code.Replace('__RUN__',$remoteRun).Replace('__ROOT__',$remoteRoot).Replace('__ENTRY__',$entry).Replace('__RESUME__',[string][bool]$Resume).Replace('__HASH__',$hash).Replace('__AUDITHASH__',$auditHash).Replace('__UPLOAD__',$upload).Replace('__ATTEMPT__',$attemptDir).Replace('__JOB__',$job.Replace("'","''"))
$code=$code.Replace('__AUDITNAME__',$auditName).Replace('__KIND__',$Kind)
Invoke-RemoteScript $code

