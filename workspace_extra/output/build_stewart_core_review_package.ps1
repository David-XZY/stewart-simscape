[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$workspaceRoot = 'D:\NPU\Matlab_code'
$sourceRoot = Join-Path $workspaceRoot 'stewart-simscape'
$outputRoot = Join-Path $workspaceRoot 'output'
$packageName = 'Stewart_全流程核心代码_20260904'
$packageRoot = Join-Path $outputRoot $packageName
$zipPath = "$packageRoot.zip"
$externalReport = Join-Path $outputRoot "${packageName}_交付验证.txt"
$matlabHelper = Join-Path $outputRoot 'export_and_validate_stewart_core_review.m'
$baseCommit = 'ae06b7508151aea2c704d4b1a204bd9fb45e3347'
$maxZipBytes = 50000000L

if (-not (Test-Path -LiteralPath $sourceRoot -PathType Container)) {
    throw "Source repository not found: $sourceRoot"
}
foreach ($target in @($packageRoot, $zipPath, $externalReport)) {
    if (Test-Path -LiteralPath $target) {
        throw "Target already exists; refusing to overwrite: $target"
    }
}
if (-not (Test-Path -LiteralPath $matlabHelper -PathType Leaf)) {
    throw "MATLAB helper not found: $matlabHelper"
}

$initialStatus = @(& git -C $sourceRoot status --porcelain=v1)
$headCommit = (& git -C $sourceRoot rev-parse HEAD).Trim()
$branchName = (& git -C $sourceRoot branch --show-current).Trim()
if ($headCommit -ne $baseCommit) {
    throw "Unexpected HEAD commit: $headCommit"
}

$traceRoot = Join-Path $packageRoot '00_追溯与清单'
$codeRoot = Join-Path $packageRoot '01_核心代码\stewart-simscape'
$resultRoot = Join-Path $packageRoot '02_精选结果'
$null = New-Item -ItemType Directory -Path $traceRoot, $codeRoot, $resultRoot -Force

$sourceMap = [System.Collections.Generic.List[object]]::new()

function Add-SourceMapRow {
    param(
        [string]$Category,
        [string]$Source,
        [string]$Destination
    )
    $sourceMap.Add([pscustomobject]@{
        Category = $Category
        Source = $Source
        PackagePath = [IO.Path]::GetRelativePath($packageRoot, $Destination)
    })
}

function Add-ArtifactFile {
    param(
        [string]$Source,
        [string]$Destination,
        [string]$Category
    )
    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) {
        throw "Required source file missing: $Source"
    }
    $parent = Split-Path -Parent $Destination
    $null = New-Item -ItemType Directory -Path $parent -Force
    Copy-Item -LiteralPath $Source -Destination $Destination
    Add-SourceMapRow -Category $Category -Source $Source -Destination $Destination
}

function Add-ArtifactDirectory {
    param(
        [string]$SourceDirectory,
        [string]$DestinationDirectory,
        [string]$Category
    )
    if (-not (Test-Path -LiteralPath $SourceDirectory -PathType Container)) {
        throw "Required source directory missing: $SourceDirectory"
    }
    $files = Get-ChildItem -LiteralPath $SourceDirectory -File -Recurse -Force | Where-Object {
        $_.FullName -notmatch '[\\/](slprj|\.SimulinkCache|\.SimulinkCode)[\\/]' -and
        $_.Extension -ne '.slxc'
    }
    foreach ($file in $files) {
        $relative = [IO.Path]::GetRelativePath($SourceDirectory, $file.FullName)
        $destination = Join-Path $DestinationDirectory $relative
        Add-ArtifactFile -Source $file.FullName -Destination $destination -Category $Category
    }
}

function Write-Utf8File {
    param([string]$Path, [string]$Content, [string]$Category = '生成说明')
    $parent = Split-Path -Parent $Path
    $null = New-Item -ItemType Directory -Path $parent -Force
    [IO.File]::WriteAllText($Path, $Content, [Text.UTF8Encoding]::new($false))
    Add-SourceMapRow -Category $Category -Source 'generated during packaging' -Destination $Path
}

function Invoke-GitRawOutput {
    param([string[]]$Arguments, [string]$Destination)
    $startInfo = [Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = 'git'
    $startInfo.UseShellExecute = $false
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    foreach ($argument in $Arguments) {
        $null = $startInfo.ArgumentList.Add($argument)
    }
    $process = [Diagnostics.Process]::Start($startInfo)
    $stream = [IO.File]::Create($Destination)
    try {
        $process.StandardOutput.BaseStream.CopyTo($stream)
    } finally {
        $stream.Dispose()
    }
    $standardError = $process.StandardError.ReadToEnd()
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) {
        throw "Git raw output failed: $standardError"
    }
}

# Core project files and relative directory layout.
foreach ($relative in @('LICENSE', 'Stewartsimscape.prj', 'readme.org')) {
    Add-ArtifactFile -Source (Join-Path $sourceRoot $relative) `
        -Destination (Join-Path $codeRoot $relative) -Category '工程入口'
}
Add-ArtifactDirectory -SourceDirectory (Join-Path $sourceRoot '.SimulinkProject') `
    -DestinationDirectory (Join-Path $codeRoot '.SimulinkProject') -Category '工程配置'
foreach ($relative in @('src', 'matlab', 'mat')) {
    Add-ArtifactDirectory -SourceDirectory (Join-Path $sourceRoot $relative) `
        -DestinationDirectory (Join-Path $codeRoot $relative) -Category 'Simscape模型与资产'
}

# Active trajectory-planning line only.
foreach ($relative in @(
    'opt_minimal\README.md',
    'opt_minimal\run_01_ihsid_trajectory.m'
)) {
    Add-ArtifactFile -Source (Join-Path $sourceRoot $relative) `
        -Destination (Join-Path $codeRoot $relative) -Category '轨迹规划核心'
}
foreach ($relative in @(
    'opt_minimal\core',
    'opt_minimal\ihsid',
    'opt_minimal\validation',
    'opt_minimal\tools',
    'opt_minimal\examples\ihsid_40x20_limited_memory'
)) {
    Add-ArtifactDirectory -SourceDirectory (Join-Path $sourceRoot $relative) `
        -DestinationDirectory (Join-Path $codeRoot $relative) -Category '轨迹规划核心'
}

# Main control entries plus the current uncommitted observer experiment.
Add-ArtifactFile -Source (Join-Path $sourceRoot 'controller\README.md') `
    -Destination (Join-Path $codeRoot 'controller\README.md') -Category '运动控制核心'
foreach ($runName in @(
    'run_02_simscape_length_control.m',
    'run_03_simscape_length_cascade_control.m',
    'run_04_simscape_pose_length_control.m',
    'run_13_strict_clf_cbf_qp_simscape.m',
    'run_15_observed_state_control_ablation.m'
)) {
    $relative = Join-Path 'controller\runs' $runName
    Add-ArtifactFile -Source (Join-Path $sourceRoot $relative) `
        -Destination (Join-Path $codeRoot $relative) -Category '运动控制入口'
}
foreach ($relative in @(
    'controller\simscape_tracking',
    'controller\strict_clf_cbf_qp',
    'controller\lqi_schedule_comparison'
)) {
    Add-ArtifactDirectory -SourceDirectory (Join-Path $sourceRoot $relative) `
        -DestinationDirectory (Join-Path $codeRoot $relative) -Category '运动控制核心'
}

# Selected trajectory result (also retained in the runnable example path).
$trajectorySource = Join-Path $sourceRoot 'opt_minimal\examples\ihsid_40x20_limited_memory'
$trajectoryResult = Join-Path $resultRoot '01_轨迹规划'
Add-ArtifactDirectory -SourceDirectory $trajectorySource `
    -DestinationDirectory $trajectoryResult -Category '代表性轨迹结果'

# One accepted physical force-input result.
$controlResult = Join-Path $resultRoot '02_运动控制'
foreach ($fileName in @(
    'result_simscape_pose_force_control_20260614_174539.mat',
    'summary_simscape_pose_force_control_20260614_174539.txt',
    'tracking_simscape_pose_force_control_20260614_174539.png'
)) {
    Add-ArtifactFile -Source (Join-Path $sourceRoot "results\controller\$fileName") `
        -Destination (Join-Path $controlResult $fileName) -Category '代表性控制结果'
}

# Current observer experiment: conclusions and four comparison views only.
$experimentSource = Join-Path $sourceRoot 'results\reports\observed_state_control_ablation_final_v2_20260901'
$experimentResult = Join-Path $resultRoot '03_当前观测器实验'
Add-ArtifactFile -Source (Join-Path $experimentSource 'observed_state_ablation_summary.csv') `
    -Destination (Join-Path $experimentResult 'observed_state_ablation_summary.csv') `
    -Category '当前实验结论'
$experimentPrefixes = @(
    'strict_qp_legacy_direct_ideal_force',
    'strict_qp_legacy_direct_nonideal_force',
    'strict_qp_observed_current_state_ideal_force',
    'strict_qp_observed_current_state_nonideal_force'
)
foreach ($prefix in $experimentPrefixes) {
    foreach ($extension in @('.txt', '.png')) {
        $fileName = "$prefix$extension"
        Add-ArtifactFile -Source (Join-Path $experimentSource $fileName) `
            -Destination (Join-Path $experimentResult $fileName) -Category '当前实验结论'
    }
}

$readme = @"
# Stewart 全流程核心代码审阅包

本包用于审阅 6-UCU Stewart 平台从受约束轨迹规划到 Simscape 运动控制的核心流程。它保留原工程相对目录，但不携带约 292 MB 的第三方求解器库，因此不是换机后开箱即跑的完整发行版。

## 建议审阅顺序

1. `01_核心代码/stewart-simscape/opt_minimal/run_01_ihsid_trajectory.m`：IHSID 轨迹规划、动力学和工程后验验收。
2. `controller/runs/run_02_simscape_length_control.m`：物理力输入的位姿控制。
3. `controller/runs/run_03_simscape_length_cascade_control.m`：理想腿长串级控制。
4. `controller/runs/run_04_simscape_pose_length_control.m`：位姿外环修正的理想腿长控制。
5. `controller/runs/run_13_strict_clf_cbf_qp_simscape.m` 与 `run_15_observed_state_control_ablation.m`：当前严格 QP、状态观测器和当前状态计算力矩实验。

## 运行环境边界

- 已验证环境：MATLAB R2025b、Simulink、Simscape（本包以代码和结果审阅为目标）。
- 重新求解 IHSID 轨迹需另行提供 CasADi 3.7.2、IPOPT 和 MA27，并按 `setupCasadiIpoptMa27.m` 配置路径。
- 若完整重放 Run02 的对比验收，需要先生成或提供通过验收的 Run03 基准结果。
- 所有指标均为 Simscape 仿真结果，不代表实机闭环、HIL 或硬件安全验收。

## 成熟度说明

- IHSID 标准轨迹和 `20260614_174539` 物理力输入控制为已通过的代表性结果。
- 当前观测器路径来自未提交工作区，代码按现状保留；最终消融 `passed=0`，不可作为默认控制器结论。
- 详细筛选、来源和哈希见 `00_追溯与清单/`。
"@
Write-Utf8File -Path (Join-Path $packageRoot 'README.md') -Content $readme

$conclusions = @"
# 核心仿真结论

## 轨迹规划

- 标准 IHSID 40+20 区间、7.5 s 轨迹求解成功，`engineeringTrajectoryPassed=1`、`simscapePublishPassed=1`。
- 记录的最小间隙和最终装配间隙均为 5 mm；5 mm 是末端装配目标，不应表述为全程统一安全距离。
- 最大支链力约 1094.38 N，最小奇异值约 0.2846，工程后验通过。

## 代表性物理力输入控制

- `result_simscape_pose_force_control_20260614_174539.mat` 记录 `overallPassed=1`。
- 最大平移峰值误差约 0.0275 mm，最大转角峰值误差约 0.00399 deg，最大控制力约 1097.49 N。
- 该结果属于 Simscape 理想力输入仿真，不等同于真实执行器闭环性能。

## 当前状态观测器实验

- 理想力和非理想力两种观测状态路径均为 `passed=0`。
- 完整控制步 P95 分别约 11.42 ms 和 11.13 ms，超过 10 ms 门槛。
- 两种场景均出现跟踪、状态/碰撞裕度不通过；虽然没有 fallback、不可行或求解器错误，但不能据此判定控制器通过。
- 当前结论是保留旧路径为默认方案，重新设计当前状态计算力矩与固定 LQI 的组合。
"@
Write-Utf8File -Path (Join-Path $packageRoot 'CONCLUSIONS.md') -Content $conclusions

$selection = @"
# 筛选与排除说明

## 收录

- IHSID 活动主线、标准参考轨迹及后验检查。
- Run02/03/04 主线控制入口、Simscape 模型和模型资产。
- Run13/15 及其严格 QP、LQI、状态观测器和当前状态计算力矩依赖。
- 一套已通过的物理力输入控制 MAT/TXT/PNG 与不超过 1000 行的精简 CSV。
- 当前观测器最终消融的两场景、两控制路径汇总和可视化结论。

## 排除

- `lib/` 中 CasADi/IPOPT/MA27/BLASFEO 等约 292 MB 第三方依赖。
- `.git`、缓存、`slprj`、`*.slxc`、历史 ZIP 和完整历史结果目录。
- 观测器消融大体积 MAT、逐点详细 CSV、重复调参和中间运行。
- PWM、UKF、因子图、FATROP、旧 HS/DMS 比较、论文及 Office 汇报材料。
"@
Write-Utf8File -Path (Join-Path $traceRoot 'SELECTION_AND_EXCLUSIONS.md') -Content $selection

$statusText = @"
SOURCE STATE
sourceRoot: $sourceRoot
baseCommit: $baseCommit
headCommit: $headCommit
branch: $branchName
capturedAt: $([datetime]::Now.ToString('yyyy-MM-dd HH:mm:ss zzz'))

git status --short --branch:
$((& git -C $sourceRoot status --short --branch) -join [Environment]::NewLine)

git diff --stat:
$((& git -C $sourceRoot diff --stat) -join [Environment]::NewLine)

Boundary: current observer/current-state code is an uncommitted experimental snapshot.
"@
Write-Utf8File -Path (Join-Path $traceRoot 'SOURCE_STATE.txt') -Content $statusText -Category '追溯信息'

$untrackedExperimentFiles = @(& git -C $sourceRoot ls-files --others --exclude-standard -- `
    'controller/runs/run_15_observed_state_control_ablation.m' `
    'controller/simscape_tracking/*.m') | Where-Object {
        $_ -match '(run_15_observed_state_control_ablation|CurrentState|currentState|PoseInnovation|poseInnovation|Luenberger|measuredStateAdapter)'
    }
Write-Utf8File -Path (Join-Path $traceRoot 'UNTRACKED_EXPERIMENT_FILES.txt') `
    -Content (($untrackedExperimentFiles -join [Environment]::NewLine) + [Environment]::NewLine) `
    -Category '追溯信息'

$patchPath = Join-Path $traceRoot 'CURRENT_CORE_EXPERIMENT.patch'
$patchArguments = @(
    '-C', $sourceRoot, 'diff', '--binary', 'HEAD', '--',
    'controller/runs/run_13_strict_clf_cbf_qp_simscape.m',
    'controller/simscape_tracking/computeComputedTorqueForce.m',
    'controller/simscape_tracking/evaluateSimscapeStrictClfCbfQp.m',
    'controller/simscape_tracking/installSimscapeStrictClfCbfQpRuntime.m',
    'controller/simscape_tracking/makeSimscapePoseForceConfig.m',
    'controller/simscape_tracking/prepareSimscapeStrictClfCbfQp.m',
    'controller/simscape_tracking/strictClfCbfQpSimscapeSfun.m'
)
Invoke-GitRawOutput -Arguments $patchArguments -Destination $patchPath
Add-SourceMapRow -Category '追溯信息' -Source 'git diff --binary HEAD (selected core experiment files)' -Destination $patchPath

# Export compact data and validate MAT, CSV, PNG, M files and SLX loading.
$env:STEWART_CORE_PACKAGE_ROOT = $packageRoot
try {
    & matlab -batch "run('$($matlabHelper.Replace('\', '/'))')"
    if ($LASTEXITCODE -ne 0) {
        throw "MATLAB validation failed with exit code $LASTEXITCODE"
    }
} finally {
    Remove-Item Env:STEWART_CORE_PACKAGE_ROOT -ErrorAction SilentlyContinue
}
Add-SourceMapRow -Category '代表性控制结果' -Source 'generated from representative control MAT' `
    -Destination (Join-Path $controlResult 'representative_pose_force_control_compact_1000.csv')
Add-SourceMapRow -Category '验证记录' -Source 'MATLAB validation' `
    -Destination (Join-Path $traceRoot 'MATLAB_VALIDATION.txt')

$sourceMapPath = Join-Path $traceRoot 'source_mapping.csv'
$sourceMap | Sort-Object Category, PackagePath | Export-Csv -LiteralPath $sourceMapPath `
    -NoTypeInformation -Encoding utf8

# Required item and exclusion checks.
$requiredRelativePaths = @(
    'README.md',
    'CONCLUSIONS.md',
    '01_核心代码\stewart-simscape\opt_minimal\run_01_ihsid_trajectory.m',
    '01_核心代码\stewart-simscape\controller\runs\run_02_simscape_length_control.m',
    '01_核心代码\stewart-simscape\controller\runs\run_03_simscape_length_cascade_control.m',
    '01_核心代码\stewart-simscape\controller\runs\run_04_simscape_pose_length_control.m',
    '01_核心代码\stewart-simscape\controller\runs\run_13_strict_clf_cbf_qp_simscape.m',
    '01_核心代码\stewart-simscape\controller\runs\run_15_observed_state_control_ablation.m',
    '01_核心代码\stewart-simscape\matlab\stewart_platform_model.slx',
    '02_精选结果\01_轨迹规划\simscape_references.mat',
    '02_精选结果\02_运动控制\result_simscape_pose_force_control_20260614_174539.mat',
    '02_精选结果\02_运动控制\representative_pose_force_control_compact_1000.csv',
    '02_精选结果\03_当前观测器实验\observed_state_ablation_summary.csv'
)
foreach ($relative in $requiredRelativePaths) {
    if (-not (Test-Path -LiteralPath (Join-Path $packageRoot $relative) -PathType Leaf)) {
        throw "Missing required package item: $relative"
    }
}

$forbiddenFiles = Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force | Where-Object {
    $_.FullName -match '[\\/](\.git|lib|slprj|\.SimulinkCache|\.SimulinkCode|unused)[\\/]' -or
    $_.Extension -in @('.slxc', '.zip', '.pptx', '.docx', '.vsdx')
}
if ($forbiddenFiles) {
    throw "Forbidden package files found: $($forbiddenFiles.FullName -join ', ')"
}

$secretPattern = '(sk-[A-Za-z0-9]{20,}|ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,}|AKIA[0-9A-Z]{16}|xox[baprs]-[A-Za-z0-9-]{20,}|-----BEGIN (RSA |EC |OPENSSH )?PRIVATE KEY-----)'
$secretMatches = @(& rg -l --hidden --pcre2 -i $secretPattern $packageRoot)
if ($LASTEXITCODE -notin @(0, 1)) {
    throw "Secret scan failed with exit code $LASTEXITCODE"
}
if ($secretMatches.Count -gt 0) {
    throw "Potential secrets found in package: $($secretMatches -join ', ')"
}

# Hash every payload file except the hash list itself.
$hashPath = Join-Path $traceRoot 'package_files_sha256.csv'
$hashRows = Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force | Where-Object {
    $_.FullName -ne $hashPath
} | Sort-Object FullName | ForEach-Object {
    [pscustomobject]@{
        PackagePath = [IO.Path]::GetRelativePath($packageRoot, $_.FullName)
        SizeBytes = $_.Length
        SHA256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
    }
}
$hashRows | Export-Csv -LiteralPath $hashPath -NoTypeInformation -Encoding utf8

# Compress and verify a clean extraction against the hash manifest.
Compress-Archive -LiteralPath $packageRoot -DestinationPath $zipPath -CompressionLevel Optimal
$zipItem = Get-Item -LiteralPath $zipPath
if ($zipItem.Length -ge $maxZipBytes) {
    throw "ZIP is too large: $($zipItem.Length) bytes"
}

$tempBase = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
$tempRoot = Join-Path $tempBase ("stewart_core_review_" + [guid]::NewGuid().ToString('N'))
$tempResolved = [IO.Path]::GetFullPath($tempRoot)
if (-not $tempResolved.StartsWith($tempBase, [StringComparison]::OrdinalIgnoreCase)) {
    throw "Unsafe temporary extraction path: $tempResolved"
}
$null = New-Item -ItemType Directory -Path $tempResolved
try {
    Expand-Archive -LiteralPath $zipPath -DestinationPath $tempResolved
    $extractedRoot = Join-Path $tempResolved $packageName
    $extractedHashPath = Join-Path $extractedRoot '00_追溯与清单\package_files_sha256.csv'
    $extractedHashes = Import-Csv -LiteralPath $extractedHashPath
    foreach ($row in $extractedHashes) {
        $extractedFile = Join-Path $extractedRoot $row.PackagePath
        if (-not (Test-Path -LiteralPath $extractedFile -PathType Leaf)) {
            throw "Extracted file missing: $($row.PackagePath)"
        }
        $actualHash = (Get-FileHash -LiteralPath $extractedFile -Algorithm SHA256).Hash
        if ($actualHash -ne $row.SHA256) {
            throw "Extracted hash mismatch: $($row.PackagePath)"
        }
    }
} finally {
    Remove-Item -LiteralPath $tempResolved -Recurse -Force
}

$finalStatus = @(& git -C $sourceRoot status --porcelain=v1)
if (($initialStatus -join "`n") -ne ($finalStatus -join "`n")) {
    throw 'Source repository status changed during packaging.'
}

$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash
$packageBytes = (Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force | `
    Measure-Object -Property Length -Sum).Sum
$verification = @"
Stewart core review package verification
packageDirectory: $packageRoot
zipPath: $zipPath
zipSizeBytes: $($zipItem.Length)
zipSizeLimitBytes: $maxZipBytes
zipSizePassed: 1
zipSHA256: $zipHash
uncompressedPayloadBytes: $packageBytes
hashedPayloadFiles: $($hashRows.Count)
requiredItemsPassed: 1
forbiddenArtifactsAbsent: 1
secretScanPassed: 1
matlabStaticAndArtifactValidationPassed: 1
slxLoadWithoutSavePassed: 1
zipExtractionPassed: 1
extractedSHA256VerificationPassed: 1
sourceRepositoryUnchanged: 1
sourceHead: $headCommit
sourceBranch: $branchName
currentExperimentSnapshotUncommitted: 1
"@
[IO.File]::WriteAllText($externalReport, $verification, [Text.UTF8Encoding]::new($false))

Write-Output "PACKAGE_DIR=$packageRoot"
Write-Output "ZIP=$zipPath"
Write-Output "ZIP_BYTES=$($zipItem.Length)"
Write-Output "ZIP_SHA256=$zipHash"
Write-Output "HASHED_FILES=$($hashRows.Count)"
Write-Output "VERIFICATION=$externalReport"
Write-Output 'STEWART_CORE_REVIEW_PACKAGE_OK'
