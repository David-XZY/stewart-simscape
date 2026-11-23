$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$baseRoot = 'D:\NPU\Matlab_code\stewart-simscape'
$experimentRoot = 'D:\NPU\Matlab_code\stewart-simscape-disturbance-experiment'
$outputRoot = 'D:\NPU\Matlab_code\output'
$baseCommit = 'ae06b7508151aea2c704d4b1a204bd9fb45e3347'
$experimentCommit = '7cc1ef1873be5d607dfed6873036af0e3bd6c7af'
$dateTag = '20260828'
$packageName = "Stewart_毕业设计归档_$dateTag"
$packageRoot = Join-Path $outputRoot $packageName
$zipPath = Join-Path $outputRoot "$packageName.zip"
$deliveryReportPath = Join-Path $outputRoot "$packageName`_交付验证.txt"
$zipHashPath = Join-Path $outputRoot "$packageName`_ZIP_SHA256.txt"
$verificationRoot = Join-Path $outputRoot "$packageName`_zip_verify"

function Assert-ExpectedCommit {
    param([string]$Repository, [string]$ExpectedCommit, [string]$Label)
    $actual = (& git -C $Repository rev-parse HEAD).Trim()
    if ($actual -ne $ExpectedCommit) {
        throw "$Label 的 HEAD 不符合预期。预期 $ExpectedCommit，实际 $actual。"
    }
    $dirty = @(& git -C $Repository status --porcelain)
    if ($dirty.Count -ne 0) {
        throw "$Label 存在未提交改动，拒绝从非锁定快照打包。"
    }
}

function Assert-NewTarget {
    param([string]$Path)
    if (Test-Path -LiteralPath $Path) {
        throw "目标已存在，拒绝覆盖：$Path"
    }
}

function Invoke-GitRawOutput {
    param(
        [Parameter(Mandatory = $true)][string]$Repository,
        [Parameter(Mandatory = $true)][string[]]$Arguments,
        [Parameter(Mandatory = $true)][string]$Destination
    )

    $gitExecutable = (Get-Command git -ErrorAction Stop).Source
    $startInfo = [System.Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = $gitExecutable
    $startInfo.WorkingDirectory = $Repository
    $startInfo.UseShellExecute = $false
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    foreach ($argument in $Arguments) {
        [void]$startInfo.ArgumentList.Add($argument)
    }

    $process = [System.Diagnostics.Process]::Start($startInfo)
    $stream = [System.IO.File]::Open($Destination, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::None)
    try {
        $process.StandardOutput.BaseStream.CopyTo($stream)
    }
    finally {
        $stream.Dispose()
    }
    $stderr = $process.StandardError.ReadToEnd()
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) {
        throw "Git raw-output command failed: $($Arguments -join ' '): $stderr"
    }
}

$sourceMap = [System.Collections.Generic.List[object]]::new()

function Add-ArtifactFile {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$DestinationRelative,
        [Parameter(Mandatory = $true)][string]$Category,
        [Parameter(Mandatory = $true)][string]$Origin
    )

    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) {
        throw "缺少应归档文件：$Source"
    }
    $destination = Join-Path $packageRoot $DestinationRelative
    $destinationDirectory = Split-Path -Parent $destination
    $null = New-Item -ItemType Directory -Path $destinationDirectory -Force
    Copy-Item -LiteralPath $Source -Destination $destination
    $sourceMap.Add([pscustomobject]@{
        Category = $Category
        PackagePath = $DestinationRelative
        Origin = $Origin
    })
}

function Add-ArtifactDirectory {
    param(
        [Parameter(Mandatory = $true)][string]$SourceDirectory,
        [Parameter(Mandatory = $true)][string]$DestinationRelative,
        [Parameter(Mandatory = $true)][string]$Category,
        [Parameter(Mandatory = $true)][string]$OriginPrefix
    )

    if (-not (Test-Path -LiteralPath $SourceDirectory -PathType Container)) {
        throw "缺少应归档目录：$SourceDirectory"
    }
    $files = Get-ChildItem -LiteralPath $SourceDirectory -File -Recurse -Force
    foreach ($file in $files) {
        $relative = $file.FullName.Substring($SourceDirectory.Length).TrimStart('\')
        $targetRelative = Join-Path $DestinationRelative $relative
        Add-ArtifactFile -Source $file.FullName -DestinationRelative $targetRelative -Category $Category -Origin (Join-Path $OriginPrefix $relative)
    }
}

Assert-ExpectedCommit -Repository $baseRoot -ExpectedCommit $baseCommit -Label '主工程'
Assert-ExpectedCommit -Repository $experimentRoot -ExpectedCommit $experimentCommit -Label '扰动实验工程'
Assert-NewTarget -Path $packageRoot
Assert-NewTarget -Path $zipPath
Assert-NewTarget -Path $deliveryReportPath
Assert-NewTarget -Path $zipHashPath
Assert-NewTarget -Path $verificationRoot

$null = New-Item -ItemType Directory -Path $packageRoot
$overviewDirectory = Join-Path $packageRoot '00_总览与复现'
$coreDirectory = Join-Path $packageRoot '01_核心工程\stewart-simscape_ae06b75'
$resultsDirectory = Join-Path $packageRoot '02_精选结果'
$extensionDirectory = Join-Path $packageRoot '03_扰动控制扩展'
$authorDocumentsDirectory = Join-Path $packageRoot '04_作者文档'
$null = New-Item -ItemType Directory -Path $overviewDirectory, $coreDirectory, $resultsDirectory, $extensionDirectory, $authorDocumentsDirectory

# 01: Export the exact tracked mainline snapshot, excluding only explicitly regenerable artifacts.
$excludedTrackedPatterns = @(
    '(^|/)(\.git|\.SimulinkCache|\.SimulinkCode|slprj)(/|$)',
    '\.slxc$'
)
$trackedFiles = @(& git -c core.quotepath=false -C $baseRoot ls-tree -r --name-only $baseCommit)
$coreFileCount = 0
foreach ($relativeUnix in $trackedFiles) {
    $skip = $false
    foreach ($pattern in $excludedTrackedPatterns) {
        if ($relativeUnix -match $pattern) {
            $skip = $true
            break
        }
    }
    if ($skip) { continue }

    $relativeWindows = $relativeUnix -replace '/', '\\'
    $source = Join-Path $baseRoot $relativeWindows
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
        throw "主工程工作区中缺少受版本控制文件：$relativeUnix"
    }
    $destination = Join-Path $coreDirectory $relativeWindows
    $null = New-Item -ItemType Directory -Path (Split-Path -Parent $destination) -Force
    Copy-Item -LiteralPath $source -Destination $destination
    $coreFileCount++
}

$coreCommitNote = @"
主工程快照

来源工作区：$baseRoot
Git 提交：$baseCommit
分支：David_main
导出原则：仅导出该提交受版本控制的文件；未包含 .git、.SimulinkCache、.SimulinkCode、slprj 和 *.slxc 等可再生项。
文件数：$coreFileCount
"@
Set-Content -LiteralPath (Join-Path $coreDirectory 'ARCHIVE_SNAPSHOT.txt') -Value $coreCommitNote -Encoding utf8

# 02: Preserve only the accepted baseline evidence, its reference trajectory, and selected report figures.
$referenceSource = Join-Path $baseRoot 'opt_minimal\examples\ihsid_40x20_limited_memory'
Add-ArtifactDirectory -SourceDirectory $referenceSource -DestinationRelative '02_精选结果\01_主线闭环验证\00_IHSID标准参考轨迹' -Category '标准轨迹参考' -OriginPrefix 'stewart-simscape/opt_minimal/examples/ihsid_40x20_limited_memory'

$acceptedRuns = @(
    @{ Folder = '01_腿长PIDF_20260609_101708'; Prefix = 'summary_simscape_length_control_20260609_101708'; Result = 'result_simscape_length_control_20260609_101708.mat' },
    @{ Folder = '02_长度级联_20260613_134501'; Prefix = 'summary_simscape_length_cascade_20260613_134501'; Result = 'result_simscape_length_cascade_20260613_134501.mat' },
    @{ Folder = '03_位姿长度级联_20260613_134713'; Prefix = 'summary_simscape_pose_length_control_20260613_134713'; Result = 'result_simscape_pose_length_control_20260613_134713.mat' },
    @{ Folder = '04_位姿力控制_20260614_174539'; Prefix = 'summary_simscape_pose_force_control_20260614_174539'; Result = 'result_simscape_pose_force_control_20260614_174539.mat' }
)
foreach ($run in $acceptedRuns) {
    $targetBase = "02_精选结果\01_主线闭环验证\$($run.Folder)"
    Add-ArtifactFile -Source (Join-Path $baseRoot "results\controller\$($run.Prefix).txt") -DestinationRelative "$targetBase\$($run.Prefix).txt" -Category '通过验收的主线结果' -Origin "stewart-simscape/results/controller/$($run.Prefix).txt"
    Add-ArtifactFile -Source (Join-Path $baseRoot "results\controller\$($run.Result)") -DestinationRelative "$targetBase\$($run.Result)" -Category '通过验收的主线结果' -Origin "stewart-simscape/results/controller/$($run.Result)"
}

$keyVisuals = @(
    'docs\controller_workflow\stewart_complete_simulation_workflow_技术报告.docx',
    'docs\controller_workflow\stewart_complete_simulation_workflow_技术报告.pdf',
    'docs\controller_workflow\figures\simulation_workflow\01_project_architecture.pdf',
    'docs\controller_workflow\figures\simulation_workflow\06_trajectory_kinematics.pdf',
    'docs\controller_workflow\figures\simulation_workflow\15_run02_tracking_results.pdf',
    'docs\controller_workflow\figures\simulation_workflow\17_run02_run03_run04_results.pdf',
    'docs\controller_workflow\figures\simulation_workflow\33_execution_validation_flow.pdf',
    'docs\controller_workflow\figures\control_block_diagrams\run02_pose_force_control.pdf',
    'docs\controller_workflow\figures\control_block_diagrams\run03_length_cascade_control.pdf',
    'docs\controller_workflow\figures\control_block_diagrams\run04_pose_length_cascade_control.pdf'
)
foreach ($relative in $keyVisuals) {
    Add-ArtifactFile -Source (Join-Path $baseRoot $relative) -DestinationRelative (Join-Path '02_精选结果\01_主线闭环验证\关键报告与图件' $relative.Replace('docs\controller_workflow\', '')) -Category '主线关键图件' -Origin ("stewart-simscape/" + $relative.Replace('\\', '/'))
}

Add-ArtifactDirectory -SourceDirectory (Join-Path $baseRoot 'results\reports\four_control_group_meeting_comparison_20260615_225458') -DestinationRelative '02_精选结果\01_主线闭环验证\附录_不同控制边界比较' -Category '附录控制比较' -OriginPrefix 'stewart-simscape/results/reports/four_control_group_meeting_comparison_20260615_225458'

# Keep the current disturbance evidence separate from the accepted mainline results.
Add-ArtifactDirectory -SourceDirectory (Join-Path $experimentRoot 'results\reports\disturbance_control_comparison') -DestinationRelative '02_精选结果\02_扰动安全研究\01_40场景控制比较' -Category '扰动研究证据' -OriginPrefix 'stewart-simscape-disturbance-experiment/results/reports/disturbance_control_comparison'
Add-ArtifactFile -Source (Join-Path $experimentRoot 'results\reports\lqi_aw_command_governor_ablation\ablation.mat') -DestinationRelative '02_精选结果\02_扰动安全研究\02_最终消融\ablation.mat' -Category '扰动研究证据' -Origin 'stewart-simscape-disturbance-experiment/results/reports/lqi_aw_command_governor_ablation/ablation.mat'
Add-ArtifactDirectory -SourceDirectory (Join-Path $experimentRoot 'docs\experiments\disturbance-control-comparison') -DestinationRelative '02_精选结果\02_扰动安全研究\研究结论与指标' -Category '扰动研究结论' -OriginPrefix 'stewart-simscape-disturbance-experiment/docs/experiments/disturbance-control-comparison'

# 03: Store a patch and exact changed-file snapshot for the extension. The mainline stays runnable without applying it.
$binaryPatchPath = Join-Path $extensionDirectory 'ae06b75_to_7cc1ef1_binary.patch'
Invoke-GitRawOutput -Repository $baseRoot -Arguments @('diff', '--binary', $baseCommit, $experimentCommit) -Destination $binaryPatchPath
$commitSeriesPath = Join-Path $extensionDirectory 'ae06b75_to_7cc1ef1_commit_series.patch'
Invoke-GitRawOutput -Repository $baseRoot -Arguments @('format-patch', '--stdout', "$baseCommit..$experimentCommit") -Destination $commitSeriesPath

$extensionChanges = [System.Collections.Generic.List[object]]::new()
$diffRows = @(& git -c core.quotepath=false -C $baseRoot diff --name-status $baseCommit $experimentCommit)
foreach ($row in $diffRows) {
    $parts = $row -split "`t"
    if ($parts.Count -lt 2) { throw "无法解析扩展变更行：$row" }
    $status = $parts[0]
    $relativeUnix = $parts[$parts.Count - 1]
    $relativeWindows = $relativeUnix -replace '/', '\\'
    $source = Join-Path $experimentRoot $relativeWindows
    Add-ArtifactFile -Source $source -DestinationRelative (Join-Path '03_扰动控制扩展\changed_files' $relativeWindows) -Category '扰动控制代码或证据变更' -Origin ("stewart-simscape-disturbance-experiment/" + $relativeUnix)
    $extensionChanges.Add([pscustomobject]@{ Status = $status; Path = $relativeUnix })
}
$extensionChanges | Export-Csv -LiteralPath (Join-Path $extensionDirectory 'extension_file_manifest.csv') -NoTypeInformation -Encoding utf8

$extensionReadme = @"
# 扰动控制研究扩展

- 基线：`ae06b75`（David_main）
- 扩展：`7cc1ef1`（experiment/disturbance-control-comparison）
- 变更：52 个新增文件、4 个修改文件；`changed_files/` 保留扩展提交的原路径副本。

## 使用边界

主工程无需应用本目录补丁即可运行。扩展功能默认关闭，只有研究复现或继续开发时才应在主工程副本中应用补丁。

- `controller/runs/run_15_disturbance_control_comparison.m`：固定矩阵下的扰动控制比较。
- `controller/runs/run_16_lqi_aw_command_governor_ablation.m`：QP 输出感知 anti-windup 与姿态 command governor 的消融。
- 测试 79–89：扰动配置、DOB、严格 QP、anti-windup 与 governor 的契约检查。

当前证据不支持把该扩展替换为默认控制器：40 场景矩阵没有控制器通过全部“优于既有控制”门槛；最终消融只显示边界条件下的安全取舍，不是通用性能保证。
"@
Set-Content -LiteralPath (Join-Path $extensionDirectory 'README.md') -Value $extensionReadme -Encoding utf8

# 04: Copy current project documents whose Git history is attributed to David-XZY, excluding third-party library/source documentation.
$documentExtensions = @('.docx', '.pdf', '.pptx', '.vsdx', '.md', '.org', '.tex')
$rawDocumentPaths = @(& git -c core.quotepath=false -C $baseRoot log --all --author='David-XZY' --name-only --format='')
$documentPaths = $rawDocumentPaths | Where-Object {
    $_ -and
    $documentExtensions -contains ([System.IO.Path]::GetExtension($_).ToLowerInvariant()) -and
    $_ -match '^(docs/|controller/|opt_minimal/|results/)' -and
    $_ -notmatch '^(lib/|docs/(figs|org_source)/)'
} | Sort-Object -Unique

foreach ($relativeUnix in $documentPaths) {
    $relativeWindows = $relativeUnix -replace '/', '\\'
    $experimentSource = Join-Path $experimentRoot $relativeWindows
    $baseSource = Join-Path $baseRoot $relativeWindows
    if (Test-Path -LiteralPath $experimentSource -PathType Leaf) {
        Add-ArtifactFile -Source $experimentSource -DestinationRelative (Join-Path '04_作者文档\项目内' $relativeWindows) -Category 'David-XZY 项目文档' -Origin ("stewart-simscape-disturbance-experiment/" + $relativeUnix)
    }
    elseif (Test-Path -LiteralPath $baseSource -PathType Leaf) {
        Add-ArtifactFile -Source $baseSource -DestinationRelative (Join-Path '04_作者文档\项目内' $relativeWindows) -Category 'David-XZY 项目文档' -Origin ("stewart-simscape/" + $relativeUnix)
    }
}

$rootDocuments = @(
    'D:\NPU\Matlab_code\第四章第一节_Simscape联合仿真平台设计_论文稿.docx',
    'D:\NPU\Matlab_code\第四章第一节_Simscape联合仿真平台设计_论文稿_定稿.docx',
    'D:\NPU\Matlab_code\Stewart平台_PID闭环控制原理图.vsdx'
)
foreach ($document in $rootDocuments) {
    Add-ArtifactFile -Source $document -DestinationRelative (Join-Path '04_作者文档\根目录论文与图件' (Split-Path -Leaf $document)) -Category '根目录直接相关成果' -Origin $document
}

$authorReadme = @"
# 作者文档收录说明

本目录收录两类文件：

1. Git 历史中作者为 `David-XZY`、位于项目文档/控制/优化/结果目录且当前仍存在的相关文档；
2. `D:\NPU\Matlab_code` 根目录的两份第四章 Word 稿和一份 PID Visio 图。

项目内文件保留原始相对路径。第三方库 `lib/`、上游原始 `docs/figs` 与 `docs/org_source` 不在本目录重复收录，但仍随 `01_核心工程` 的原始快照保存。具体来源见 `00_总览与复现/source_mapping.csv`。
"@
Set-Content -LiteralPath (Join-Path $authorDocumentsDirectory 'README.md') -Value $authorReadme -Encoding utf8

# Write the overview after all selections are fixed.
$overview = @"
# Stewart 平台毕业设计归档资料包

## 路线总览

1. **问题背景与模型对象**：6-UCU Stewart 并联平台，关注受约束空间轨迹的可执行性与闭环跟踪。
2. **轨迹规划**：IHSID 生成满足几何、运动学和工程后验约束的参考轨迹，入口为 `opt_minimal/run_01_ihsid_trajectory.m`。
3. **建模仿真**：`matlab/stewart_platform_model.slx` 建立平台、六支链、负载、关节、重力、传感与日志链路；参考输入被转换为控制与 Simscape 接口数据。
4. **控制验证**：腿长 PIDF、长度级联、位姿—长度级联、位姿—力控制分别以选定的通过验收结果保存。
5. **扰动安全研究**：DOB-aware 严格 CLF-CBF-QP、QP 输出感知 anti-windup 和姿态 command governor 作为独立实验扩展保留。

## 版本与入口

- 可复现主线：`David_main` / `$baseCommit`，完整快照在 `01_核心工程/stewart-simscape_ae06b75/`。
- 研究扩展：`experiment/disturbance-control-comparison` / `$experimentCommit`，补丁和变更文件在 `03_扰动控制扩展/`。
- 轨迹：`opt_minimal/run_01_ihsid_trajectory.m`。
- 基础 Simscape 运行：`controller/runs/run_02_simscape_length_control.m`、`run_03_simscape_length_cascade_control.m`、`run_04_simscape_pose_length_control.m`。
- 活动契约测试：`controller/tests/run_all_active_tests.m`；扰动扩展对应 `run_15`、`run_16` 和测试 79–89。

## 环境与边界

需要 MATLAB、Simulink 和 Simscape；归档包含当前工程使用的 CasADi/IPOPT/MA27 相关 `lib` 资源，但运行仍取决于 MATLAB 版本、Windows 二进制兼容性及本机工具链。

所有控制指标均为该归档中记录条件下的 **Simscape 仿真** 证据。工程包含原型/接口与传感建模资料，但不等同于已经完成实机闭环、HIL 或硬件安全验收。

## 结果筛选原则

- 主线正面证据只保留明确 `passed` / `overallPassed` 的四组运行；`20260617_214358` 位姿—力运行未通过，不作为正面结果收录。
- 四方案比较保留为附录，因为执行器与控制边界不同，不作统一性能排名。
- 扰动矩阵与最终消融被保留为研究证据；其结论是“尚未满足默认控制器替代门槛”，而不是算法优越性声明。
- 不收录可再生缓存、重复/调参 smoke 运行、已拒绝报告和无关临时文件；原始位置与理由见 `selection_and_exclusions.md`。
"@
Set-Content -LiteralPath (Join-Path $overviewDirectory 'README.md') -Value $overview -Encoding utf8

$selectionAndExclusions = @"
# 精选与排除说明

## 已收录

- 标准 IHSID 参考轨迹及其可供 Simscape 使用的数据；
- 四组已通过验收的主线 Simscape 运行（腿长 PIDF、长度级联、位姿—长度级联、位姿—力）；
- 支撑科研路线的技术报告、流程图、控制结构图和一组边界不同的附录比较；
- 40 场景扰动控制矩阵、DOB-aware 严格 QP 代表性记录、最终 command-governor/anti-windup 消融；
- Git 可追溯的作者文档和根目录三份直接相关论文/图件。

## 未收录

| 原始位置或类型 | 原因 |
|---|---|
| `.git`、`.SimulinkCache`、`.SimulinkCode`、`slprj`、`*.slxc` | 可再生版本/构建缓存，不影响由核心快照重建。 |
| `results/controller/summary_simscape_pose_force_control_20260617_214358.*` | 该次运行 `overallPassed=0`，不作为毕业设计的正面控制结果。 |
| `results/reports/lqi_aw_command_governor_ablation/smoke*`、`tune_3hz`、`high_gain_0p1` | 中间筛选或调参运行；最终原始消融 MAT 与结论/CSV 已单独保留。 |
| `results/reports/strict_clf_cbf_qp` 的早期精确模型研究 | 与最终扰动 Simscape 证据重叠，且不构成默认控制器结论。 |
| 第三方库内文档、上游 `docs/figs` 与 `docs/org_source` | 在核心工程快照中保留，不在作者文档区重复归档。 |
"@
Set-Content -LiteralPath (Join-Path $overviewDirectory 'selection_and_exclusions.md') -Value $selectionAndExclusions -Encoding utf8

$sourceMap | Sort-Object Category, PackagePath | Export-Csv -LiteralPath (Join-Path $overviewDirectory 'source_mapping.csv') -NoTypeInformation -Encoding utf8

# Do not archive likely credentials. Record only paths, never matching contents.
$secretPattern = '(sk-[A-Za-z0-9]{20,}|ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,}|AKIA[0-9A-Z]{16}|xox[baprs]-[A-Za-z0-9-]{20,}|-----BEGIN (RSA |EC |OPENSSH )?PRIVATE KEY-----)'
$suspectPaths = @(& rg -l --hidden --pcre2 -i $secretPattern $packageRoot)
if ($LASTEXITCODE -eq 0 -and $suspectPaths.Count -gt 0) {
    $suspectPaths | Set-Content -LiteralPath (Join-Path $overviewDirectory 'sensitive_file_paths_detected.txt') -Encoding utf8
    throw '检测到疑似凭据文件，已停止 ZIP 前的交付流程。'
}
if ($LASTEXITCODE -gt 1) {
    throw '敏感信息扫描失败。'
}

# The self-hash file is deliberately excluded from its own inventory.
$hashManifestPath = Join-Path $overviewDirectory 'package_files_sha256.csv'
$hashRows = Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force |
    Sort-Object FullName |
    ForEach-Object {
        [pscustomobject]@{
            PackagePath = $_.FullName.Substring($packageRoot.Length).TrimStart('\')
            Bytes = $_.Length
            SHA256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
        }
    }
$hashRows | Export-Csv -LiteralPath $hashManifestPath -NoTypeInformation -Encoding utf8

$packageBytes = (Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force | Measure-Object -Property Length -Sum).Sum
Compress-Archive -LiteralPath $packageRoot -DestinationPath $zipPath -CompressionLevel Optimal
if (-not (Test-Path -LiteralPath $zipPath -PathType Leaf)) {
    throw 'ZIP 文件未生成。'
}

# Verify the delivery by extracting it and comparing every inventoried source file.
$null = New-Item -ItemType Directory -Path $verificationRoot
Expand-Archive -LiteralPath $zipPath -DestinationPath $verificationRoot
$extractedRoot = Join-Path $verificationRoot $packageName
if (-not (Test-Path -LiteralPath $extractedRoot -PathType Container)) {
    throw 'ZIP 解压后的根目录不符合预期。'
}
$extractedManifest = Import-Csv -LiteralPath (Join-Path $extractedRoot '00_总览与复现\package_files_sha256.csv')
foreach ($entry in $extractedManifest) {
    $extractedFile = Join-Path $extractedRoot $entry.PackagePath
    if (-not (Test-Path -LiteralPath $extractedFile -PathType Leaf)) {
        throw "ZIP 校验缺少文件：$($entry.PackagePath)"
    }
    $actualHash = (Get-FileHash -LiteralPath $extractedFile -Algorithm SHA256).Hash
    if ($actualHash -ne $entry.SHA256) {
        throw "ZIP 校验哈希不一致：$($entry.PackagePath)"
    }
}
$expectedPaths = @($extractedManifest.PackagePath)
$actualPaths = Get-ChildItem -LiteralPath $extractedRoot -File -Recurse -Force |
    ForEach-Object { $_.FullName.Substring($extractedRoot.Length).TrimStart('\') } |
    Where-Object { $_ -ne '00_总览与复现\package_files_sha256.csv' }
if (@($actualPaths | Where-Object { $_ -notin $expectedPaths }).Count -ne 0 -or $actualPaths.Count -ne $expectedPaths.Count) {
    throw 'ZIP 校验文件清单与哈希清单不一致。'
}

$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash
Set-Content -LiteralPath $zipHashPath -Value "$zipHash  $(Split-Path -Leaf $zipPath)" -Encoding ascii
$zipBytes = (Get-Item -LiteralPath $zipPath).Length
$deliveryReport = @"
Stewart 毕业设计归档交付验证
生成时间：$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss K')
归档目录：$packageRoot
ZIP：$zipPath
主工程提交：$baseCommit
扰动扩展提交：$experimentCommit
核心快照文件：$coreFileCount
归档清单文件：$($hashRows.Count)
归档目录字节数：$packageBytes
ZIP 字节数：$zipBytes
ZIP SHA-256：$zipHash
敏感信息扫描：通过（未发现所定义的凭据签名）
ZIP 解压与逐文件 SHA-256 比对：通过
"@
Set-Content -LiteralPath $deliveryReportPath -Value $deliveryReport -Encoding utf8

# The extraction directory is a verified transient copy only.
$resolvedOutput = (Resolve-Path -LiteralPath $outputRoot).Path.TrimEnd('\')
$resolvedVerification = [System.IO.Path]::GetFullPath($verificationRoot).TrimEnd('\')
if (-not $resolvedVerification.StartsWith($resolvedOutput + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
    throw '验证临时目录不在 output 内，拒绝删除。'
}
Remove-Item -LiteralPath $verificationRoot -Recurse -Force

Write-Output "PACKAGE_ROOT=$packageRoot"
Write-Output "ZIP_PATH=$zipPath"
Write-Output "DELIVERY_REPORT=$deliveryReportPath"
Write-Output "ZIP_HASH_FILE=$zipHashPath"
Write-Output "CORE_FILE_COUNT=$coreFileCount"
Write-Output "MANIFEST_FILE_COUNT=$($hashRows.Count)"
Write-Output "PACKAGE_BYTES=$packageBytes"
Write-Output "ZIP_BYTES=$zipBytes"
