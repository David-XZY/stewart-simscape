$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$baseRoot = 'D:\NPU\Matlab_code\stewart-simscape'
$baseCommit = 'ae06b7508151aea2c704d4b1a204bd9fb45e3347'
$experimentCommit = '7cc1ef1873be5d607dfed6873036af0e3bd6c7af'
$outputRoot = 'D:\NPU\Matlab_code\output'
$packageName = 'Stewart_毕业设计归档_20260828'
$packageRoot = Join-Path $outputRoot $packageName
$extensionRoot = Join-Path $packageRoot '03_扰动控制扩展'
$patchPath = Join-Path $extensionRoot 'ae06b75_to_7cc1ef1_binary.patch'
$seriesPath = Join-Path $extensionRoot 'ae06b75_to_7cc1ef1_commit_series.patch'
$zipPath = Join-Path $outputRoot "$packageName.zip"
$zipTempPath = Join-Path $outputRoot "$packageName.rebuilt.zip"
$verificationRoot = Join-Path $outputRoot "$packageName`_zip_verify_rebuilt"
$deliveryReportPath = Join-Path $outputRoot "$packageName`_交付验证.txt"
$zipHashPath = Join-Path $outputRoot "$packageName`_ZIP_SHA256.txt"

function Assert-UnderOutput {
    param([string]$Path)
    $resolvedOutput = (Resolve-Path -LiteralPath $outputRoot).Path.TrimEnd('\')
    $resolvedPath = [System.IO.Path]::GetFullPath($Path).TrimEnd('\')
    if (-not $resolvedPath.StartsWith($resolvedOutput + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to modify a path outside output: $Path"
    }
}

function Invoke-GitRawOutput {
    param([string[]]$Arguments, [string]$Destination)
    $gitExecutable = (Get-Command git -ErrorAction Stop).Source
    $startInfo = [System.Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = $gitExecutable
    $startInfo.WorkingDirectory = $baseRoot
    $startInfo.UseShellExecute = $false
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    foreach ($argument in $Arguments) { [void]$startInfo.ArgumentList.Add($argument) }
    $process = [System.Diagnostics.Process]::Start($startInfo)
    $stream = [System.IO.File]::Open($Destination, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::None)
    try { $process.StandardOutput.BaseStream.CopyTo($stream) } finally { $stream.Dispose() }
    $stderr = $process.StandardError.ReadToEnd()
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) { throw "Git output command failed: $stderr" }
}

foreach ($path in @($packageRoot, $patchPath, $seriesPath, $zipPath, $deliveryReportPath, $zipHashPath)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Expected delivery artifact is missing: $path" }
}
foreach ($path in @($zipTempPath, $verificationRoot)) {
    Assert-UnderOutput -Path $path
    if (Test-Path -LiteralPath $path) { throw "Temporary target already exists: $path" }
}

$patchTemp = "$patchPath.rebuilt"
$seriesTemp = "$seriesPath.rebuilt"
Assert-UnderOutput -Path $patchTemp
Assert-UnderOutput -Path $seriesTemp
if ((Test-Path -LiteralPath $patchTemp) -or (Test-Path -LiteralPath $seriesTemp)) { throw 'Patch temporary target already exists.' }

Invoke-GitRawOutput -Arguments @('diff', '--binary', $baseCommit, $experimentCommit) -Destination $patchTemp
git -C $baseRoot apply --check --binary $patchTemp
if ($LASTEXITCODE -ne 0) { throw 'Rebuilt binary patch did not pass git apply --check.' }
Invoke-GitRawOutput -Arguments @('format-patch', '--stdout', "$baseCommit..$experimentCommit") -Destination $seriesTemp

[System.IO.File]::Copy($patchTemp, $patchPath, $true)
[System.IO.File]::Copy($seriesTemp, $seriesPath, $true)
[System.IO.File]::Delete($patchTemp)
[System.IO.File]::Delete($seriesTemp)

$manifestPath = Join-Path $packageRoot '00_总览与复现\package_files_sha256.csv'
$hashRows = Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force |
    Where-Object { $_.FullName -ne $manifestPath } |
    Sort-Object FullName |
    ForEach-Object {
        [pscustomobject]@{
            PackagePath = $_.FullName.Substring($packageRoot.Length).TrimStart('\')
            Bytes = $_.Length
            SHA256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
        }
    }
$hashRows | Export-Csv -LiteralPath $manifestPath -NoTypeInformation -Encoding utf8

Compress-Archive -LiteralPath $packageRoot -DestinationPath $zipTempPath -CompressionLevel Optimal
if (-not (Test-Path -LiteralPath $zipTempPath -PathType Leaf)) { throw 'Rebuilt ZIP was not created.' }

$null = New-Item -ItemType Directory -Path $verificationRoot
Expand-Archive -LiteralPath $zipTempPath -DestinationPath $verificationRoot
$extractedRoot = Join-Path $verificationRoot $packageName
$manifest = Import-Csv -LiteralPath (Join-Path $extractedRoot '00_总览与复现\package_files_sha256.csv')
foreach ($entry in $manifest) {
    $file = Join-Path $extractedRoot $entry.PackagePath
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Rebuilt ZIP is missing $($entry.PackagePath)" }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -ne $entry.SHA256) { throw "Rebuilt ZIP hash mismatch: $($entry.PackagePath)" }
}
$expected = @($manifest.PackagePath)
$actual = Get-ChildItem -LiteralPath $extractedRoot -File -Recurse -Force |
    ForEach-Object { $_.FullName.Substring($extractedRoot.Length).TrimStart('\') } |
    Where-Object { $_ -ne '00_总览与复现\package_files_sha256.csv' }
if (@($actual | Where-Object { $_ -notin $expected }).Count -ne 0 -or $actual.Count -ne $expected.Count) { throw 'Rebuilt ZIP file inventory differs from manifest.' }

Assert-UnderOutput -Path $zipPath
[System.IO.File]::Delete($zipPath)
[System.IO.File]::Move($zipTempPath, $zipPath)
$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash
[System.IO.File]::WriteAllText($zipHashPath, "$zipHash  $(Split-Path -Leaf $zipPath)`r`n", [System.Text.UTF8Encoding]::new($false))
$packageBytes = (Get-ChildItem -LiteralPath $packageRoot -File -Recurse -Force | Measure-Object Length -Sum).Sum
$zipBytes = (Get-Item -LiteralPath $zipPath).Length
$report = @"
Stewart 毕业设计归档交付验证
生成时间：$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss K')
归档目录：$packageRoot
ZIP：$zipPath
主工程提交：$baseCommit
扰动扩展提交：$experimentCommit
归档清单文件：$($hashRows.Count)
归档目录字节数：$packageBytes
ZIP 字节数：$zipBytes
ZIP SHA-256：$zipHash
敏感信息扫描：通过（首次构建时未发现所定义的凭据签名）
扩展二进制补丁 git apply --check：通过
ZIP 解压与逐文件 SHA-256 比对：通过
"@
[System.IO.File]::WriteAllText($deliveryReportPath, $report, [System.Text.UTF8Encoding]::new($false))

Assert-UnderOutput -Path $verificationRoot
[System.IO.Directory]::Delete($verificationRoot, $true)

Write-Output "PATCH_CHECK=passed"
Write-Output "ZIP_PATH=$zipPath"
Write-Output "ZIP_BYTES=$zipBytes"
Write-Output "ZIP_HASH=$zipHash"
Write-Output "MANIFEST_FILE_COUNT=$($hashRows.Count)"
