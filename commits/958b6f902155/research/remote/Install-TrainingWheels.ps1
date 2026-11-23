param([string]$Root = 'D:\XieZhenyu', [string]$ProxyUrl = 'http://127.0.0.1:7890')
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$wheelDir = Join-Path $Root 'downloads\wheels'
New-Item -ItemType Directory -Path $wheelDir -Force | Out-Null
$report = Get-Content -LiteralPath (Join-Path $Root 'downloads\torch-resolve.json') -Raw | ConvertFrom-Json
foreach ($package in $report.install) {
    $url = $package.download_info.url
    $uri = [uri]$url
    if ($uri.Scheme -ne 'https' -or $uri.Host -notin @('download.pytorch.org','download-r2.pytorch.org','files.pythonhosted.org')) {
        throw 'Unexpected wheel source'
    }
    $file = Join-Path $wheelDir ([uri]::UnescapeDataString($uri.Segments[-1]))
    $expected = $package.download_info.archive_info.hashes.sha256
    if (!$expected) { throw 'Missing wheel hash' }
    if (!(Test-Path -LiteralPath $file)) {
        Write-Output ('DOWNLOAD ' + $package.metadata.name)
        $curlArgs = @('--fail','--location','--retry','5','--retry-delay','2',
            '--continue-at','-','--max-time','1800','--output',($file+'.partial'),$url)
        if ($ProxyUrl) { $curlArgs = @('--proxy',$ProxyUrl) + $curlArgs }
        $download = Start-Process -FilePath "$env:SystemRoot\System32\curl.exe" -ArgumentList $curlArgs -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput (Join-Path $Root 'logs\wheel-download.stdout.log') -RedirectStandardError (Join-Path $Root 'logs\wheel-download.stderr.log')
        if ($download.ExitCode -ne 0) { throw "Wheel download failed; partial file retained: $($download.ExitCode)" }
        if ((Get-FileHash -LiteralPath ($file + '.partial') -Algorithm SHA256).Hash.ToLower() -ne $expected) { throw 'Wheel SHA256 mismatch' }
        Move-Item -LiteralPath ($file + '.partial') -Destination $file
    }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLower() -ne $expected) { throw 'Existing wheel SHA256 mismatch' }
}
$env:TEMP = Join-Path $Root 'tmp'
$env:TMP = $env:TEMP
$requirements = @($report.install | ForEach-Object { $_.metadata.name+'=='+$_.metadata.version })
$installArgs = @('-m','pip','install','--no-index','--find-links',$wheelDir,'--disable-pip-version-check') + $requirements
$p = Start-Process -FilePath (Join-Path $Root 'envs\stewart-abfd\Scripts\python.exe') -ArgumentList $installArgs -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput (Join-Path $Root 'logs\install-pytorch.log') -RedirectStandardError (Join-Path $Root 'logs\install-pytorch.stderr.log')
Set-Content -LiteralPath (Join-Path $Root 'logs\install-pytorch.exit') -Value $p.ExitCode
Get-Content -LiteralPath (Join-Path $Root 'logs\install-pytorch.log') -Tail 12
exit $p.ExitCode
