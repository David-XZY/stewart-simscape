param(
    [Parameter(Mandatory=$true)][string]$ScriptPath,
    [string]$HostAlias = 'workstation-p3'
)
$ErrorActionPreference = 'Stop'
$script = Get-Content -LiteralPath $ScriptPath -Raw
$encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($script))
& ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=15 $HostAlias powershell.exe -NoProfile -NonInteractive -OutputFormat Text -EncodedCommand $encoded
exit $LASTEXITCODE
