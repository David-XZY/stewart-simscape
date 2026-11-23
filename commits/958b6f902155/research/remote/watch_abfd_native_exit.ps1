param(
    [Parameter(Mandatory=$true)][int]$WorkerId,
    [Parameter(Mandatory=$true)][string]$ProjectDirectory,
    [Parameter(Mandatory=$true)][string]$OutputDirectory
)
$ErrorActionPreference='Stop'; $ProgressPreference='SilentlyContinue'
$base=[IO.Path]::GetFullPath('D:/XieZhenyu').TrimEnd('\')+'\'
$project=[IO.Path]::GetFullPath($ProjectDirectory).TrimEnd('\')
$output=[IO.Path]::GetFullPath($OutputDirectory)
if(-not $project.StartsWith($base,[StringComparison]::OrdinalIgnoreCase) -or
   -not $output.StartsWith($base,[StringComparison]::OrdinalIgnoreCase)){throw 'Observer paths must remain under D:/XieZhenyu'}
$file=Join-Path $output 'native_exit_status.json'
if(Test-Path -LiteralPath $file){throw 'Native exit observer already exists'}
$worker=Get-Process -Id $WorkerId
# Retain the Windows process handle before exit. Without this, a Process
# returned by Start-Process may lose its exit code after process teardown.
$heldHandle=$worker.Handle
$identity=Get-CimInstance Win32_Process -Filter "ProcessId=$WorkerId"
if($identity.Name -ne 'MATLAB.exe' -or -not $identity.CommandLine -or
   -not $identity.CommandLine.Replace('\','/').Contains($project.Replace('\','/'))){throw 'Worker is not the requested project MATLAB process'}
if($heldHandle -eq [IntPtr]::Zero){throw 'Native process handle was not retained'}
$status=@{state='attached';workerPid=$WorkerId;observerPid=$PID;started=[DateTime]::UtcNow.ToString('o');nativeExitCode=$null;
    scope='Read-only native MATLAB exit observation; numerical acceptance remains in experiment artifacts'}
function Publish {
    $status.updated=[DateTime]::UtcNow.ToString('o')
    $temporary=$file+'.tmp'
    $status | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $temporary -Encoding utf8
    Move-Item -LiteralPath $temporary -Destination $file -Force
}
Publish
try {
    $worker.WaitForExit()
    if($null -eq $worker.ExitCode){throw 'Retained native process handle returned no exit code'}
    $status.nativeExitCode=$worker.ExitCode; $status.state='exited'
} catch {
    $status.state='failed'; $status.message=$_.Exception.Message
} finally {
    $status.finished=[DateTime]::UtcNow.ToString('o'); Publish
}
if($status.state -eq 'failed'){exit 1}
