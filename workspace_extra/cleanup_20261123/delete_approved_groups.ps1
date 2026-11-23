$ErrorActionPreference = 'Stop'
$workspace = [IO.Path]::GetFullPath('D:\NPU\Matlab_code').TrimEnd('\')
$receiptRoot = Join-Path $workspace 'cleanup_20261123\delete_groups_3_4_10'
$plan = Get-Content -LiteralPath (Join-Path $receiptRoot 'approved_plan.json') -Raw | ConvertFrom-Json
$expected = @(
 'stewart-simscape-tracking-mpc', 'stewart-simscape-tracking-mpc-v2',
 'stewart-simscape-tracking-mpc-v3', 'stewart-simscape-tracking-mpc-v4',
 'stewart-simscape-tracking-mpc-v5', 'stewart-simscape-tracking-mpc-v6-50hz',
 'stewart-simscape-tracking-mpc-v8-sqp-statistics', 'stewart-simscape-abfd',
 'deliveries', '_archive\20261117'
) | ForEach-Object { [IO.Path]::GetFullPath((Join-Path $workspace $_)) }
$plannedRoots = @($plan.roots | ForEach-Object { [IO.Path]::GetFullPath($_) })
if (($plan.groups -join ',') -ne '3,4,10' -or (Compare-Object $expected $plannedRoots)) { throw 'Approved group scope does not match.' }
$commonGit = [IO.Path]::GetFullPath((Join-Path $workspace 'stewart-simscape\.git'))
if ([IO.Path]::GetFullPath($plan.protected_git_common_dir) -ne $commonGit) { throw 'Unexpected shared Git directory.' }
$repo = Join-Path $workspace 'stewart-simscape'
$actions = Join-Path $receiptRoot 'actions.jsonl'

function Write-Action($value) {
    $value | ConvertTo-Json -Compress -Depth 12 | Add-Content -LiteralPath $actions -Encoding utf8
}

function Assert-SafeTarget([string]$target, [bool]$allowDescendant = $false) {
    if (-not [IO.Path]::IsPathFullyQualified($target)) { throw "Target is not absolute: $target" }
    $full = [IO.Path]::GetFullPath($target).TrimEnd('\')
    if (-not $full.StartsWith($workspace + '\', [StringComparison]::OrdinalIgnoreCase)) { throw "Target escapes workspace: $full" }
    if ($full -eq $commonGit -or $full.StartsWith($commonGit + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Shared Git metadata is protected.' }
    if ($allowDescendant) {
        $inside = @($expected | Where-Object { $full -eq $_ -or $full.StartsWith($_ + '\', [StringComparison]::OrdinalIgnoreCase) })
        if ($inside.Count -eq 0) { throw "Worktree outside approved roots: $full" }
    } elseif ($full -notin $plannedRoots -and $full -notin @($plan.files)) { throw "Target is not on approved manifest: $full" }
    if (Test-Path -LiteralPath $full) {
        $resolved = (Resolve-Path -LiteralPath $full).ProviderPath.TrimEnd('\')
        if ($resolved -ne $full) { throw "Resolved target differs: $full => $resolved" }
        $part = Get-Item -LiteralPath $full -Force
        while ($part -and $part.FullName -ne $workspace) {
            if ($part.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse point is excluded: $($part.FullName)" }
            $parentName = [IO.Path]::GetDirectoryName($part.FullName)
            $part = if ($parentName) { Get-Item -LiteralPath $parentName -Force } else { $null }
        }
    }
    return $full
}

# Recheck all target roots before the first destructive operation. Do not
# follow directory junctions or let another process's old research be stopped.
foreach ($target in @($plan.roots) + @($plan.files)) {
    $full = Assert-SafeTarget $target
    if ((Test-Path -LiteralPath $full -PathType Container)) {
        $links = @(Get-ChildItem -LiteralPath $full -Recurse -Force -Attributes ReparsePoint)
        if ($links.Count) { throw "Reparse entries require separate review: $full" }
    }
}
$busy = @(Get-CimInstance Win32_Process | Where-Object {
    if ($_.ProcessId -eq $PID -or -not $_.CommandLine) { return $false }
    $command = $_.CommandLine.Replace('/', '\')
    foreach ($target in $plannedRoots) {
        if ($command.IndexOf($target, [StringComparison]::OrdinalIgnoreCase) -ge 0) { return $true }
    }
    return $false
})
if ($busy.Count) {
    $busy | Select-Object ProcessId, Name, CommandLine | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $receiptRoot 'busy_processes.json') -Encoding utf8
    throw 'An approved old directory is still referenced by a running process; no process has been stopped.'
}

# Nested ABFD worktrees are listed deepest first by the read-only preparer.
# Git removes only these worktree registrations, leaving common objects and
# branch refs in stewart-simscape/.git intact.
$registeredWorktrees = @(& git --no-optional-locks -C $repo worktree list --porcelain | Where-Object { $_.StartsWith('worktree ') } | ForEach-Object { [IO.Path]::GetFullPath($_.Substring(9)) })
if ($LASTEXITCODE -ne 0) { throw 'Cannot read current worktree registry.' }
foreach ($entry in $plan.worktrees_to_remove) {
    $target = Assert-SafeTarget $entry.worktree $true
    if (-not (Test-Path -LiteralPath $target)) { continue }
    if ($target -notin $registeredWorktrees) {
        Write-Action @{ operation='registration_already_removed_keep_remainder_for_scoped_cleanup'; path=$target; time=(Get-Date -Format o) }
        continue
    }
    & git --no-optional-locks -c core.longpaths=true -C $repo worktree remove --force -- $target
    $code = $LASTEXITCODE
    Write-Action @{ operation='git_worktree_remove'; path=$target; exit_code=$code; time=(Get-Date -Format o) }
    if ($code -ne 0) { throw "Git worktree removal failed: $target" }
    Write-Output "REMOVED_WORKTREE $target"
}

foreach ($target in @($plan.roots) + @($plan.files)) {
    $full = Assert-SafeTarget $target
    if (-not (Test-Path -LiteralPath $full)) { continue }
    if (Test-Path -LiteralPath $full -PathType Container) {
        Remove-Item -LiteralPath $full -Recurse -Force -ErrorAction Stop
    } else {
        Remove-Item -LiteralPath $full -Force -ErrorAction Stop
    }
    if (Test-Path -LiteralPath $full) { throw "Target remains after deletion: $full" }
    Write-Action @{ operation='remove_approved_target'; path=$full; time=(Get-Date -Format o) }
    Write-Output "REMOVED_TARGET $full"
}
@{ completed=(Get-Date -Format o); groups=@(3,4,10); status='deletion_finished_pending_independent_verification' } |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $receiptRoot 'execution.json') -Encoding utf8
