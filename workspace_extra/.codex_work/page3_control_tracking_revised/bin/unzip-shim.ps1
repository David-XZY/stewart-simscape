param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ArgsRest
)

$mode = $ArgsRest[0]
$zipPath = $ArgsRest[1]
if ($mode -eq '-Z1') {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $zip = [System.IO.Compression.ZipFile]::OpenRead($zipPath)
    try {
        foreach ($entry in $zip.Entries) { [Console]::Out.WriteLine($entry.FullName) }
    }
    finally { $zip.Dispose() }
    exit 0
}
if ($mode -eq '-p') {
    $entryName = $ArgsRest[2]
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $zip = [System.IO.Compression.ZipFile]::OpenRead($zipPath)
    try {
        $entry = $zip.GetEntry($entryName)
        if ($null -eq $entry) { exit 1 }
        $stream = $entry.Open()
        try { $stream.CopyTo([Console]::OpenStandardOutput()) }
        finally { $stream.Dispose() }
    }
    finally { $zip.Dispose() }
    exit 0
}
exit 1
