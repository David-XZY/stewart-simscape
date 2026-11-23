param(
    [Parameter(Mandatory = $true)][string]$InputDocx,
    [Parameter(Mandatory = $true)][string]$OutputDir
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$rendererSource = @"
using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading;

public static class ClipboardEmfRenderer
{
    [DllImport("user32.dll")] static extern bool OpenClipboard(IntPtr hWndNewOwner);
    [DllImport("user32.dll")] static extern bool CloseClipboard();
    [DllImport("user32.dll")] static extern IntPtr GetClipboardData(uint uFormat);
    [DllImport("gdi32.dll", CharSet = CharSet.Unicode)] static extern IntPtr CopyEnhMetaFile(IntPtr hemfSrc, string lpszFile);
    [DllImport("gdi32.dll")] static extern bool DeleteEnhMetaFile(IntPtr hemf);

    public static void SaveClipboardPage(string emfPath, string pngPath)
    {
        bool opened = false;
        for (int i = 0; i < 30 && !opened; i++)
        {
            opened = OpenClipboard(IntPtr.Zero);
            if (!opened) Thread.Sleep(100);
        }
        if (!opened) throw new InvalidOperationException("Cannot open clipboard");
        try
        {
            IntPtr source = GetClipboardData(14);
            if (source == IntPtr.Zero) throw new InvalidOperationException("Clipboard has no enhanced metafile");
            IntPtr copy = CopyEnhMetaFile(source, emfPath);
            if (copy == IntPtr.Zero) throw new InvalidOperationException("CopyEnhMetaFile failed");
            DeleteEnhMetaFile(copy);
        }
        finally { CloseClipboard(); }

        using (var metafile = new Metafile(emfPath))
        {
            int width = Math.Max(1, metafile.Width * 2);
            int height = Math.Max(1, metafile.Height * 2);
            using (var bitmap = new Bitmap(width, height, PixelFormat.Format32bppArgb))
            {
                bitmap.SetResolution(192, 192);
                using (var graphics = Graphics.FromImage(bitmap))
                {
                    graphics.Clear(Color.White);
                    graphics.SmoothingMode = SmoothingMode.HighQuality;
                    graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;
                    graphics.DrawImage(metafile, 0, 0, width, height);
                }
                bitmap.Save(pngPath, ImageFormat.Png);
            }
        }
    }
}
"@
Add-Type -TypeDefinition $rendererSource -ReferencedAssemblies 'System.Drawing'

$inputPath = (Resolve-Path -LiteralPath $InputDocx).Path
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
$outPath = (Resolve-Path -LiteralPath $OutputDir).Path
$word = $null
$doc = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Open($inputPath, $false, $true)
    $pages = $doc.ComputeStatistics(2)
    for ($page = 1; $page -le $pages; $page++) {
        $start = $doc.GoTo(1, 1, $page).Start
        if ($page -lt $pages) {
            $end = $doc.GoTo(1, 1, $page + 1).Start - 1
        } else {
            $end = $doc.Content.End - 1
        }
        $range = $doc.Range($start, $end)
        $range.CopyAsPicture()
        Start-Sleep -Milliseconds 350
        $file = Join-Path $outPath ("page-{0}.png" -f $page)
        $emf = Join-Path $outPath ("page-{0}.emf" -f $page)
        [ClipboardEmfRenderer]::SaveClipboardPage($emf, $file)
        Remove-Item -LiteralPath $emf -Force
        Write-Output $file
    }
}
finally {
    if ($doc -ne $null) { $doc.Close($false) }
    if ($word -ne $null) { $word.Quit() }
}
