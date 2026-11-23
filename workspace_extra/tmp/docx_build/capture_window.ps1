param([int]$ProcessId, [string]$OutputPath)
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class WinRect {
 [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
 [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
}
'@
$p=Get-Process -Id $ProcessId
$rect=New-Object WinRect+RECT
[void][WinRect]::GetWindowRect([IntPtr]$p.MainWindowHandle,[ref]$rect)
$bmp=New-Object Drawing.Bitmap ($rect.Right-$rect.Left),($rect.Bottom-$rect.Top)
$g=[Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($rect.Left,$rect.Top,0,0,$bmp.Size)
$bmp.Save($OutputPath,[Drawing.Imaging.ImageFormat]::Png)
$g.Dispose();$bmp.Dispose()
