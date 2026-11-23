param(
    [Parameter(Mandatory=$true)][string]$InputPath,
    [Parameter(Mandatory=$true)][string]$OutputPath
)

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class MathTypeMouse {
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint flags, uint dx, uint dy, uint data, UIntPtr extraInfo);
    public const uint LEFTDOWN = 0x0002;
    public const uint LEFTUP = 0x0004;
    public static void Click(int x, int y) {
        SetCursorPos(x, y);
        mouse_event(LEFTDOWN, 0, 0, 0, UIntPtr.Zero);
        mouse_event(LEFTUP, 0, 0, 0, UIntPtr.Zero);
    }
}
'@

$word = $null
$doc = $null
$shell = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $true
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Open($InputPath, $false, $false)
    $doc.SetCompatibilityMode(15)
    $word.Activate()
    Start-Sleep -Milliseconds 700

    $root = [System.Windows.Automation.AutomationElement]::FromHandle(
        [IntPtr]$doc.ActiveWindow.Hwnd)
    $all = $root.FindAll(
        [System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    $mathTab = $null
    foreach ($element in $all) {
        if ($element.Current.Name -eq 'MathType' -and
                $element.Current.ControlType -eq [System.Windows.Automation.ControlType]::TabItem) {
            $mathTab = $element
            break
        }
    }
    if ($null -eq $mathTab) { throw 'MathType Word tab not found.' }
    $mathTab.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern).Select()
    Start-Sleep -Milliseconds 700

    $all = $root.FindAll(
        [System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    $buttons = @()
    foreach ($element in $all) {
        if ($element.Current.ControlType -eq [System.Windows.Automation.ControlType]::Button) {
            $buttons += $element
        }
    }
    $convertButton = $null
    for ($index = 0; $index -lt $buttons.Count; $index++) {
        if ($buttons[$index].Current.Name -match 'MathPage' -and $index -ge 2) {
            $convertButton = $buttons[$index - 2]
            break
        }
    }
    if ($null -eq $convertButton) { throw 'MathType Convert Equations button not found.' }
    $convertButton.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
    Start-Sleep -Milliseconds 900

    $dialog = $null
    $dialogCandidates = $root.FindAll(
        [System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($candidate in $dialogCandidates) {
        if ($candidate.Current.ControlType -eq [System.Windows.Automation.ControlType]::Window) {
            $dialog = $candidate
            break
        }
    }
    if ($null -eq $dialog) { throw 'MathType conversion dialog not found.' }
    $dialogRect = $dialog.Current.BoundingRectangle
    [void][MathTypeMouse]::SetForegroundWindow([IntPtr]$dialog.Current.NativeWindowHandle)
    $shell = New-Object -ComObject WScript.Shell
    Start-Sleep -Milliseconds 300
    # MathType 7's dialog leaves the OMML source type unchecked by default.
    # Enable the fourth source checkbox and click the Convert button.
    [MathTypeMouse]::Click(
        [int]($dialogRect.X + 0.046 * $dialogRect.Width),
        [int]($dialogRect.Y + 0.497 * $dialogRect.Height))
    Start-Sleep -Milliseconds 100
    [MathTypeMouse]::Click(
        [int]($dialogRect.X + 0.046 * $dialogRect.Width),
        [int]($dialogRect.Y + 0.497 * $dialogRect.Height))
    Start-Sleep -Milliseconds 250
    $debugBitmap = New-Object System.Drawing.Bitmap([int]$dialogRect.Width, [int]$dialogRect.Height)
    $debugGraphics = [System.Drawing.Graphics]::FromImage($debugBitmap)
    $debugGraphics.CopyFromScreen([int]$dialogRect.X, [int]$dialogRect.Y, 0, 0, $debugBitmap.Size)
    $debugBitmap.Save('D:/NPU/Matlab_code/tmp/docx_build/mathtype_after_omml_click.png', [System.Drawing.Imaging.ImageFormat]::Png)
    $debugGraphics.Dispose()
    $debugBitmap.Dispose()
    [MathTypeMouse]::Click(
        [int]($dialogRect.X + 0.929 * $dialogRect.Width),
        [int]($dialogRect.Y + 0.181 * $dialogRect.Height))

    Start-Sleep -Seconds 4
    # Accept MathType's completion report.
    $shell.SendKeys('{ENTER}')

    for ($waitIndex = 0; $waitIndex -lt 60; $waitIndex++) {
        Start-Sleep -Milliseconds 500
        try {
            $null = $doc.InlineShapes.Count
            break
        } catch {
            # Word remains busy while the modal conversion is running.
        }
    }

    $doc.SaveAs2($OutputPath, 12)
    $mathTypeCount = 0
    $progIds = @()
    for ($index = 1; $index -le $doc.InlineShapes.Count; $index++) {
        $shape = $doc.InlineShapes.Item($index)
        try {
            $progId = $shape.OLEFormat.ProgID
            $progIds += $progId
            if ($progId -match 'Equation\.DSMT|Equation\.3|MathType') {
                $mathTypeCount++
            }
        } catch {
        } finally {
            [System.Runtime.InteropServices.Marshal]::ReleaseComObject($shape) | Out-Null
        }
    }
    Write-Output ('MathTypeObjects=' + $mathTypeCount)
    Write-Output ('InlineShapes=' + $doc.InlineShapes.Count)
    Write-Output ('OMaths=' + $doc.OMaths.Count)
    Write-Output ('ProgIDs=' + ($progIds -join ','))
    Write-Output $OutputPath
} finally {
    if ($null -ne $doc) {
        try { $doc.Close($false) } catch { }
    }
    if ($null -ne $word) {
        try { $word.Quit() } catch { }
    }
    if ($null -ne $shell) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($shell) | Out-Null }
    if ($null -ne $doc) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc) | Out-Null }
    if ($null -ne $word) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null }
}
