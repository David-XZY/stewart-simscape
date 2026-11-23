param([string]$InputPath = 'D:/NPU/Matlab_code/tmp/docx_build/compat15_test.docx')

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class Win32Inspect {
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool EnumChildWindows(IntPtr hWndParent, EnumWindowsProc lpEnumFunc, IntPtr lParam);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetClassName(IntPtr hWnd, StringBuilder lpClassName, int nMaxCount);
    [DllImport("user32.dll")] public static extern int GetDlgCtrlID(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
}
'@

$word = $null
$doc = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $true
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Open($InputPath, $false, $false)
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
    if ($null -eq $mathTab) { throw 'MathType tab missing' }
    $mathTab.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern).Select()
    Start-Sleep -Milliseconds 700
    $all = $root.FindAll(
        [System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    $convertButton = $null
    $buttonList = @()
    foreach ($element in $all) {
        if ($element.Current.ControlType -eq [System.Windows.Automation.ControlType]::Button) {
            $buttonList += $element
            Write-Output ('BUTTON :: ' + $element.Current.AutomationId + ' :: ' + $element.Current.Name)
        }
    }
    for ($buttonIndex = 0; $buttonIndex -lt $buttonList.Count; $buttonIndex++) {
        if ($buttonList[$buttonIndex].Current.Name -match 'MathPage' -and $buttonIndex -ge 2) {
            $convertButton = $buttonList[$buttonIndex - 2]
            break
        }
    }
    if ($null -eq $convertButton) { throw 'Convert button missing' }
    Write-Output ('INVOKE :: ' + $convertButton.Current.AutomationId)
    $convertButton.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
    Start-Sleep -Seconds 2

    $dialog = $null
    $rootDescendants = $root.FindAll(
        [System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($candidate in $rootDescendants) {
        if ($candidate.Current.ControlType -eq [System.Windows.Automation.ControlType]::Window) {
            $dialog = $candidate
            break
        }
    }
    if ($null -ne $dialog) {
        Write-Output ('DIALOG :: ' + $dialog.Current.NativeWindowHandle + ' :: ' + $dialog.Current.AutomationId + ' :: ' + $dialog.Current.Name)
        $rect = $dialog.Current.BoundingRectangle
        $bitmap = New-Object System.Drawing.Bitmap([int]$rect.Width, [int]$rect.Height)
        $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
        $graphics.CopyFromScreen([int]$rect.X, [int]$rect.Y, 0, 0, $bitmap.Size)
        $screenshotPath = 'D:/NPU/Matlab_code/tmp/docx_build/mathtype_convert_dialog.png'
        $bitmap.Save($screenshotPath, [System.Drawing.Imaging.ImageFormat]::Png)
        $graphics.Dispose()
        $bitmap.Dispose()
        Write-Output ('SCREENSHOT :: ' + $screenshotPath)
        $dialogHandle = [IntPtr]$dialog.Current.NativeWindowHandle
        $callback = [Win32Inspect+EnumWindowsProc]{
            param([IntPtr]$handle, [IntPtr]$parameter)
            $textBuilder = New-Object Text.StringBuilder 512
            $classBuilder = New-Object Text.StringBuilder 256
            [void][Win32Inspect]::GetWindowText($handle, $textBuilder, $textBuilder.Capacity)
            [void][Win32Inspect]::GetClassName($handle, $classBuilder, $classBuilder.Capacity)
            Write-Output ('HWND :: ' + $handle + ' :: id=' + [Win32Inspect]::GetDlgCtrlID($handle) +
                ' :: visible=' + [Win32Inspect]::IsWindowVisible($handle) +
                ' :: class=' + $classBuilder.ToString() + ' :: text=' + $textBuilder.ToString())
            return $true
        }
        [void][Win32Inspect]::EnumChildWindows($dialogHandle, $callback, [IntPtr]::Zero)
        $dialogChildren = $dialog.FindAll(
            [System.Windows.Automation.TreeScope]::Descendants,
            [System.Windows.Automation.Condition]::TrueCondition)
        foreach ($child in $dialogChildren) {
            if ($child.Current.Name -or $child.Current.AutomationId) {
                Write-Output ('D  ' + $child.Current.ControlType.ProgrammaticName + ' :: ' +
                    $child.Current.AutomationId + ' :: ' + $child.Current.Name +
                    ' :: enabled=' + $child.Current.IsEnabled)
            }
        }
        foreach ($child in $dialogChildren) {
            if ($child.Current.ControlType -eq [System.Windows.Automation.ControlType]::Button -and
                    $child.Current.AutomationId -eq '2') {
                $child.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
                Start-Sleep -Milliseconds 500
                break
            }
        }
    }

    $desktop = [System.Windows.Automation.AutomationElement]::RootElement
    $windows = $desktop.FindAll(
        [System.Windows.Automation.TreeScope]::Children,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($window in $windows) {
        if ($window.Current.ProcessId -eq $root.Current.ProcessId) {
            Write-Output ('WINDOW :: ' + $window.Current.AutomationId + ' :: ' + $window.Current.Name)
            $children = $window.FindAll(
                [System.Windows.Automation.TreeScope]::Descendants,
                [System.Windows.Automation.Condition]::TrueCondition)
            foreach ($child in $children) {
                if ($child.Current.Name -or $child.Current.AutomationId) {
                    Write-Output ('  ' + $child.Current.ControlType.ProgrammaticName + ' :: ' +
                        $child.Current.AutomationId + ' :: ' + $child.Current.Name)
                }
            }
        }
    }
    Start-Sleep -Seconds 3
} finally {
    if ($null -ne $doc) { $doc.Close($false) }
    if ($null -ne $word) { $word.Quit() }
    if ($null -ne $doc) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc) | Out-Null }
    if ($null -ne $word) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null }
}
