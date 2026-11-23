param([string]$DocumentPath = 'D:/NPU/Matlab_code/tmp/docx_build/tex_toggle_retry.docx')

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes

$word = New-Object -ComObject Word.Application
$word.Visible = $true
$word.DisplayAlerts = 0
$doc = $word.Documents.Open($DocumentPath, $false, $true)
try {
    $root = [System.Windows.Automation.AutomationElement]::FromHandle([IntPtr]$doc.ActiveWindow.Hwnd)
    $all = $root.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($element in $all) {
        if ($element.Current.Name -eq 'MathType' -and
            $element.Current.ControlType -eq [System.Windows.Automation.ControlType]::TabItem) {
            $element.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern).Select()
            break
        }
    }
    Start-Sleep -Milliseconds 800
    $all = $root.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($element in $all) {
        if ($element.Current.ControlType -in @(
            [System.Windows.Automation.ControlType]::Button,
            [System.Windows.Automation.ControlType]::MenuItem,
            [System.Windows.Automation.ControlType]::SplitButton)) {
            Write-Output ($element.Current.ControlType.ProgrammaticName + ' | ' +
                $element.Current.AutomationId + ' | ' + $element.Current.Name)
        }
    }
} finally {
    $doc.Close($false)
    $word.Quit()
}
