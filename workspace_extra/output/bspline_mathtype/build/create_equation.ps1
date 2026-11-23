param([string]$OutputPath = 'D:/NPU/Matlab_code/tmp/docx_build/mathtype_uia_test.docx')

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes

$word = $null
$doc = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $true
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Add()
    $doc.Content.Text = '$\begin{gathered}s=t/T,\qquad p=5,\qquad n=16,\\[6pt]U=[0^{\times6},1/11,\ldots,10/11,1^{\times6}],\\[6pt]P_0=P_1=P_2=q_0,\qquad P_{13}=P_{14}=P_{15}=q_f,\\[6pt]z=[P_3^{\mathsf T},\ldots,P_{12}^{\mathsf T}]^{\mathsf T}\in\mathbb R^{60},\\[6pt]q(t)=\sum_{i=0}^{15}N_{i,5}(t/T)P_i.\end{gathered}$'
    $texRange = $doc.Range(0, $doc.Content.End - 1)
    $texRange.Select()
    $word.Activate()
    Start-Sleep -Milliseconds 700

    $windowHandle = $doc.ActiveWindow.Hwnd
    $root = [System.Windows.Automation.AutomationElement]::FromHandle([IntPtr]$windowHandle)
    $nameCondition = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty, 'MathType')
    $mathTab = $root.FindFirst(
        [System.Windows.Automation.TreeScope]::Descendants, $nameCondition)
    if ($null -eq $mathTab) {
        throw 'MathType tab was not found.'
    }
    Write-Output ('MathTabType=' + $mathTab.Current.ControlType.ProgrammaticName)
    $selectPattern = $mathTab.GetCurrentPattern(
        [System.Windows.Automation.SelectionItemPattern]::Pattern)
    $selectPattern.Select()
    Start-Sleep -Milliseconds 600
    $texRange.Select()
    $word.Activate()
    Start-Sleep -Milliseconds 300

    $toggleButton = $null
    $all = $root.FindAll(
        [System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($element in $all) {
        if ($element.Current.Name -match 'TeX' -and
                $element.Current.ControlType -eq [System.Windows.Automation.ControlType]::Button) {
            $toggleButton = $element
            break
        }
    }
    if ($null -eq $toggleButton) {
        foreach ($element in $all) {
            $name = $element.Current.Name
            if ($name -match 'TeX|公式|转换|MathType') {
                Write-Output ($element.Current.ControlType.ProgrammaticName + ' :: ' +
                    $element.Current.AutomationId + ' :: ' + $name)
            }
        }
        throw 'MathType TeX Toggle button was not found.'
    }
    Write-Output ('ToggleType=' + $toggleButton.Current.ControlType.ProgrammaticName)
    Write-Output ('ToggleEnabled=' + $toggleButton.Current.IsEnabled)
    $invoke = $toggleButton.GetCurrentPattern(
        [System.Windows.Automation.InvokePattern]::Pattern)
    $invoke.Invoke()
    for ($waitIndex = 0; $waitIndex -lt 30; $waitIndex++) {
        Start-Sleep -Milliseconds 500
        if ($doc.InlineShapes.Count -gt 0) { break }
    }

    $doc.SaveAs2($OutputPath, 16)
    Write-Output ('InlineShapes=' + $doc.InlineShapes.Count)
    Write-Output ('Shapes=' + $doc.Shapes.Count)
    Write-Output ('Content=' + $doc.Content.Text)
    for ($index = 1; $index -le $doc.InlineShapes.Count; $index++) {
        $shape = $doc.InlineShapes.Item($index)
        try {
            Write-Output ('ProgID=' + $shape.OLEFormat.ProgID)
        } finally {
            [System.Runtime.InteropServices.Marshal]::ReleaseComObject($shape) | Out-Null
        }
    }
} finally {
    if ($null -ne $doc) { $doc.Close($false) }
    if ($null -ne $word) { $word.Quit() }
    if ($null -ne $doc) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc) | Out-Null }
    if ($null -ne $word) { [System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null }
}

