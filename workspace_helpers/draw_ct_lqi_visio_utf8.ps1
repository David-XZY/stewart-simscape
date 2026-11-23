$ErrorActionPreference = 'Stop'

$helperSource = 'C:\Users\David_X\.codex\skills\visio-automation\scripts\visio_helpers.ps1'
$helperCopy = Join-Path $env:TEMP 'codex_visio_helpers_utf8.ps1'
$helperText = [IO.File]::ReadAllText($helperSource, [Text.Encoding]::UTF8)
[IO.File]::WriteAllText($helperCopy, $helperText, (New-Object Text.UTF8Encoding($true)))
. $helperCopy

$outputDirectory = 'D:\NPU\并联机器人\控制\控制框图'
$outputVsdx = Join-Path $outputDirectory '计算力矩_LQI控制框图.vsdx'
$outputPng1 = Join-Path $outputDirectory '计算力矩_LQI总体控制结构.png'
$outputPng2 = Join-Path $outputDirectory 'LQI内部结构与抗饱和回灌.png'
$verificationPath = 'D:\NPU\Matlab_code\ct_lqi_visio_verification.json'

New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
foreach ($path in @($outputVsdx, $outputPng1, $outputPng2)) {
    if (Test-Path -LiteralPath $path) {
        Remove-Item -LiteralPath $path -Force
    }
}

function Invoke-ComRetry {
    param(
        [Parameter(Mandatory = $true)][scriptblock]$Action,
        [int]$Retries = 40,
        [int]$DelayMilliseconds = 500
    )
    for ($i = 0; $i -lt $Retries; $i++) {
        try {
            return & $Action
        } catch {
            if ($i -eq ($Retries - 1)) { throw }
            Start-Sleep -Milliseconds $DelayMilliseconds
        }
    }
}

function Set-Font {
    param($Shape, [string]$FontName = 'SimSun')
    if ($Shape.CellExistsU('Char.Font', 0) -ne 0) {
        $Shape.CellsU('Char.Font').FormulaU = "FONT(`"$FontName`")"
    }
}

function Add-Block {
    param(
        $Page,
        [double]$X,
        [double]$Y,
        [double]$Width,
        [double]$Height,
        [string]$Text,
        [double]$FontSize = 10.5
    )
    $shape = $Page.DrawRectangle($X - $Width / 2, $Y - $Height / 2, $X + $Width / 2, $Y + $Height / 2)
    $shape.Text = $Text
    Set-VisioShapeFill -Shape $shape -Fill 'RGB(255,255,255)' -Line 'RGB(0,0,0)' -LineWeight '1 pt'
    Set-VisioTextStyle -Shape $shape -Color 'RGB(0,0,0)' -Size $FontSize -Bold 0
    Set-Font -Shape $shape -FontName 'SimSun'
    return $shape
}

function Add-Sum {
    param($Page, [double]$X, [double]$Y, [string]$Text = "+`n+")
    $shape = $Page.DrawOval($X - 0.27, $Y - 0.27, $X + 0.27, $Y + 0.27)
    $shape.Text = $Text
    Set-VisioShapeFill -Shape $shape -Fill 'RGB(255,255,255)' -Line 'RGB(0,0,0)' -LineWeight '1 pt'
    Set-VisioTextStyle -Shape $shape -Color 'RGB(0,0,0)' -Size 9 -Bold 0
    Set-Font -Shape $shape -FontName 'Times New Roman'
    return $shape
}

function Add-Text {
    param(
        $Page,
        [double]$X,
        [double]$Y,
        [double]$Width,
        [double]$Height,
        [string]$Text,
        [double]$FontSize = 10,
        [int]$Style = 0,
        [string]$FontName = 'SimSun'
    )
    $shape = $Page.DrawRectangle($X - $Width / 2, $Y - $Height / 2, $X + $Width / 2, $Y + $Height / 2)
    $shape.Text = $Text
    $shape.CellsU('LinePattern').FormulaU = '0'
    $shape.CellsU('FillPattern').FormulaU = '0'
    Set-VisioTextStyle -Shape $shape -Color 'RGB(0,0,0)' -Size $FontSize -Bold $Style
    Set-Font -Shape $shape -FontName $FontName
    return $shape
}

function Add-Anchor {
    param($Page, [double]$X, [double]$Y)
    $shape = $Page.DrawOval($X - 0.015, $Y - 0.015, $X + 0.015, $Y + 0.015)
    $shape.CellsU('LinePattern').FormulaU = '0'
    $shape.CellsU('FillPattern').FormulaU = '0'
    return $shape
}

function Add-Junction {
    param($Page, [double]$X, [double]$Y)
    $shape = $Page.DrawOval($X - 0.045, $Y - 0.045, $X + 0.045, $Y + 0.045)
    Set-VisioShapeFill -Shape $shape -Fill 'RGB(0,0,0)' -Line 'RGB(0,0,0)' -LineWeight '0.75 pt'
    return $shape
}

function Add-Boundary {
    param($Page, [double]$X1, [double]$Y1, [double]$X2, [double]$Y2)
    $shape = $Page.DrawRectangle($X1, $Y1, $X2, $Y2)
    $shape.CellsU('FillPattern').FormulaU = '0'
    $shape.CellsU('LineColor').FormulaU = 'RGB(0,0,0)'
    $shape.CellsU('LineWeight').FormulaU = '1.2 pt'
    $shape.CellsU('LinePattern').FormulaU = '2'
    if ($shape.CellExistsU('Rounding', 0) -ne 0) {
        $shape.CellsU('Rounding').FormulaU = '0.18 in'
    }
    try { $shape.SendToBack() } catch { }
    return $shape
}

function Set-ConnectorAppearance {
    param($Connector, [bool]$Dashed = $false, [int]$EndArrow = 4)
    $Connector.CellsU('LineColor').FormulaU = 'RGB(0,0,0)'
    $Connector.CellsU('LineWeight').FormulaU = '1 pt'
    $Connector.CellsU('EndArrow').FormulaU = "$EndArrow"
    if ($Dashed) {
        $Connector.CellsU('LinePattern').FormulaU = '2'
    } else {
        $Connector.CellsU('LinePattern').FormulaU = '1'
    }
    return $Connector
}

function Add-StraightConnector {
    param(
        $Page, $Master, $From, $To,
        [double]$FromX = 1.0, [double]$FromY = 0.5,
        [double]$ToX = 0.0, [double]$ToY = 0.5,
        [int]$EndArrow = 4, [bool]$Dashed = $false
    )
    $connector = Connect-VisioShapesStraight -Page $Page -ConnectorMaster $Master -From $From -To $To -FromX $FromX -FromY $FromY -ToX $ToX -ToY $ToY -EndArrow $EndArrow -LineColor 'RGB(0,0,0)'
    Set-ConnectorAppearance -Connector $connector -Dashed $Dashed -EndArrow $EndArrow | Out-Null
    return $connector
}

function Add-OrthogonalConnector {
    param(
        $Page, $Master, $From, $To,
        [double]$FromX = 1.0, [double]$FromY = 0.5,
        [double]$ToX = 0.0, [double]$ToY = 0.5,
        [int]$EndArrow = 4, [bool]$Dashed = $false
    )
    $connector = Connect-VisioShapesOrthogonal -Page $Page -ConnectorMaster $Master -From $From -To $To -FromX $FromX -FromY $FromY -ToX $ToX -ToY $ToY -EndArrow $EndArrow -LineColor 'RGB(0,0,0)'
    Set-ConnectorAppearance -Connector $connector -Dashed $Dashed -EndArrow $EndArrow | Out-Null
    return $connector
}

function Set-PageGeometry {
    param($Page, [double]$Width, [double]$Height)
    $Page.PageSheet.CellsU('PageWidth').FormulaU = "$Width in"
    $Page.PageSheet.CellsU('PageHeight').FormulaU = "$Height in"
    if ($Page.PageSheet.CellExistsU('PageLeftMargin', 0) -ne 0) {
        $Page.PageSheet.CellsU('PageLeftMargin').FormulaU = '0 in'
        $Page.PageSheet.CellsU('PageRightMargin').FormulaU = '0 in'
        $Page.PageSheet.CellsU('PageTopMargin').FormulaU = '0 in'
        $Page.PageSheet.CellsU('PageBottomMargin').FormulaU = '0 in'
    }
}

# Lightweight graph models used as the drawing source of truth.
$graphOverview = [pscustomobject]@{
    diagramType = 'control-block-diagram'
    page = [pscustomobject]@{ width = 13.2; height = 6.3 }
    nodes = @(
        [pscustomobject]@{ id = 'trajectory'; text = "参考轨迹`n连续重建"; x = 1.45; y = 3.65; w = 1.55; h = 0.85 }
        [pscustomobject]@{ id = 'error_sum'; text = "+`n−"; x = 3.00; y = 2.65; kind = 'sum' }
        [pscustomobject]@{ id = 'ct'; text = "逆动力学`n计算力矩"; x = 4.25; y = 4.55; w = 1.70; h = 0.85 }
        [pscustomobject]@{ id = 'lqi'; text = "LQI输出反馈`n状态观测/积分校正"; x = 4.45; y = 2.65; w = 1.80; h = 0.90 }
        [pscustomobject]@{ id = 'force_sum'; text = "+`n+"; x = 6.25; y = 3.65; kind = 'sum' }
        [pscustomobject]@{ id = 'actuator'; text = "非理想力执行器`n延迟/死区/速率/滞后/饱和"; x = 8.15; y = 3.65; w = 2.05; h = 0.95 }
        [pscustomobject]@{ id = 'platform'; text = "Stewart`n平台"; x = 10.35; y = 3.65; w = 1.55; h = 0.90 }
    )
    edges = @('input->trajectory', 'trajectory->ct', 'trajectory->error_sum', 'error_sum->lqi', 'ct->force_sum', 'lqi->force_sum', 'force_sum->actuator', 'actuator->platform', 'platform->output', 'output->error_sum', 'actuator->lqi')
}

$graphLqi = [pscustomobject]@{
    diagramType = 'control-block-diagram'
    page = [pscustomobject]@{ width = 12.0; height = 6.2 }
    nodes = @(
        [pscustomobject]@{ id = 'observer'; text = "状态观测器`nx̂̇=A x̂+B F_LQI+L_o(e_q−C x̂)"; x = 3.05; y = 4.55; w = 2.65; h = 0.95 }
        [pscustomobject]@{ id = 'kx'; text = '−K_x'; x = 5.55; y = 4.55; w = 1.0; h = 0.70 }
        [pscustomobject]@{ id = 'integrator'; text = "积分状态更新`nẋ_I=−e_q−λx_I+K_awΔF"; x = 3.05; y = 2.25; w = 2.65; h = 1.00 }
        [pscustomobject]@{ id = 'ki'; text = '−K_I'; x = 5.55; y = 2.25; w = 1.0; h = 0.70 }
        [pscustomobject]@{ id = 'sum'; text = "+`n+"; x = 7.25; y = 3.40; kind = 'sum' }
    )
    edges = @('e_q->observer', 'e_q->integrator', 'observer->kx', 'integrator->ki', 'kx->sum', 'ki->sum', 'sum->F_LQI', 'F_LQI->observer', 'DeltaF->integrator')
}

$visio = New-Object -ComObject Visio.Application
Start-Sleep -Seconds 5
$shell = New-Object -ComObject WScript.Shell
$null = $shell.AppActivate('Microsoft Visio')
$shell.SendKeys('{ESC}')
Start-Sleep -Seconds 2
Invoke-ComRetry { $visio.Visible = $true } | Out-Null

$documents = Invoke-ComRetry { $visio.Documents }
$stencil = Invoke-ComRetry { Open-VisioStencilReadOnly -Visio $visio -StencilNameOrPath 'BASFLO_M.VSSX' }
$connectorMaster = Invoke-ComRetry { $stencil.Masters.ItemU('Dynamic connector') }
$document = Invoke-ComRetry { $documents.Add('') }

# Page 1: overall computed-torque-LQI controller.
$page1 = $visio.ActivePage
$page1.Name = '计算力矩-LQI总体控制结构'
Set-PageGeometry -Page $page1 -Width $graphOverview.page.width -Height $graphOverview.page.height

$boundary1 = Add-Boundary -Page $page1 -X1 2.55 -Y1 1.65 -X2 6.65 -Y2 5.55
$null = Add-Text -Page $page1 -X 4.60 -Y 5.74 -Width 2.5 -Height 0.30 -Text '计算力矩–LQI控制器' -FontSize 10.5

$input1 = Add-Anchor -Page $page1 -X 0.20 -Y 3.65
$trajectory = Add-Block -Page $page1 -X 1.45 -Y 3.65 -Width 1.55 -Height 0.85 -Text "参考轨迹`n连续重建"
$refJunction = Add-Junction -Page $page1 -X 2.55 -Y 3.65
$errorSum = Add-Sum -Page $page1 -X 3.00 -Y 2.65 -Text "+`n−"
$ct = Add-Block -Page $page1 -X 4.25 -Y 4.55 -Width 1.70 -Height 0.85 -Text "逆动力学`n计算力矩"
$lqi = Add-Block -Page $page1 -X 4.45 -Y 2.65 -Width 1.80 -Height 0.90 -Text "LQI输出反馈`n状态观测/积分校正" -FontSize 9.5
$forceSum = Add-Sum -Page $page1 -X 6.25 -Y 3.65 -Text "+`n+"
$actuator = Add-Block -Page $page1 -X 8.15 -Y 3.65 -Width 2.05 -Height 0.95 -Text "非理想力执行器`n延迟/死区/速率/滞后/饱和" -FontSize 9.2
$platform = Add-Block -Page $page1 -X 10.35 -Y 3.65 -Width 1.55 -Height 0.90 -Text "Stewart`n平台"
$outputJunction = Add-Junction -Page $page1 -X 11.45 -Y 3.65
$output1 = Add-Anchor -Page $page1 -X 12.85 -Y 3.65

$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $input1 -To $trajectory
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $trajectory -To $refJunction -EndArrow 0
$null = Add-OrthogonalConnector -Page $page1 -Master $connectorMaster -From $refJunction -To $ct -FromX 0.5 -FromY 1.0 -ToX 0.0 -ToY 0.5
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $refJunction -To $errorSum -FromX 0.5 -FromY 0.0 -ToX 0.0 -ToY 0.5
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $errorSum -To $lqi
$null = Add-OrthogonalConnector -Page $page1 -Master $connectorMaster -From $ct -To $forceSum -FromX 1.0 -FromY 0.5 -ToX 0.5 -ToY 1.0
$null = Add-OrthogonalConnector -Page $page1 -Master $connectorMaster -From $lqi -To $forceSum -FromX 1.0 -FromY 0.5 -ToX 0.5 -ToY 0.0
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $forceSum -To $actuator
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $actuator -To $platform
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $platform -To $outputJunction -EndArrow 0
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $outputJunction -To $output1

# Exact orthogonal feedback route via invisible glued waypoints.
$feedbackWp1 = Add-Anchor -Page $page1 -X 11.45 -Y 0.95
$feedbackWp2 = Add-Anchor -Page $page1 -X 3.00 -Y 0.95
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $outputJunction -To $feedbackWp1 -FromX 0.5 -FromY 0.0 -ToX 0.5 -ToY 1.0 -EndArrow 0
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $feedbackWp1 -To $feedbackWp2 -FromX 0.0 -FromY 0.5 -ToX 1.0 -ToY 0.5 -EndArrow 0
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $feedbackWp2 -To $errorSum -FromX 0.5 -FromY 1.0 -ToX 0.5 -ToY 0.0

# Dashed anti-windup route from the actuator to LQI.
$awWp1 = Add-Anchor -Page $page1 -X 8.15 -Y 1.45
$awWp2 = Add-Anchor -Page $page1 -X 4.45 -Y 1.45
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $actuator -To $awWp1 -FromX 0.5 -FromY 0.0 -ToX 0.5 -ToY 1.0 -EndArrow 0 -Dashed $true
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $awWp1 -To $awWp2 -FromX 0.0 -FromY 0.5 -ToX 1.0 -ToY 0.5 -EndArrow 0 -Dashed $true
$null = Add-StraightConnector -Page $page1 -Master $connectorMaster -From $awWp2 -To $lqi -FromX 0.5 -FromY 1.0 -ToX 0.5 -ToY 0.0 -Dashed $true

$null = Add-Text -Page $page1 -X 0.62 -Y 4.02 -Width 1.15 -Height 0.50 -Text "期望轨迹`nr(t)" -FontSize 9.5
$null = Add-Text -Page $page1 -X 2.75 -Y 4.02 -Width 1.65 -Height 0.30 -Text 'q_r, q̇_r, q̈_r' -FontSize 10 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 5.35 -Y 4.88 -Width 0.75 -Height 0.30 -Text 'F_CT' -FontSize 10 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 3.52 -Y 2.95 -Width 0.65 -Height 0.28 -Text 'e_q' -FontSize 10 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 5.45 -Y 2.92 -Width 0.85 -Height 0.28 -Text 'F_LQI' -FontSize 10 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 7.12 -Y 4.00 -Width 0.85 -Height 0.28 -Text 'F_nom' -FontSize 10 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 9.23 -Y 4.00 -Width 0.75 -Height 0.28 -Text 'F_act' -FontSize 10 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 12.15 -Y 4.20 -Width 1.55 -Height 0.75 -Text "实际位姿 q(t)`n支链状态 L, L̇, F" -FontSize 9.2
$null = Add-Text -Page $page1 -X 7.00 -Y 1.66 -Width 2.10 -Height 0.28 -Text 'ΔF = F_act − F_raw' -FontSize 9.5 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page1 -X 7.20 -Y 0.72 -Width 2.20 -Height 0.28 -Text '实际位姿反馈 q(t)' -FontSize 9.5

# Page 2: LQI internal structure and anti-windup.
$page2 = $document.Pages.Add()
$page2.Name = 'LQI内部结构与抗饱和回灌'
Set-PageGeometry -Page $page2 -Width $graphLqi.page.width -Height $graphLqi.page.height
$boundary2 = Add-Boundary -Page $page2 -X1 0.65 -Y1 0.65 -X2 11.00 -Y2 5.72
$null = Add-Text -Page $page2 -X 5.83 -Y 5.88 -Width 1.55 -Height 0.30 -Text 'LQI控制器' -FontSize 10.5

$inputEq = Add-Anchor -Page $page2 -X 0.20 -Y 3.40
$eqJunction = Add-Junction -Page $page2 -X 1.00 -Y 3.40
$observer = Add-Block -Page $page2 -X 3.05 -Y 4.55 -Width 2.65 -Height 0.95 -Text "状态观测器`nx̂̇=A x̂+B F_LQI+L_o(e_q−C x̂)" -FontSize 9.2
$kx = Add-Block -Page $page2 -X 5.55 -Y 4.55 -Width 1.00 -Height 0.70 -Text '−K_x' -FontSize 11
$integrator = Add-Block -Page $page2 -X 3.05 -Y 2.25 -Width 2.65 -Height 1.00 -Text "积分状态更新`nẋ_I=−e_q−λx_I+K_awΔF" -FontSize 9.4
$ki = Add-Block -Page $page2 -X 5.55 -Y 2.25 -Width 1.00 -Height 0.70 -Text '−K_I' -FontSize 11
$sumLqi = Add-Sum -Page $page2 -X 7.25 -Y 3.40 -Text "+`n+"
$outJunctionLqi = Add-Junction -Page $page2 -X 8.25 -Y 3.40
$outputLqi = Add-Anchor -Page $page2 -X 10.65 -Y 3.40
$inputDelta = Add-Anchor -Page $page2 -X 0.20 -Y 1.10

$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $inputEq -To $eqJunction -EndArrow 0
$null = Add-OrthogonalConnector -Page $page2 -Master $connectorMaster -From $eqJunction -To $observer -FromX 0.5 -FromY 1.0 -ToX 0.0 -ToY 0.5
$null = Add-OrthogonalConnector -Page $page2 -Master $connectorMaster -From $eqJunction -To $integrator -FromX 0.5 -FromY 0.0 -ToX 0.0 -ToY 0.5
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $observer -To $kx
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $integrator -To $ki
$null = Add-OrthogonalConnector -Page $page2 -Master $connectorMaster -From $kx -To $sumLqi -FromX 1.0 -FromY 0.5 -ToX 0.5 -ToY 1.0
$null = Add-OrthogonalConnector -Page $page2 -Master $connectorMaster -From $ki -To $sumLqi -FromX 1.0 -FromY 0.5 -ToX 0.5 -ToY 0.0
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $sumLqi -To $outJunctionLqi -EndArrow 0
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $outJunctionLqi -To $outputLqi

$deltaWp = Add-Anchor -Page $page2 -X 1.10 -Y 1.10
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $inputDelta -To $deltaWp -EndArrow 0 -Dashed $true
$null = Add-OrthogonalConnector -Page $page2 -Master $connectorMaster -From $deltaWp -To $integrator -FromX 0.5 -FromY 1.0 -ToX 0.0 -ToY 0.15 -Dashed $true

$modelWp1 = Add-Anchor -Page $page2 -X 8.25 -Y 5.25
$modelWp2 = Add-Anchor -Page $page2 -X 3.05 -Y 5.25
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $outJunctionLqi -To $modelWp1 -FromX 0.5 -FromY 1.0 -ToX 0.5 -ToY 0.0 -EndArrow 0
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $modelWp1 -To $modelWp2 -FromX 0.0 -FromY 0.5 -ToX 1.0 -ToY 0.5 -EndArrow 0
$null = Add-StraightConnector -Page $page2 -Master $connectorMaster -From $modelWp2 -To $observer -FromX 0.5 -FromY 0.0 -ToX 0.5 -ToY 1.0

$null = Add-Text -Page $page2 -X 0.58 -Y 3.73 -Width 0.80 -Height 0.30 -Text 'e_q' -FontSize 10.5 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page2 -X 4.45 -Y 4.88 -Width 0.55 -Height 0.28 -Text 'x̂' -FontSize 10.5 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page2 -X 4.45 -Y 2.58 -Width 0.55 -Height 0.28 -Text 'x_I' -FontSize 10.5 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page2 -X 9.25 -Y 3.75 -Width 1.30 -Height 0.30 -Text 'F_LQI' -FontSize 10.5 -Style 2 -FontName 'Times New Roman'
$null = Add-Text -Page $page2 -X 0.72 -Y 1.42 -Width 1.25 -Height 0.45 -Text "执行器偏差`nΔF" -FontSize 9.5
$null = Add-Text -Page $page2 -X 5.55 -Y 5.42 -Width 2.55 -Height 0.28 -Text 'F_LQI（观测器模型输入）' -FontSize 9.5

Invoke-ComRetry { $document.SaveAs($outputVsdx) } | Out-Null
Invoke-ComRetry { $page1.Export($outputPng1) } | Out-Null
Invoke-ComRetry { $page2.Export($outputPng2) } | Out-Null
try { $visio.ActiveWindow.Page = $page1 } catch { }

function Get-PageVerification {
    param($Page)
    $connectors = @($Page.Shapes) | Where-Object { $_.OneD -ne 0 }
    $glued = 0
    $orthogonal = 0
    $straight = 0
    $dashed = 0
    foreach ($connector in $connectors) {
        $beginFormula = $connector.CellsU('BeginX').FormulaU
        $endFormula = $connector.CellsU('EndX').FormulaU
        if (($beginFormula -match 'PAR\(' -or $beginFormula -match 'Sheet\.') -and ($endFormula -match 'PAR\(' -or $endFormula -match 'Sheet\.')) {
            $glued++
        }
        if ($connector.CellExistsU('ConLineRouteExt', 0) -ne 0) {
            $route = [int][math]::Round($connector.CellsU('ConLineRouteExt').ResultIU)
            if ($route -eq 0) { $orthogonal++ }
            if ($route -eq 1) { $straight++ }
        }
        if ([int][math]::Round($connector.CellsU('LinePattern').ResultIU) -eq 2) { $dashed++ }
    }
    return [pscustomobject]@{
        page = $Page.Name
        shapes = $Page.Shapes.Count
        connectors = $connectors.Count
        gluedConnectors = $glued
        orthogonalConnectors = $orthogonal
        straightConnectors = $straight
        dashedConnectors = $dashed
    }
}

$verification = [pscustomobject]@{
    output = $outputVsdx
    pages = @((Get-PageVerification -Page $page1), (Get-PageVerification -Page $page2))
    png1Exists = Test-Path -LiteralPath $outputPng1
    png2Exists = Test-Path -LiteralPath $outputPng2
    timestamp = (Get-Date).ToString('s')
}
$verification | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $verificationPath -Encoding UTF8
$verification | ConvertTo-Json -Depth 5

