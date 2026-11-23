param(
  [switch] $Invisible
)

$ErrorActionPreference = 'Stop'

$helperPath = Join-Path $env:USERPROFILE '.codex\skills\visio-automation\scripts\visio_helpers.ps1'
$helperText = [System.IO.File]::ReadAllText($helperPath, [System.Text.Encoding]::UTF8)
. ([scriptblock]::Create($helperText))

$workspaceRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
$outputPath = Join-Path $workspaceRoot 'Stewart平台_PID闭环控制原理图.vsdx'
$previewPath = Join-Path $workspaceRoot 'Stewart平台_PID闭环控制原理图.png'
$svgPath = Join-Path $workspaceRoot 'Stewart平台_PID闭环控制原理图.svg'
$reportPath = Join-Path $workspaceRoot 'Stewart平台_PID闭环控制原理图_验证.json'
$reference1 = 'C:\Users\David_X\AppData\Local\Temp\codex-clipboard-52deab9d-9d68-47ea-b89a-3bd4da50f3bf.png'
$reference2 = 'C:\Users\David_X\AppData\Local\Temp\codex-clipboard-fcea451e-02df-4b38-ab63-1900ff619384.png'

function Set-IU {
  param($Shape, [string] $Cell, [double] $Value)
  $Shape.CellsU($Cell).ResultIU = $Value
}

function Set-TextAppearance {
  param(
    [Parameter(Mandatory = $true)] $Shape,
    [double] $Size = 12,
    [string] $Font = 'Microsoft YaHei',
    [switch] $Italic,
    [switch] $Bold
  )
  Set-VisioTextStyle -Shape $Shape -Color 'RGB(0,0,0)' -Size $Size -Bold ($(if ($Bold) { 1 } else { 0 }))
  $Shape.CellsU('Char.Font').FormulaU = 'FONT("' + $Font + '")'
  if ($Italic) { $Shape.CellsU('Char.Style').FormulaU = '2' }
  $Shape.CellsU('Para.HorzAlign').FormulaU = '1'
  $Shape.CellsU('VerticalAlign').FormulaU = '1'
}

function Add-Label {
  param(
    $Page, [string] $Text, [double] $X, [double] $Y,
    [double] $Width = 1.2, [double] $Height = 0.35,
    [double] $Size = 12, [string] $Font = 'Microsoft YaHei',
    [switch] $Italic, [switch] $Bold
  )
  $shape = Add-VisioLabel -Page $Page -Text $Text -X $X -Y $Y -Width $Width -Height $Height
  Set-TextAppearance -Shape $shape -Size $Size -Font $Font -Italic:$Italic -Bold:$Bold
  $shape.CellsU('LeftMargin').ResultIU = 0
  $shape.CellsU('RightMargin').ResultIU = 0
  $shape.CellsU('TopMargin').ResultIU = 0
  $shape.CellsU('BottomMargin').ResultIU = 0
  return $shape
}

function Add-ProcessBlock {
  param($Page, $Master, [string] $Text, [double] $X, [double] $Y, [double] $Width, [double] $Height, [double] $Size = 13)
  $shape = $Page.Drop($Master, $X, $Y)
  Set-VisioShapeBounds -Shape $shape -X $X -Y $Y -Width $Width -Height $Height
  $shape.Text = $Text
  Set-VisioShapeFill -Shape $shape -Fill 'RGB(255,255,255)' -Line 'RGB(0,0,0)' -LineWeight '1.5 pt'
  Set-TextAppearance -Shape $shape -Size $Size
  return $shape
}

function Add-SumNode {
  param($Page, [double] $X, [double] $Y, [double] $Diameter)
  $r = $Diameter / 2.0
  $shape = $Page.DrawOval($X-$r, $Y-$r, $X+$r, $Y+$r)
  Set-VisioShapeFill -Shape $shape -Fill 'RGB(255,255,255)' -Line 'RGB(0,0,0)' -LineWeight '1.5 pt'
  $shape.Text = '×'
  Set-TextAppearance -Shape $shape -Size 24 -Font 'Times New Roman'
  return $shape
}

function Add-Junction {
  param($Page, [double] $X, [double] $Y, [switch] $Hidden)
  $r = if ($Hidden) { 0.012 } else { 0.045 }
  $shape = $Page.DrawOval($X-$r, $Y-$r, $X+$r, $Y+$r)
  if ($Hidden) {
    $shape.CellsU('LinePattern').FormulaU = '0'
    $shape.CellsU('FillPattern').FormulaU = '0'
  } else {
    $shape.CellsU('LinePattern').FormulaU = '0'
    $shape.CellsU('FillPattern').FormulaU = '1'
    $shape.CellsU('FillForegnd').FormulaU = 'RGB(0,0,0)'
  }
  return $shape
}

function Add-StraightConnector {
  param($Page, $Master, $From, $To, [double] $FromX, [double] $FromY, [double] $ToX, [double] $ToY, [int] $Arrow = 13)
  $c = Connect-VisioShapesStraight -Page $Page -ConnectorMaster $Master -From $From -To $To -FromX $FromX -FromY $FromY -ToX $ToX -ToY $ToY -EndArrow $Arrow -LineColor 'RGB(0,0,0)'
  $c.CellsU('LineWeight').FormulaU = '1.5 pt'
  return $c
}

$visio = $null
$document = $null
$runInvisible = [bool] $Invisible

try {
  foreach ($path in @($outputPath, $reportPath)) {
    if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path -Force }
  }

  $visio = if ($runInvisible) { New-InvisibleVisioApplication } else { New-VisibleVisioApplication }
  $document = $visio.Documents.Add('')
  $page = $visio.ActivePage
  $page.Name = 'Stewart平台PID闭环控制'
  Set-IU $page.PageSheet 'PageWidth' 15.5
  Set-IU $page.PageSheet 'PageHeight' 8.5
  Set-IU $page.PageSheet 'DrawingScale' 1
  Set-IU $page.PageSheet 'PageScale' 1

  $stencil = Open-VisioStencilReadOnly -Visio $visio -StencilNameOrPath 'BASFLO_M.VSSX'
  $processMaster = $stencil.Masters.ItemU('Process')
  $connectorMaster = $stencil.Masters.ItemU('Dynamic connector')

  $mainY = 4.80
  $sum1X = 4.12
  $sum2X = 8.62
  $sumD = 0.64

  # 主体节点
  $input = Add-Junction -Page $page -X 0.45 -Y $mainY -Hidden
  $inverse = Add-ProcessBlock -Page $page -Master $processMaster -Text "运动学`n逆解" -X 2.15 -Y $mainY -Width 1.80 -Height 1.05 -Size 14
  $sum1 = Add-SumNode -Page $page -X $sum1X -Y $mainY -Diameter $sumD
  $pBlock = Add-ProcessBlock -Page $page -Master $processMaster -Text '比例  P' -X 6.35 -Y 6.30 -Width 1.55 -Height 0.90 -Size 13
  $iBlock = Add-ProcessBlock -Page $page -Master $processMaster -Text '积分  I' -X 6.35 -Y $mainY -Width 1.55 -Height 0.90 -Size 13
  $dBlock = Add-ProcessBlock -Page $page -Master $processMaster -Text '微分  D' -X 6.35 -Y 3.30 -Width 1.55 -Height 0.90 -Size 13
  $sum2 = Add-SumNode -Page $page -X $sum2X -Y $mainY -Diameter $sumD
  $plant = Add-ProcessBlock -Page $page -Master $processMaster -Text "Stewart`n平台" -X 10.80 -Y $mainY -Width 2.00 -Height 1.05 -Size 14
  $takeoff = Add-Junction -Page $page -X 12.55 -Y $mainY
  $output = Add-Junction -Page $page -X 14.72 -Y $mainY -Hidden

  # 主通道连接线
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $input -To $inverse -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $inverse -To $sum1 -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13

  # 误差信号分支：使用粘附的隐形中继点保持正交几何
  $bus = Add-Junction -Page $page -X 5.08 -Y $mainY
  $busTop = Add-Junction -Page $page -X 5.08 -Y 6.30 -Hidden
  $busBottom = Add-Junction -Page $page -X 5.08 -Y 3.30 -Hidden
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $sum1 -To $bus -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $bus -To $busTop -FromX 0.5 -FromY 1 -ToX 0.5 -ToY 0 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $busTop -To $pBlock -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $bus -To $iBlock -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $bus -To $busBottom -FromX 0.5 -FromY 0 -ToX 0.5 -ToY 1 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $busBottom -To $dBlock -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13

  # PID 三路合成
  $mergeTop = Add-Junction -Page $page -X $sum2X -Y 6.30 -Hidden
  $mergeBottom = Add-Junction -Page $page -X $sum2X -Y 3.30 -Hidden
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $pBlock -To $mergeTop -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $mergeTop -To $sum2 -FromX 0.5 -FromY 0 -ToX 0.5 -ToY 1 -Arrow 13
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $iBlock -To $sum2 -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $dBlock -To $mergeBottom -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $mergeBottom -To $sum2 -FromX 0.5 -FromY 1 -ToX 0.5 -ToY 0 -Arrow 13

  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $sum2 -To $plant -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $plant -To $takeoff -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $takeoff -To $output -FromX 1 -FromY 0.5 -ToX 0 -ToY 0.5 -Arrow 13

  # 输出杆长负反馈
  $feedbackY = 1.70
  $feedbackRight = Add-Junction -Page $page -X 12.55 -Y $feedbackY -Hidden
  $feedbackLeft = Add-Junction -Page $page -X $sum1X -Y $feedbackY -Hidden
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $takeoff -To $feedbackRight -FromX 0.5 -FromY 0 -ToX 0.5 -ToY 1 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $feedbackRight -To $feedbackLeft -FromX 0 -FromY 0.5 -ToX 1 -ToY 0.5 -Arrow 0
  $null = Add-StraightConnector -Page $page -Master $connectorMaster -From $feedbackLeft -To $sum1 -FromX 0.5 -FromY 1 -ToX 0.5 -ToY 0 -Arrow 13

  # 标签
  $null = Add-Label -Page $page -Text 'r(t)' -X 0.95 -Y 5.42 -Width 1.0 -Height 0.42 -Size 20 -Font 'Times New Roman' -Italic
  $null = Add-Label -Page $page -Text '期望轨迹' -X 0.75 -Y 4.24 -Width 1.25 -Height 0.38 -Size 13
  $null = Add-Label -Page $page -Text '+' -X 3.63 -Y 5.18 -Width 0.25 -Height 0.28 -Size 14 -Font 'Times New Roman'
  $null = Add-Label -Page $page -Text '−' -X 3.72 -Y 4.30 -Width 0.25 -Height 0.28 -Size 15 -Font 'Times New Roman'
  $null = Add-Label -Page $page -Text 'e(t)' -X 4.65 -Y 5.35 -Width 0.85 -Height 0.42 -Size 19 -Font 'Times New Roman' -Italic
  $null = Add-Label -Page $page -Text 'PID 控制器' -X 6.35 -Y 7.10 -Width 1.90 -Height 0.38 -Size 14
  $null = Add-Label -Page $page -Text 'u(t)' -X 9.22 -Y 5.38 -Width 0.90 -Height 0.42 -Size 19 -Font 'Times New Roman' -Italic
  $null = Add-Label -Page $page -Text 'y(t)' -X 13.90 -Y 5.42 -Width 1.00 -Height 0.42 -Size 20 -Font 'Times New Roman' -Italic
  $null = Add-Label -Page $page -Text '输出轨迹' -X 14.00 -Y 4.25 -Width 1.35 -Height 0.38 -Size 13
  $null = Add-Label -Page $page -Text '输出杆长' -X 8.35 -Y 1.98 -Width 1.60 -Height 0.38 -Size 13
  $null = Add-Label -Page $page -Text 'Stewart 平台 PID 闭环控制原理图' -X 7.75 -Y 0.88 -Width 6.50 -Height 0.42 -Size 14
  $null = Add-Label -Page $page -Text 'Closed-loop PID Control Principle of Stewart Platform' -X 7.75 -Y 0.40 -Width 7.20 -Height 0.38 -Size 13 -Font 'Times New Roman'

  # 第二页保留两张原始参考图，便于后续在 Visio 内对照
  $referencePage = $document.Pages.Add()
  $referencePage.Name = '参考图'
  Set-IU $referencePage.PageSheet 'PageWidth' 15.5
  Set-IU $referencePage.PageSheet 'PageHeight' 8.5
  if (Test-Path -LiteralPath $reference1) {
    $img1 = $referencePage.Import($reference1)
    Set-VisioShapeBounds -Shape $img1 -X 4.00 -Y 4.25 -Width 7.10 -Height 4.45
  }
  if (Test-Path -LiteralPath $reference2) {
    $img2 = $referencePage.Import($reference2)
    Set-VisioShapeBounds -Shape $img2 -X 11.50 -Y 4.25 -Width 7.10 -Height 3.95
  }
  $null = Add-Label -Page $referencePage -Text '原始参考图（已嵌入，仅用于对照）' -X 7.75 -Y 7.55 -Width 5.20 -Height 0.42 -Size 15 -Bold

  $document.SaveAs($outputPath)

  $connectors = @($page.Shapes) | Where-Object { $_.OneD -ne 0 }
  $glued = @($connectors | Where-Object {
    $_.CellsU('BeginX').FormulaU -match 'GUARD|PAR|Sheet' -and $_.CellsU('EndX').FormulaU -match 'GUARD|PAR|Sheet'
  })
  $straight = @($connectors | Where-Object {
    $_.CellExistsU('ConLineRouteExt', 0) -ne 0 -and [int]$_.CellsU('ConLineRouteExt').ResultIU -eq 1
  })
  $report = [ordered]@{
    output = $outputPath
    preview = $previewPath
    preview_export = 'fallback renderer (Visio Page.Export unavailable on this machine)'
    pages = $document.Pages.Count
    main_page_shapes = $page.Shapes.Count
    connectors = $connectors.Count
    glued_connectors = $glued.Count
    straight_connectors = $straight.Count
    reference_page_embedded_images = @($referencePage.Shapes | Where-Object { $_.Type -eq 4 }).Count
    generated_at = (Get-Date).ToString('s')
  }
  [System.IO.File]::WriteAllText($reportPath, ($report | ConvertTo-Json -Depth 4), (New-Object System.Text.UTF8Encoding($false)))

  try { $visio.ActiveWindow.Page = $page } catch {}
  Write-Output ($report | ConvertTo-Json -Depth 4)
}
finally {
  if ($runInvisible) {
    if ($null -ne $document) { try { $document.Close() } catch {} }
    if ($null -ne $visio) { try { $visio.Quit() } catch {} }
  }
}
