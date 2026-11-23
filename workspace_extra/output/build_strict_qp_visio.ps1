param([switch]$SkipFormulaPaste)
$ErrorActionPreference = 'Stop'

$outputPath = 'D:\NPU\Matlab_code\output\documents\Stewart平台_严格CLF-CBF-QP控制原理图.vsdx'
$wordPath = 'D:\NPU\Matlab_code\output\documents\第四章_4.3_Stewart平台严格CLF-CBF-QP在线控制.docx'
$helperPath = 'C:\Users\David_X\.codex\skills\visio-automation\scripts\visio_helpers.ps1'

# Windows PowerShell 5 must load this UTF-8 helper through an explicit decoder.
$helperText = [IO.File]::ReadAllText($helperPath, [Text.Encoding]::UTF8)
. ([scriptblock]::Create($helperText))

$visio = New-VisibleVisioApplication
$word = $null
$wordCreated = $false
$visioDoc = $null
$wordDoc = $null

function Add-Process {
    param(
        [Parameter(Mandatory = $true)] $Page,
        [Parameter(Mandatory = $true)] $Master,
        [Parameter(Mandatory = $true)] [string] $Text,
        [double] $X,
        [double] $Y,
        [double] $Width,
        [double] $Height,
        [string] $Fill = 'RGB(236,243,250)'
    )
    $shape = $Page.Drop($Master, $X, $Y)
    Set-VisioShapeBounds -Shape $shape -X $X -Y $Y -Width $Width -Height $Height
    $shape.Text = $Text
    Set-VisioShapeFill -Shape $shape -Fill $Fill -Line 'RGB(69,103,135)' -LineWeight '1 pt'
    Set-VisioTextStyle -Shape $shape -Color 'RGB(35,35,35)' -Size 10 -Bold 0
    return $shape
}

function Add-WordEquationToVisio {
    param(
        [Parameter(Mandatory = $true)] $WordDocument,
        [int] $EquationIndex,
        [Parameter(Mandatory = $true)] $Page,
        [double] $X,
        [double] $Y,
        [double] $Width,
        [double] $Height
    )
    $before = $Page.Shapes.Count
    $WordDocument.OMaths.Item($EquationIndex).Range.Copy()
    Start-Sleep -Milliseconds 250
    $Page.PasteToLocation($X, $Y, 0)
    Start-Sleep -Milliseconds 250
    if ($Page.Shapes.Count -le $before) {
        throw "Word OMML equation $EquationIndex was not pasted into Visio."
    }
    $shape = $Page.Shapes.Item($Page.Shapes.Count)
    Set-VisioShapeBounds -Shape $shape -X $X -Y $Y -Width $Width -Height $Height
    return $shape
}

try {
    $visioDoc = $visio.Documents.Add('')
    $page = $visioDoc.Pages.Item(1)
    $page.Name = '严格CLF-CBF-QP控制框图'
    $page.PageSheet.CellsU('PageWidth').FormulaU = '11.69 in'
    $page.PageSheet.CellsU('PageHeight').FormulaU = '8.27 in'
    $page.PageSheet.CellsU('PageScale').FormulaU = '1 in'
    $page.PageSheet.CellsU('DrawingScale').FormulaU = '1 in'

    $stencil = Open-VisioStencilReadOnly -Visio $visio -StencilNameOrPath 'BASFLO_M.VSSX'
    $processMaster = $stencil.Masters.ItemU('Process')
    $connectorMaster = $stencil.Masters.ItemU('Dynamic connector')

    $reference = Add-Process -Page $page -Master $processMaster -Text "参考轨迹`n位姿、速度、加速度" -X 1.15 -Y 4.15 -Width 1.55 -Height 0.82 -Fill 'RGB(230,241,252)'
    $nominal = Add-Process -Page $page -Master $processMaster -Text "计算力矩-LQI`n名义力生成" -X 3.25 -Y 4.15 -Width 1.75 -Height 0.90 -Fill 'RGB(232,244,234)'
    $qp = Add-Process -Page $page -Master $processMaster -Text "严格CLF-CBF-QP`n在线优化" -X 5.80 -Y 4.15 -Width 1.85 -Height 0.95 -Fill 'RGB(255,243,219)'
    $actuator = Add-Process -Page $page -Master $processMaster -Text "力执行器" -X 8.05 -Y 4.15 -Width 1.30 -Height 0.72 -Fill 'RGB(242,237,249)'
    $platform = Add-Process -Page $page -Master $processMaster -Text "Stewart平台" -X 10.10 -Y 4.15 -Width 1.45 -Height 0.78 -Fill 'RGB(234,239,247)'
    $measurement = Add-Process -Page $page -Master $processMaster -Text "状态测量与`n速度观测" -X 10.10 -Y 1.65 -Width 1.55 -Height 0.80 -Fill 'RGB(230,241,252)'
    $model = Add-Process -Page $page -Master $processMaster -Text "非线性模型与`n约束余量计算" -X 6.55 -Y 1.65 -Width 1.90 -Height 0.82 -Fill 'RGB(239,245,248)'
    $clf = Add-Process -Page $page -Master $processMaster -Text "CLF跟踪约束`n允许松弛" -X 3.80 -Y 6.85 -Width 1.55 -Height 0.78 -Fill 'RGB(248,235,235)'
    $cbf = Add-Process -Page $page -Master $processMaster -Text "HOCBF硬约束`n力、腿长、腿速、加速度`n奇异性与碰撞间隙" -X 6.10 -Y 6.85 -Width 2.05 -Height 1.20 -Fill 'RGB(248,235,235)'
    $previousForce = Add-Process -Page $page -Master $processMaster -Text "上一时刻`n支链力" -X 8.05 -Y 6.55 -Width 1.35 -Height 0.74 -Fill 'RGB(242,237,249)'

    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $reference -To $nominal
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $nominal -To $qp
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $qp -To $actuator
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $actuator -To $platform
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $platform -To $measurement
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $measurement -To $model
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $model -To $qp
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $reference -To $clf
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $clf -To $qp
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $cbf -To $qp
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $previousForce -To $qp
    $null = Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $connectorMaster -From $model -To $cbf

    try {
        $word = [Runtime.InteropServices.Marshal]::GetActiveObject('Word.Application')
    } catch {
        $word = New-Object -ComObject Word.Application
        $word.Visible = $false
        $wordCreated = $true
    }
    $wordDoc = $word.Documents.Open($wordPath, $false, $true)
    if ($SkipFormulaPaste) {
        $formulaNominal = Add-Process -Page $page -Master $processMaster -Text 'Word原生OMML公式①`n名义力关系（见4.3.1）' -X 3.25 -Y 2.95 -Width 2.35 -Height 0.52 -Fill 'RGB(255,255,255)'
        $formulaQp = Add-Process -Page $page -Master $processMaster -Text 'Word原生OMML公式⑧`nQP优化式（见4.3.4）' -X 5.80 -Y 2.85 -Width 3.85 -Height 0.58 -Fill 'RGB(255,255,255)'
        $formulaNominal.Name = 'WordOMML_名义力关系_粘贴预留'
        $formulaQp.Name = 'WordOMML_QP优化式_粘贴预留'
    } else {
        $formulaNominal = Add-WordEquationToVisio -WordDocument $wordDoc -EquationIndex 1 -Page $page -X 3.25 -Y 2.95 -Width 2.35 -Height 0.40
        $formulaQp = Add-WordEquationToVisio -WordDocument $wordDoc -EquationIndex 8 -Page $page -X 5.80 -Y 2.85 -Width 3.85 -Height 0.58
        $formulaNominal.Name = 'WordOMML_名义力关系'
        $formulaQp.Name = 'WordOMML_QP优化式'
    }

    $title = $page.DrawRectangle(0.45, 7.62, 11.20, 8.02)
    $title.Text = '图4.4 Stewart平台严格CLF-CBF-QP在线控制原理图'
    Set-VisioShapeFill -Shape $title -Fill 'RGB(255,255,255)' -Line 'RGB(255,255,255)' -LineWeight '0 pt'
    Set-VisioTextStyle -Shape $title -Color 'RGB(20,20,20)' -Size 14 -Bold 1

    $visioDoc.SaveAs($outputPath)
    $pageCount = $visioDoc.Pages.Count
    $shapeCount = $page.Shapes.Count
    $connectorCount = @($page.Shapes | Where-Object { $_.OneD -ne 0 }).Count
    [pscustomobject]@{
        Output = $outputPath
        Pages = $pageCount
        Shapes = $shapeCount
        Connectors = $connectorCount
        FormulaNominalType = $formulaNominal.Type
        FormulaQpType = $formulaQp.Type
        WordEquationCount = $wordDoc.OMaths.Count
    } | ConvertTo-Json -Compress
} finally {
    if ($null -ne $wordDoc) { $wordDoc.Close(0) }
    if ($wordCreated -and $null -ne $word) { $word.Quit() }
    # Keep the task-created Visio window open so the user can edit the diagram.
}
