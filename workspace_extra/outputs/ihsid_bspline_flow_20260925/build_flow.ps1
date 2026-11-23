$ErrorActionPreference = 'Stop'
. "$env:USERPROFILE/.codex/skills/visio-automation/scripts/visio_helpers.ps1"
$out = $PSScriptRoot
$labels = @(
 "01  IHSID 离散解`n节点位姿、速度、加速度与力`n保留原始解及任务信息",
 "02  原始解合格检查`n求解状态＋离散残差与变量界`n独立物理检查全部通过",
 "03  分段五次 Hermite 重建`n匹配每段两端的位姿、速度、加速度`n得到连续参考轨迹",
 "04  采样与 B 样条基底`n参考轨迹均匀采样 1001 点`n五次／16 控制点／夹持均匀节点",
 "05  固定边界并最小二乘拟合`n首 3 点＝起点，末 3 点＝终点`n拟合中间 10 点，最小化位姿误差",
 "06  重建拟合轨迹`n控制点 × 基函数 → 位姿轨迹`n解析求导，并重新计算逆动力学力",
 "07  拟合结果重新验收`n完整物理检查＋模式偏差检查`n不合格：保留原解，不作为训练标签",
 "08  合格且去重后提取标签`n10 个自由控制点 × 6 维`n得到 60 维训练目标"
)
$nodes = @()
for ($i=0; $i -lt $labels.Count; $i++) {
 $nodes += [pscustomobject]@{ id="n$i"; text=$labels[$i]; x=1.95; y=(11.6-1.45*$i); width=3.35; height=1.08; semanticType='process' }
}
$graph = [pscustomobject]@{diagramType='flowchart'; page=@{width=3.9;height=12.95}; nodes=$nodes; edges=@(for($i=0;$i -lt 7;$i++){@{from="n$i";to="n$($i+1)"}})}
$graph | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath "$out/flow.graph.json" -Encoding utf8
$app = New-VisibleVisioApplication
$doc = $app.Documents.Add('')
$page = $doc.Pages.Item(1)
$page.Name='IHSID 到 B 样条'
$page.PageSheet.CellsU('PageWidth').FormulaU='3.9 in'
$page.PageSheet.CellsU('PageHeight').FormulaU='12.95 in'
$stencil=Open-VisioStencilReadOnly -Visio $app -StencilNameOrPath 'BASFLO_M.VSSX'
$shapes=@()
foreach($node in $nodes){
 $s=$page.Drop($stencil.Masters.ItemU('Process'),$node.x,$node.y)
 Set-VisioShapeBounds -Shape $s -X $node.x -Y $node.y -Width $node.width -Height $node.height
 $s.Text=$node.text
 Set-VisioShapeFill -Shape $s -Fill 'RGB(242,247,252)' -Line 'RGB(66,104,145)' -LineWeight '1 pt'
 Set-VisioTextStyle -Shape $s -Size 11 -Color 'RGB(27,47,67)'
 $s.CellsU('Char.Font').FormulaU='FONT("Microsoft YaHei")'
 $s.CellsU('LeftMargin').FormulaU='0.06 in'
 $s.CellsU('RightMargin').FormulaU='0.06 in'
 if($shapes.Count -eq 7){Set-VisioShapeFill -Shape $s -Fill 'RGB(229,244,240)' -Line 'RGB(38,119,104)'}
 $shapes += $s
}
$connectors=@()
for($i=0;$i -lt 7;$i++){
 $connectors += Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $stencil.Masters.ItemU('Dynamic connector') -From $shapes[$i] -To $shapes[$i+1] -FromX .5 -FromY 0 -ToX .5 -ToY 1
}
$title=Add-VisioLabel -Page $page -Text "IHSID → 五次 B 样条" -X 1.95 -Y 12.58 -Width 3.6 -Height .35
Set-VisioTextStyle -Shape $title -Size 16 -Bold 1 -Color 'RGB(27,47,67)'
$title.CellsU('Char.Font').FormulaU='FONT("Microsoft YaHei")'
$note=Add-VisioLabel -Page $page -Text '16 控制点表示；拟合近似不等于原解' -X 1.95 -Y .45 -Width 3.5 -Height .3
Set-VisioTextStyle -Shape $note -Size 9 -Color 'RGB(90,100,110)'
$note.CellsU('Char.Font').FormulaU='FONT("Microsoft YaHei")'
$doc.SaveAs("$out/ihsid_to_bspline.vsdx")
try { $app.Settings.SetRasterExportResolution(2,180,180,0) } catch {}
$page.Export("$out/ihsid_to_bspline.png")
$audit=@($connectors | ForEach-Object { [pscustomobject]@{id=$_.ID; oneD=$_.OneD; begin=$_.CellsU('BeginX').FormulaU; end=$_.CellsU('EndX').FormulaU; connections=$_.Connects.Count} })
if(@($audit | Where-Object {$_.oneD -eq 0 -or $_.connections -lt 2}).Count){throw 'Connector verification failed'}
[pscustomobject]@{shapes=$page.Shapes.Count; processNodes=$shapes.Count; connectors=$audit; output="$out/ihsid_to_bspline.vsdx"} | ConvertTo-Json -Depth 6 | Set-Content "$out/verification.json" -Encoding utf8
Get-Item "$out/ihsid_to_bspline.vsdx","$out/ihsid_to_bspline.png" | Select-Object Name,Length
