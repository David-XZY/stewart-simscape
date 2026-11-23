$ErrorActionPreference = 'Stop'
. "$env:USERPROFILE/.codex/skills/visio-automation/scripts/visio_helpers.ps1"
$out = $PSScriptRoot
$labels = @(
 "IHSID 求解与原始解验收`n求解状态、残差及物理检查",
 "分段五次 Hermite 重建`n连续参考轨迹采样 1001 点",
 "构建五次 B 样条表示`n16 个控制点，首尾各 3 点固定",
 "最小二乘拟合位姿轨迹`n求解中间 10 个自由控制点",
 "重建、物理复查与模式检查`n合格并去重后提取 60 维标签"
)
$nodes = @()
for ($i=0; $i -lt $labels.Count; $i++) {
 $nodes += [pscustomobject]@{ id="n$i"; text=$labels[$i]; x=1.95; y=(4.95-1.1*$i); width=3.35; height=0.78; semanticType='process' }
}
$graph = [pscustomobject]@{diagramType='flowchart'; page=@{width=3.9;height=5.5}; nodes=$nodes; edges=@(for($i=0;$i -lt 4;$i++){@{from="n$i";to="n$($i+1)"}})}
$graph | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath "$out/clean.graph.json" -Encoding utf8
$app = New-VisibleVisioApplication
$doc = $app.Documents.Add('')
$page = $doc.Pages.Item(1)
$page.Name='IHSID 到 B 样条'
$page.PageSheet.CellsU('PageWidth').FormulaU='3.9 in'
$page.PageSheet.CellsU('PageHeight').FormulaU='5.5 in'
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
 $shapes += $s
}
$connectors=@()
for($i=0;$i -lt 4;$i++){
 $connectors += Connect-VisioShapesOrthogonal -Page $page -ConnectorMaster $stencil.Masters.ItemU('Dynamic connector') -From $shapes[$i] -To $shapes[$i+1] -FromX .5 -FromY 0 -ToX .5 -ToY 1
}
$doc.SaveAs("$out/ihsid_to_bspline_clean.vsdx")
try { $app.Settings.SetRasterExportResolution(2,180,180,0) } catch {}
$page.Export("$out/ihsid_to_bspline_clean.png")
$audit=@($connectors | ForEach-Object { [pscustomobject]@{id=$_.ID; oneD=$_.OneD; begin=$_.CellsU('BeginX').FormulaU; end=$_.CellsU('EndX').FormulaU; connections=$_.Connects.Count} })
if(@($audit | Where-Object {$_.oneD -eq 0 -or $_.connections -lt 2}).Count){throw 'Connector verification failed'}
[pscustomobject]@{shapes=$page.Shapes.Count; processNodes=$shapes.Count; connectors=$audit; output="$out/ihsid_to_bspline_clean.vsdx"} | ConvertTo-Json -Depth 6 | Set-Content "$out/clean.verification.json" -Encoding utf8
Get-Item "$out/ihsid_to_bspline_clean.vsdx","$out/ihsid_to_bspline_clean.png" | Select-Object Name,Length