$ErrorActionPreference='Stop'
. "$env:USERPROFILE\.codex\skills\visio-automation\scripts\visio_helpers.ps1"
$v=New-VisibleVisioApplication
$st=Open-VisioStencilReadOnly -Visio $v -StencilNameOrPath 'BASFLO_M.VSSX'
$palette=@{blue=@('RGB(236,243,253)','RGB(49,91,151)');purple=@('RGB(242,237,252)','RGB(111,74,169)');green=@('RGB(234,246,241)','RGB(49,123,98)');ink=@('','RGB(28,43,65)');muted=@('','RGB(94,109,127)')}
function Font($s,$size=13,$bold=0,$color='RGB(28,43,65)'){
    Set-VisioTextStyle -Shape $s -Color $color -Size $size -Bold $bold
    $s.CellsU('Char.Font').FormulaU='FONT("Microsoft YaHei")'
    if($s.CellExistsU('Char.AsianFont',0)){$s.CellsU('Char.AsianFont').FormulaU='FONT("Microsoft YaHei")'}
    $s.CellsU('Para.HorzAlign').FormulaU='1'
    $s.CellsU('VerticalAlign').FormulaU='1'
}
$reports=@()
foreach($kind in @('baseline','experiment')){
    $g=Get-Content -LiteralPath (Join-Path $PSScriptRoot "$kind.graph.json") -Raw -Encoding utf8 | ConvertFrom-Json
    $path=Join-Path $PSScriptRoot ($g.name+'.vsdx')
    $d=Get-OpenVisioDocumentByPath $path
    if($null -eq $d){$d=$v.Documents.Add('')}
    $p=$d.Pages.Item(1);$p.Name=$g.name
    for($i=$p.Shapes.Count;$i -ge 1;$i--){$p.Shapes.Item($i).Delete()}
    $p.PageSheet.CellsU('PageWidth').ResultIU=$g.width
    $p.PageSheet.CellsU('PageHeight').ResultIU=$g.height
    $bg=$p.DrawRectangle(0,0,$g.width,$g.height)
    $bg.CellsU('LinePattern').FormulaU='0'
    $bg.CellsU('FillForegnd').FormulaU='RGB(255,255,255)'
    $bg.CellsU('FillPattern').FormulaU='1'
    $bg.SendToBack()
    $nodes=@{}
    foreach($n in $g.nodes){
        $s=$p.Drop($st.Masters.ItemU($n.master),$n.x,$n.y)
        Set-VisioShapeBounds -Shape $s -X $n.x -Y $n.y -Width $n.w -Height $n.h
        $s.Text=$n.text
        Set-VisioShapeFill -Shape $s -Fill $palette[$n.color][0] -Line $palette[$n.color][1] -LineWeight '1.3 pt'
        Font $s 13
        $s.CellsU('Rounding').FormulaU='0.08 in'
        $s.CellsU('LeftMargin').ResultIU=.09;$s.CellsU('RightMargin').ResultIU=.09
        $nodes[$n.id]=$s
    }
    $cs=@()
    foreach($e in $g.edges){
        $args=@{Page=$p;ConnectorMaster=$st.Masters.ItemU('Dynamic connector');From=$nodes[$e.a];To=$nodes[$e.b];EndArrow=13;LineColor='RGB(74,95,120)'}
        if($e.side){$args.FromX=$e.side[0];$args.FromY=$e.side[1];$args.ToX=$e.side[2];$args.ToY=$e.side[3]}
        $c=Connect-VisioShapesOrthogonal @args
        $c.CellsU('LineWeight').FormulaU='1.25 pt'
        if($e.dashed){$c.CellsU('LinePattern').FormulaU='2'}
        if($e.label){$c.Text=$e.label;Font $c 10 0 'RGB(74,95,120)';$c.CellsU('TextBkgnd').FormulaU='2'}
        $cs+= $c
    }
    foreach($a in $g.labels){
        $s=Add-VisioLabel -Page $p -Text $a.text -X $a.x -Y $a.y -Width $a.w -Height $a.h
        Font $s $a.size ([int]$a.bold) $palette[$a.color][1]
    }
    $path=Join-Path $PSScriptRoot ($g.name+'.vsdx')
    $d.SaveAs($path)
    $null=$v.Settings.SetRasterExportResolution(2,180,180,0)
    $p.Export((Join-Path $PSScriptRoot ($g.name+'.png')))
    $p.Export((Join-Path $PSScriptRoot ($g.name+'.svg')))
    $connections=@($cs | ForEach-Object { [pscustomobject]@{id=$_.ID;oneD=$_.OneD;begin=$_.CellsU('BeginX').FormulaU;end=$_.CellsU('EndX').FormulaU;glueCount=$_.Connects.Count;route=$_.CellsU('ConLineRouteExt').ResultIU} })
    if(@($connections | Where-Object {$_.oneD -eq 0 -or $_.glueCount -ne 2 -or $_.route -ne 0}).Count -gt 0){throw 'Connector glue verification failed'}
    $reports += [pscustomobject]@{file=$path;nodes=$g.nodes.Count;connectors=$cs.Count;connections=$connections;saved=$d.Saved}
    if($null -ne $d.Application.ActiveWindow){$d.Application.ActiveWindow.ViewFit=1}
}
$reports | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'verification.json') -Encoding utf8
$reports | Select-Object file,nodes,connectors,saved | ConvertTo-Json
