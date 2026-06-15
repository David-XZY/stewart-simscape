param(
    [string]$OutputDir = ""
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($OutputDir)) {
    $OutputDir = Join-Path (Split-Path $PSScriptRoot -Parent) "docs\figures\visio_control_block_diagrams"
}
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$script:Visio = New-Object -ComObject Visio.Application
$script:Visio.Visible = $false
$script:Doc = $script:Visio.Documents.Add("")

function Set-Cell {
    param($Shape, [string]$Cell, [string]$Formula)
    $Shape.CellsU($Cell).FormulaU = $Formula
}

function Add-Box {
    param(
        $Page, [double]$X, [double]$Y, [double]$W, [double]$H,
        [string]$Text, [string]$Fill, [string]$Line = "RGB(92,103,112)",
        [double]$FontSize = 10, [switch]$Bold
    )
    $shape = $Page.DrawRectangle($X-$W/2, $Y-$H/2, $X+$W/2, $Y+$H/2)
    $shape.Text = $Text
    Set-Cell $shape "FillForegnd" $Fill
    Set-Cell $shape "FillPattern" "1"
    Set-Cell $shape "LineColor" $Line
    Set-Cell $shape "LineWeight" "0.012 in"
    Set-Cell $shape "Rounding" "0.04 in"
    Set-Cell $shape "Char.Size" "$FontSize pt"
    Set-Cell $shape "Char.Font" "FONT(`"Microsoft YaHei`")"
    if ($Bold) { Set-Cell $shape "Char.Style" "1" }
    Set-Cell $shape "Para.HorzAlign" "1"
    Set-Cell $shape "VerticalAlign" "1"
    return $shape
}

function Add-Text {
    param(
        $Page, [double]$X, [double]$Y, [double]$W, [double]$H,
        [string]$Text, [string]$Color = "RGB(38,50,56)",
        [double]$FontSize = 9.5, [switch]$Bold
    )
    $shape = $Page.DrawRectangle($X-$W/2, $Y-$H/2, $X+$W/2, $Y+$H/2)
    $shape.Text = $Text
    Set-Cell $shape "FillPattern" "0"
    Set-Cell $shape "LinePattern" "0"
    Set-Cell $shape "Char.Color" $Color
    Set-Cell $shape "Char.Size" "$FontSize pt"
    Set-Cell $shape "Char.Font" "FONT(`"Microsoft YaHei`")"
    if ($Bold) { Set-Cell $shape "Char.Style" "1" }
    Set-Cell $shape "Para.HorzAlign" "1"
    Set-Cell $shape "VerticalAlign" "1"
    return $shape
}

function Add-MathText {
    param(
        $Page, [double]$X, [double]$Y, [double]$W, [double]$H,
        [string]$Text, [string]$Color = "RGB(38,50,56)",
        [double]$FontSize = 10
    )
    $shape = Add-Text $Page $X $Y $W $H $Text $Color $FontSize
    Set-Cell $shape "Char.Font" "FONT(`"Cambria Math`")"
    Set-Cell $shape "Char.Style" "2"
    return $shape
}

function Add-Line {
    param(
        $Page, [double]$X1, [double]$Y1, [double]$X2, [double]$Y2,
        [string]$Color = "RGB(38,50,56)", [switch]$Arrow, [switch]$Dashed
    )
    $line = $Page.DrawLine($X1, $Y1, $X2, $Y2)
    Set-Cell $line "LineColor" $Color
    Set-Cell $line "LineWeight" "0.014 in"
    if ($Arrow) { Set-Cell $line "EndArrow" "13" }
    if ($Dashed) { Set-Cell $line "LinePattern" "2" }
    return $line
}

function Add-Path {
    param($Page, [array]$Points, [string]$Color = "RGB(22,134,111)", [switch]$Dashed)
    for ($index = 0; $index -lt $Points.Count - 1; $index++) {
        $isLast = $index -eq $Points.Count - 2
        Add-Line $Page $Points[$index][0] $Points[$index][1] `
            $Points[$index+1][0] $Points[$index+1][1] $Color -Arrow:$isLast -Dashed:$Dashed | Out-Null
    }
}

function Add-Sum {
    param($Page, [double]$X, [double]$Y, [string]$Mode)
    $circle = $Page.DrawOval($X-0.20, $Y-0.20, $X+0.20, $Y+0.20)
    Set-Cell $circle "FillForegnd" "RGB(255,255,255)"
    Set-Cell $circle "LineColor" "RGB(92,103,112)"
    Set-Cell $circle "LineWeight" "0.012 in"
    if ($Mode -eq "PM") {
        $circle.Text = "+`n−"
    } else {
        $circle.Text = "+`n+"
    }
    Set-Cell $circle "Char.Font" "FONT(`"Cambria Math`")"
    Set-Cell $circle "Char.Size" "8.5 pt"
    Set-Cell $circle "Para.HorzAlign" "1"
    Set-Cell $circle "VerticalAlign" "1"
}

function Add-Dot {
    param($Page, [double]$X, [double]$Y)
    $dot = $Page.DrawOval($X-0.04, $Y-0.04, $X+0.04, $Y+0.04)
    Set-Cell $dot "FillForegnd" "RGB(22,134,111)"
    Set-Cell $dot "LineColor" "RGB(22,134,111)"
}

function Add-Group {
    param($Page, [double]$X1, [double]$Y1, [double]$X2, [double]$Y2, [string]$Label)
    $shape = $Page.DrawRectangle($X1, $Y1, $X2, $Y2)
    Set-Cell $shape "FillPattern" "0"
    Set-Cell $shape "LineColor" "RGB(195,201,205)"
    Set-Cell $shape "LinePattern" "2"
    Set-Cell $shape "LineWeight" "0.008 in"
    Add-Text $Page (($X1+$X2)/2) ($Y1-0.16) ($X2-$X1) 0.25 $Label "RGB(92,103,112)" 8 -Bold | Out-Null
}

function Set-Page {
    param($Page, [string]$Name, [double]$Width, [double]$Height)
    $Page.Name = $Name
    $Page.PageSheet.CellsU("PageWidth").ResultIU = $Width
    $Page.PageSheet.CellsU("PageHeight").ResultIU = $Height
    $Page.PageSheet.CellsU("PageScale").ResultIU = 1
    $Page.PageSheet.CellsU("DrawingScale").ResultIU = 1
}

$ink = "RGB(38,50,56)"
$blue = "RGB(37,99,165)"
$teal = "RGB(22,134,111)"
$gold = "RGB(197,122,23)"
$violet = "RGB(107,88,165)"
$softBlue = "RGB(237,244,248)"
$softTeal = "RGB(238,248,245)"
$softGold = "RGB(251,245,232)"
$softViolet = "RGB(243,240,249)"

function Draw-Run02 {
    param($Page)
    Set-Page $Page "位姿反馈力控制" 15.7 5.6
    Add-Text $Page 7.85 5.2 6 0.4 "位姿反馈力控制" $ink 16 -Bold | Out-Null
    $y = 3.45
    Add-Sum $Page 1.5 $y "PM"
    Add-Box $Page 3.55 $y 1.8 0.9 "笛卡尔 PIDF`nKₓ(s)" $softTeal $ink 11 -Bold | Out-Null
    Add-Box $Page 6.15 $y 1.8 0.9 "固定力映射`nJᵥ(q₀)⁻ᵀ" $softViolet $ink 11 -Bold | Out-Null
    Add-Sum $Page 8.25 $y "PP"
    Add-Box $Page 10.25 $y 1.8 0.9 "理想力执行器" $softViolet $ink 10 -Bold | Out-Null
    Add-Box $Page 13.0 $y 1.9 0.9 "Stewart 平台`nGF→q" $softBlue $ink 10 -Bold | Out-Null

    Add-Line $Page 0.25 $y 1.32 $y $blue -Arrow | Out-Null
    Add-MathText $Page 0.62 ($y+0.24) 0.7 0.22 "qᵣₑf" $blue 10 | Out-Null
    Add-Line $Page 1.68 $y 2.65 $y $ink -Arrow | Out-Null
    Add-MathText $Page 2.15 ($y+0.24) 0.6 0.22 "eᵩ" $ink 10 | Out-Null
    Add-Line $Page 4.45 $y 5.25 $y $teal -Arrow | Out-Null
    Add-MathText $Page 4.85 ($y+0.24) 0.7 0.22 "ΔW" $teal 10 | Out-Null
    Add-Line $Page 7.05 $y 8.07 $y $teal -Arrow | Out-Null
    Add-MathText $Page 7.55 ($y+0.24) 0.8 0.22 "Fᶠᵇ" $teal 10 | Out-Null
    Add-Line $Page 8.43 $y 9.35 $y $ink -Arrow | Out-Null
    Add-MathText $Page 8.85 ($y+0.24) 0.9 0.22 "Fᶜᵐᵈ" $ink 10 | Out-Null
    Add-Line $Page 11.15 $y 12.05 $y $ink -Arrow | Out-Null
    Add-Line $Page 13.95 $y 15.7 $y $ink -Arrow | Out-Null
    Add-MathText $Page 14.65 ($y+0.24) 0.8 0.22 "qᵃᶜᵗ" $ink 10 | Out-Null
    Add-Line $Page 8.25 4.7 8.25 3.63 $gold -Arrow | Out-Null
    Add-MathText $Page 8.62 4.4 0.8 0.22 "Fᶠᶠ" $gold 10 | Out-Null
    Add-Dot $Page 14.7 $y
    Add-Path $Page @(@(14.7,$y),@(14.7,1.35),@(1.5,1.35),@(1.5,3.27)) $teal
    Add-Group $Page 2.45 2.75 7.25 4.18 "位姿反馈补偿"
}

function Draw-Run03 {
    param($Page)
    Set-Page $Page "理想腿长串级控制" 20.5 6.6
    Add-Text $Page 10.25 6.2 7 0.4 "理想腿长串级控制" $ink 16 -Bold | Out-Null
    $y = 4.0
    Add-Sum $Page 1.35 $y "PM"
    Add-Box $Page 3.0 $y 1.45 0.9 "位置 P`nKₚₒₛ" $softTeal $ink 10.5 -Bold | Out-Null
    Add-Sum $Page 4.75 $y "PP"
    Add-Box $Page 6.15 $y 1.35 0.9 "腿速限幅" $softGold $ink 9.5 -Bold | Out-Null
    Add-Box $Page 8.05 $y 1.45 0.9 "加速度限制" $softGold $ink 9.5 -Bold | Out-Null
    Add-Sum $Page 9.65 $y "PM"
    Add-Box $Page 11.3 $y 1.5 0.9 "腿速 PIDF`nKᵥ(z)" $softTeal $ink 10.5 -Bold | Out-Null
    Add-Box $Page 13.45 $y 1.55 0.9 "伺服加速度`n限制" $softGold $ink 9.5 -Bold | Out-Null
    Add-Box $Page 16.2 $y 2.1 0.95 "Length-Servo`n理想运动执行器" $softBlue $ink 9.5 -Bold | Out-Null

    Add-Line $Page 0.2 $y 1.17 $y $blue -Arrow | Out-Null
    Add-MathText $Page 0.55 ($y+0.24) 0.8 0.22 "Lᵣₑf" $blue 10 | Out-Null
    Add-Line $Page 1.53 $y 2.28 $y $ink -Arrow | Out-Null
    Add-MathText $Page 1.9 ($y+0.24) 0.6 0.22 "eL" $ink 10 | Out-Null
    Add-Line $Page 3.73 $y 4.57 $y $teal -Arrow | Out-Null
    Add-MathText $Page 4.1 ($y+0.24) 0.8 0.22 "ΔL̇" $teal 10 | Out-Null
    Add-Line $Page 4.93 $y 5.47 $y $ink -Arrow | Out-Null
    Add-MathText $Page 5.2 ($y+0.24) 0.9 0.22 "L̇ᵣₐw" $ink 10 | Out-Null
    Add-Line $Page 6.83 $y 7.33 $y $ink -Arrow | Out-Null
    Add-Line $Page 8.78 $y 9.47 $y $ink -Arrow | Out-Null
    Add-MathText $Page 9.1 ($y+0.24) 0.9 0.22 "L̇cmd" $ink 10 | Out-Null
    Add-Line $Page 9.83 $y 10.55 $y $ink -Arrow | Out-Null
    Add-MathText $Page 10.15 ($y+0.24) 0.8 0.22 "eL̇" $ink 10 | Out-Null
    Add-Line $Page 12.05 $y 12.68 $y $teal -Arrow | Out-Null
    Add-Line $Page 14.23 $y 15.15 $y $ink -Arrow | Out-Null
    Add-MathText $Page 14.65 ($y+0.24) 1.0 0.22 "L̇servo" $ink 10 | Out-Null
    Add-Line $Page 17.25 $y 20.0 $y $ink -Arrow | Out-Null
    Add-MathText $Page 18.3 ($y+0.24) 0.8 0.22 "Lact" $ink 10 | Out-Null
    Add-Line $Page 4.75 5.15 4.75 4.18 $gold -Arrow | Out-Null
    Add-MathText $Page 5.15 4.9 0.9 0.22 "L̇ref" $gold 10 | Out-Null

    Add-Dot $Page 18.7 $y
    Add-Path $Page @(@(18.7,$y),@(18.7,2.65),@(1.35,2.65),@(1.35,3.82)) $teal
    Add-Path $Page @(@(16.2,3.52),@(16.2,2.95),@(9.65,2.95),@(9.65,3.82)) $teal
    Add-MathText $Page 16.65 3.15 0.9 0.22 "L̇act" $teal 10 | Out-Null
    Add-Path $Page @(@(14.7,$y),@(14.7,5.15),@(11.3,5.15),@(11.3,4.45)) $teal -Dashed
    Add-Text $Page 13.95 5.38 2.2 0.25 "跟踪抗饱和回送" $teal 9.5 -Bold | Out-Null

    Add-Group $Page 0.9 3.25 5.15 4.72 "腿长位置外环"
    Add-Group $Page 5.35 3.25 8.85 4.72 "腿速参考整形"
    Add-Group $Page 9.2 3.25 14.4 4.72 "腿速内环"
}

function Draw-Run04 {
    param($Page)
    Set-Page $Page "位姿外环修正的理想腿长串级控制" 20.6 10.5
    Add-Text $Page 10.3 10.05 9 0.4 "位姿外环修正的理想腿长串级控制" $ink 16 -Bold | Out-Null
    $yt = 8.25
    Add-Sum $Page 1.35 $yt "PM"
    Add-Box $Page 3.0 $yt 1.45 0.9 "低通滤波`nLPF" $softTeal $ink 9.5 -Bold | Out-Null
    Add-Box $Page 5.25 $yt 1.65 0.95 "时变参考 Jacobian`nJq(qᵣₑf)" $softViolet $ink 10 -Bold | Out-Null
    Add-Box $Page 7.65 $yt 1.55 0.95 "位姿反馈增益`nKₚₒₛₑ" $softTeal $ink 10 -Bold | Out-Null
    Add-Box $Page 9.95 $yt 1.55 0.95 "腿长修正限幅`nsat" $softGold $ink 9.2 -Bold | Out-Null

    Add-Line $Page 0.2 $yt 1.17 $yt $blue -Arrow | Out-Null
    Add-MathText $Page 0.55 ($yt+0.24) 0.8 0.22 "qᵣₑf" $blue 10 | Out-Null
    Add-Line $Page 1.53 $yt 2.28 $yt $ink -Arrow | Out-Null
    Add-MathText $Page 1.9 ($yt+0.24) 0.6 0.22 "e_q" $ink 10 | Out-Null
    Add-Line $Page 3.73 $yt 4.42 $yt $ink -Arrow | Out-Null
    Add-Line $Page 6.08 $yt 6.88 $yt $violet -Arrow | Out-Null
    Add-Line $Page 8.43 $yt 9.18 $yt $teal -Arrow | Out-Null
    Add-Group $Page 0.9 7.5 10.8 9.02 "位姿修正外环"

    $y = 4.2
    Add-Sum $Page 1.35 $y "PP"
    Add-Sum $Page 2.45 $y "PM"
    Add-Box $Page 4.0 $y 1.45 0.9 "位置 P`nKₚₒₛ" $softTeal $ink 10.5 -Bold | Out-Null
    Add-Sum $Page 5.65 $y "PP"
    Add-Box $Page 7.05 $y 1.35 0.9 "腿速限幅" $softGold $ink 9.5 -Bold | Out-Null
    Add-Box $Page 8.95 $y 1.45 0.9 "加速度限制" $softGold $ink 9.5 -Bold | Out-Null
    Add-Sum $Page 10.55 $y "PM"
    Add-Box $Page 12.15 $y 1.5 0.9 "腿速 PIDF`nKᵥ(z)" $softTeal $ink 10.5 -Bold | Out-Null
    Add-Box $Page 14.3 $y 1.55 0.9 "伺服加速度`n限制" $softGold $ink 9.5 -Bold | Out-Null
    Add-Box $Page 17.0 $y 2.1 0.95 "Length-Servo`n理想运动执行器" $softBlue $ink 9.5 -Bold | Out-Null

    Add-Line $Page 0.2 $y 1.17 $y $blue -Arrow | Out-Null
    Add-MathText $Page 0.55 ($y+0.24) 0.8 0.22 "Lᵣₑf" $blue 10 | Out-Null
    Add-Line $Page 1.53 $y 2.27 $y $ink -Arrow | Out-Null
    Add-MathText $Page 1.85 ($y+0.24) 0.8 0.22 "Lᶜᵒʳʳ" $ink 10 | Out-Null
    Add-Line $Page 2.63 $y 3.27 $y $ink -Arrow | Out-Null
    Add-MathText $Page 2.95 ($y+0.24) 0.6 0.22 "eL" $ink 10 | Out-Null
    Add-Line $Page 4.73 $y 5.47 $y $teal -Arrow | Out-Null
    Add-Line $Page 5.83 $y 6.37 $y $ink -Arrow | Out-Null
    Add-Line $Page 7.73 $y 8.23 $y $ink -Arrow | Out-Null
    Add-Line $Page 9.68 $y 10.37 $y $ink -Arrow | Out-Null
    Add-Line $Page 10.73 $y 11.4 $y $ink -Arrow | Out-Null
    Add-Line $Page 12.9 $y 13.53 $y $teal -Arrow | Out-Null
    Add-Line $Page 15.08 $y 15.95 $y $ink -Arrow | Out-Null
    Add-Line $Page 18.05 $y 20.1 $y $ink -Arrow | Out-Null
    Add-MathText $Page 18.9 ($y+0.24) 0.8 0.22 "Lᵃᶜᵗ" $ink 10 | Out-Null
    Add-Line $Page 5.65 5.3 5.65 4.18 $gold -Arrow | Out-Null
    Add-MathText $Page 6.05 5.05 0.9 0.22 "L̇ᵣₑf" $gold 10 | Out-Null

    Add-Path $Page @(@(9.95,7.77),@(9.95,6.25),@(1.35,6.25),@(1.35,4.18)) $teal
    Add-MathText $Page 10.5 6.65 1.1 0.22 "ΔLᵖᵒˢᵉ" $teal 10 | Out-Null
    Add-Dot $Page 18.9 $y
    Add-Path $Page @(@(18.9,$y),@(18.9,2.55),@(2.45,2.55),@(2.45,4.02)) $teal
    Add-Path $Page @(@(17.0,3.72),@(17.0,2.85),@(10.55,2.85),@(10.55,4.02)) $teal
    Add-Path $Page @(@(14.7,$y),@(14.7,5.35),@(12.15,5.35),@(12.15,4.45)) $teal -Dashed
    Add-Text $Page 13.65 5.68 2.2 0.25 "跟踪抗饱和回送" $teal 9.5 -Bold | Out-Null
    Add-Path $Page @(@(17.0,4.48),@(17.0,9.5),@(1.35,9.5),@(1.35,8.43)) $teal
    Add-Group $Page 0.9 3.45 5.95 4.92 "腿长位置环"
    Add-Group $Page 6.25 3.45 9.75 4.92 "腿速参考整形"
    Add-Group $Page 10.1 3.45 15.25 4.92 "腿速内环"
}

try {
    $page1 = $script:Doc.Pages.Item(1)
    Draw-Run02 $page1
    $page2 = $script:Doc.Pages.Add()
    Draw-Run03 $page2
    $page3 = $script:Doc.Pages.Add()
    Draw-Run04 $page3

    $vsdx = Join-Path $OutputDir "stewart_control_block_diagrams.vsdx"
    $script:Doc.SaveAs($vsdx)
    foreach ($page in @($page1, $page2, $page3)) {
        $safeName = $page.Name.Replace(" ", "_")
        $page.Export((Join-Path $OutputDir ($safeName + ".png")))
        $page.Export((Join-Path $OutputDir ($safeName + ".svg")))
    }
    Write-Host "Visio 控制框图已导出：$vsdx"
}
finally {
    if ($script:Doc) { $script:Doc.Close() }
    if ($script:Visio) { $script:Visio.Quit() }
}
