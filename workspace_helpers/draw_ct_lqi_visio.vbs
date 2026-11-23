Option Explicit

Const visOpenHidden = 64
Const visOpenRO = 2

Dim outputDirectory, outputVsdx, outputPng1, outputPng2
outputDirectory = "D:\NPU\并联机器人\控制\控制框图"
outputVsdx = outputDirectory & "\计算力矩_LQI控制框图.vsdx"
outputPng1 = outputDirectory & "\计算力矩_LQI总体控制结构.png"
outputPng2 = outputDirectory & "\LQI内部结构与抗饱和回灌.png"

Dim fso
Set fso = CreateObject("Scripting.FileSystemObject")
If Not fso.FolderExists(outputDirectory) Then fso.CreateFolder outputDirectory
If fso.FileExists(outputVsdx) Then fso.DeleteFile outputVsdx, True
If fso.FileExists(outputPng1) Then fso.DeleteFile outputPng1, True
If fso.FileExists(outputPng2) Then fso.DeleteFile outputPng2, True

Function AddBlock(ByVal page, ByVal x, ByVal y, ByVal w, ByVal h, ByVal txt, ByVal fontSize)
  Dim sh
  Set sh = page.DrawRectangle(x-w/2, y-h/2, x+w/2, y+h/2)
  sh.Text = txt
  sh.CellsU("FillForegnd").FormulaU = "RGB(255,255,255)"
  sh.CellsU("LineColor").FormulaU = "RGB(0,0,0)"
  sh.CellsU("LineWeight").FormulaU = "1 pt"
  sh.CellsU("Char.Color").FormulaU = "RGB(0,0,0)"
  sh.CellsU("Char.Size").FormulaU = CStr(fontSize) & " pt"
  sh.CellsU("Char.Font").FormulaU = "FONT(""SimSun"")"
  sh.CellsU("Para.HorzAlign").FormulaU = "1"
  sh.CellsU("VerticalAlign").FormulaU = "1"
  Set AddBlock = sh
End Function

Function AddSum(ByVal page, ByVal x, ByVal y, ByVal txt)
  Dim sh
  Set sh = page.DrawOval(x-0.27, y-0.27, x+0.27, y+0.27)
  sh.Text = txt
  sh.CellsU("FillForegnd").FormulaU = "RGB(255,255,255)"
  sh.CellsU("LineColor").FormulaU = "RGB(0,0,0)"
  sh.CellsU("LineWeight").FormulaU = "1 pt"
  sh.CellsU("Char.Color").FormulaU = "RGB(0,0,0)"
  sh.CellsU("Char.Size").FormulaU = "9 pt"
  sh.CellsU("Char.Font").FormulaU = "FONT(""Times New Roman"")"
  sh.CellsU("Para.HorzAlign").FormulaU = "1"
  sh.CellsU("VerticalAlign").FormulaU = "1"
  Set AddSum = sh
End Function

Function AddText(ByVal page, ByVal x, ByVal y, ByVal w, ByVal h, ByVal txt, ByVal fontSize, ByVal styleCode, ByVal fontName)
  Dim sh
  Set sh = page.DrawRectangle(x-w/2, y-h/2, x+w/2, y+h/2)
  sh.Text = txt
  sh.CellsU("LinePattern").FormulaU = "0"
  sh.CellsU("FillPattern").FormulaU = "0"
  sh.CellsU("Char.Color").FormulaU = "RGB(0,0,0)"
  sh.CellsU("Char.Size").FormulaU = CStr(fontSize) & " pt"
  sh.CellsU("Char.Style").FormulaU = CStr(styleCode)
  sh.CellsU("Char.Font").FormulaU = "FONT(""" & fontName & """)"
  sh.CellsU("Para.HorzAlign").FormulaU = "1"
  sh.CellsU("VerticalAlign").FormulaU = "1"
  Set AddText = sh
End Function

Function AddAnchor(ByVal page, ByVal x, ByVal y)
  Dim sh
  Set sh = page.DrawOval(x-0.015, y-0.015, x+0.015, y+0.015)
  sh.CellsU("LinePattern").FormulaU = "0"
  sh.CellsU("FillPattern").FormulaU = "0"
  Set AddAnchor = sh
End Function

Function AddJunction(ByVal page, ByVal x, ByVal y)
  Dim sh
  Set sh = page.DrawOval(x-0.045, y-0.045, x+0.045, y+0.045)
  sh.CellsU("FillForegnd").FormulaU = "RGB(0,0,0)"
  sh.CellsU("LineColor").FormulaU = "RGB(0,0,0)"
  sh.CellsU("LineWeight").FormulaU = "0.75 pt"
  Set AddJunction = sh
End Function

Function AddBoundary(ByVal page, ByVal x1, ByVal y1, ByVal x2, ByVal y2)
  Dim sh
  Set sh = page.DrawRectangle(x1, y1, x2, y2)
  sh.CellsU("FillPattern").FormulaU = "0"
  sh.CellsU("LineColor").FormulaU = "RGB(0,0,0)"
  sh.CellsU("LineWeight").FormulaU = "1.2 pt"
  sh.CellsU("LinePattern").FormulaU = "2"
  If sh.CellExistsU("Rounding", 0) <> 0 Then sh.CellsU("Rounding").FormulaU = "0.18 in"
  sh.SendToBack
  Set AddBoundary = sh
End Function

Sub StyleConnector(ByVal connector, ByVal dashed, ByVal endArrow)
  connector.CellsU("LineColor").FormulaU = "RGB(0,0,0)"
  connector.CellsU("LineWeight").FormulaU = "1 pt"
  connector.CellsU("EndArrow").FormulaU = CStr(endArrow)
  If dashed Then
    connector.CellsU("LinePattern").FormulaU = "2"
  Else
    connector.CellsU("LinePattern").FormulaU = "1"
  End If
End Sub

Function AddStraight(ByVal page, ByVal master, ByVal fromShape, ByVal toShape, ByVal fromX, ByVal fromY, ByVal toX, ByVal toY, ByVal endArrow, ByVal dashed)
  Dim connector
  Set connector = page.Drop(master, 0, 0)
  connector.CellsU("BeginX").GlueToPos fromShape, fromX, fromY
  connector.CellsU("EndX").GlueToPos toShape, toX, toY
  If connector.CellExistsU("ShapeRouteStyle", 0) <> 0 Then connector.CellsU("ShapeRouteStyle").FormulaU = "2"
  If connector.CellExistsU("ConLineRouteExt", 0) <> 0 Then connector.CellsU("ConLineRouteExt").FormulaU = "1"
  StyleConnector connector, dashed, endArrow
  Set AddStraight = connector
End Function

Function AddOrthogonal(ByVal page, ByVal master, ByVal fromShape, ByVal toShape, ByVal fromX, ByVal fromY, ByVal toX, ByVal toY, ByVal endArrow, ByVal dashed)
  Dim connector
  Set connector = page.Drop(master, 0, 0)
  connector.CellsU("BeginX").GlueToPos fromShape, fromX, fromY
  connector.CellsU("EndX").GlueToPos toShape, toX, toY
  If connector.CellExistsU("ShapeRouteStyle", 0) <> 0 Then connector.CellsU("ShapeRouteStyle").FormulaU = "0"
  If connector.CellExistsU("ConLineRouteExt", 0) <> 0 Then connector.CellsU("ConLineRouteExt").FormulaU = "0"
  StyleConnector connector, dashed, endArrow
  Set AddOrthogonal = connector
End Function

Sub SetPageGeometry(ByVal page, ByVal pageWidth, ByVal pageHeight)
  page.PageSheet.CellsU("PageWidth").FormulaU = CStr(pageWidth) & " in"
  page.PageSheet.CellsU("PageHeight").FormulaU = CStr(pageHeight) & " in"
End Sub

Function ConnectorCount(ByVal page)
  Dim sh, n
  n = 0
  For Each sh In page.Shapes
    If sh.OneD <> 0 Then n = n + 1
  Next
  ConnectorCount = n
End Function

Function GluedConnectorCount(ByVal page)
  Dim sh, n, beginFormula, endFormula
  n = 0
  For Each sh In page.Shapes
    If sh.OneD <> 0 Then
      beginFormula = sh.CellsU("BeginX").FormulaU
      endFormula = sh.CellsU("EndX").FormulaU
      If (InStr(1, beginFormula, "PAR(", 1) > 0 Or InStr(1, beginFormula, "Sheet.", 1) > 0) And _
         (InStr(1, endFormula, "PAR(", 1) > 0 Or InStr(1, endFormula, "Sheet.", 1) > 0) Then
        n = n + 1
      End If
    End If
  Next
  GluedConnectorCount = n
End Function

' Lightweight graph models: node order and coordinates are the drawing source of truth.
Dim overviewNodes, lqiNodes
overviewNodes = Array( _
  Array("trajectory", 1.45, 3.65, 1.55, 0.85), _
  Array("error_sum", 3.00, 2.65, 0.54, 0.54), _
  Array("ct", 4.25, 4.55, 1.70, 0.85), _
  Array("lqi", 4.45, 2.65, 1.80, 0.90), _
  Array("force_sum", 6.25, 3.65, 0.54, 0.54), _
  Array("actuator", 8.15, 3.65, 2.05, 0.95), _
  Array("platform", 10.35, 3.65, 1.55, 0.90))
lqiNodes = Array( _
  Array("observer", 3.05, 4.55, 2.65, 0.95), _
  Array("kx", 5.55, 4.55, 1.00, 0.70), _
  Array("integrator", 3.05, 2.25, 2.65, 1.00), _
  Array("ki", 5.55, 2.25, 1.00, 0.70), _
  Array("sum", 7.25, 3.40, 0.54, 0.54))

Dim visio, shell
On Error Resume Next
Set visio = CreateObject("Visio.Application")
WScript.Sleep 5000
Set shell = CreateObject("WScript.Shell")
shell.AppActivate "Microsoft Visio"
shell.SendKeys "{ESC}"
WScript.Sleep 2000
Err.Clear
visio.Visible = True
If Err.Number <> 0 Then
  WScript.Echo "Visio startup failed: " & Err.Number & " " & Err.Description
  WScript.Quit 2
End If
On Error GoTo 0

Dim stencil, connectorMaster, document, page1, page2
Set stencil = visio.Documents.OpenEx("BASFLO_M.VSSX", visOpenHidden + visOpenRO)
Set connectorMaster = stencil.Masters.ItemU("Dynamic connector")
Set document = visio.Documents.Add("")

' Page 1: computed-torque-LQI overall control structure.
Set page1 = visio.ActivePage
page1.Name = "计算力矩-LQI总体控制结构"
SetPageGeometry page1, 13.2, 6.3

Dim boundary1, input1, trajectory, refJunction, errorSum, ct, lqi, forceSum, actuator, platform, outputJunction, output1
Set boundary1 = AddBoundary(page1, 2.55, 1.65, 6.65, 5.55)
Call AddText(page1, 4.60, 5.74, 2.5, 0.30, "计算力矩–LQI控制器", 10.5, 0, "SimSun")
Set input1 = AddAnchor(page1, 0.20, 3.65)
Set trajectory = AddBlock(page1, 1.45, 3.65, 1.55, 0.85, "参考轨迹" & vbCrLf & "连续重建", 10.5)
Set refJunction = AddJunction(page1, 2.55, 3.65)
Set errorSum = AddSum(page1, 3.00, 2.65, "+" & vbCrLf & "−")
errorSum.Text = ""
Set ct = AddBlock(page1, 4.25, 4.55, 1.70, 0.85, "逆动力学" & vbCrLf & "计算力矩", 10.5)
Set lqi = AddBlock(page1, 4.45, 2.65, 1.80, 0.90, "LQI输出反馈" & vbCrLf & "状态观测/积分校正", 9.5)
Set forceSum = AddSum(page1, 6.25, 3.65, "+" & vbCrLf & "+")
Set actuator = AddBlock(page1, 8.15, 3.65, 2.05, 0.95, "非理想力执行器" & vbCrLf & "延迟/死区/速率/滞后/饱和", 9.2)
Set platform = AddBlock(page1, 10.35, 3.65, 1.55, 0.90, "Stewart" & vbCrLf & "平台", 10.5)
Set outputJunction = AddJunction(page1, 11.45, 3.65)
Set output1 = AddAnchor(page1, 12.85, 3.65)

Dim c, feedbackWp1, feedbackWp2, awWp1, awWp2
Set c = AddStraight(page1, connectorMaster, input1, trajectory, 1, 0.5, 0, 0.5, 4, False)
Set c = AddStraight(page1, connectorMaster, trajectory, refJunction, 1, 0.5, 0, 0.5, 0, False)
Set c = AddOrthogonal(page1, connectorMaster, refJunction, ct, 0.5, 1, 0, 0.5, 4, False)
Set c = AddStraight(page1, connectorMaster, refJunction, errorSum, 0.5, 0, 0, 0.5, 4, False)
Set c = AddStraight(page1, connectorMaster, errorSum, lqi, 1, 0.5, 0, 0.5, 4, False)
Set c = AddOrthogonal(page1, connectorMaster, ct, forceSum, 1, 0.5, 0.5, 1, 4, False)
Set c = AddOrthogonal(page1, connectorMaster, lqi, forceSum, 1, 0.5, 0.5, 0, 4, False)
Set c = AddStraight(page1, connectorMaster, forceSum, actuator, 1, 0.5, 0, 0.5, 4, False)
Set c = AddStraight(page1, connectorMaster, actuator, platform, 1, 0.5, 0, 0.5, 4, False)
Set c = AddStraight(page1, connectorMaster, platform, outputJunction, 1, 0.5, 0, 0.5, 0, False)
Set c = AddStraight(page1, connectorMaster, outputJunction, output1, 1, 0.5, 0, 0.5, 4, False)

Set feedbackWp1 = AddAnchor(page1, 11.45, 0.95)
Set feedbackWp2 = AddAnchor(page1, 3.00, 0.95)
Set c = AddStraight(page1, connectorMaster, outputJunction, feedbackWp1, 0.5, 0, 0.5, 1, 0, False)
Set c = AddStraight(page1, connectorMaster, feedbackWp1, feedbackWp2, 0, 0.5, 1, 0.5, 0, False)
Set c = AddStraight(page1, connectorMaster, feedbackWp2, errorSum, 0.5, 1, 0.5, 0, 4, False)

Set awWp1 = AddAnchor(page1, 8.15, 1.45)
Set awWp2 = AddAnchor(page1, 4.45, 1.45)
Set c = AddStraight(page1, connectorMaster, actuator, awWp1, 0.5, 0, 0.5, 1, 0, True)
Set c = AddStraight(page1, connectorMaster, awWp1, awWp2, 0, 0.5, 1, 0.5, 0, True)
Set c = AddStraight(page1, connectorMaster, awWp2, lqi, 0.5, 1, 0.5, 0, 4, True)

Call AddText(page1, 0.62, 4.02, 1.15, 0.50, "期望轨迹" & vbCrLf & "r(t)", 9.5, 0, "SimSun")
Call AddText(page1, 2.75, 4.02, 1.65, 0.30, "q_r, q̇_r, q̈_r", 10, 2, "Times New Roman")
Call AddText(page1, 5.35, 4.88, 0.75, 0.30, "F_CT", 10, 2, "Times New Roman")
Call AddText(page1, 3.52, 2.95, 0.65, 0.28, "e_q", 10, 2, "Times New Roman")
Call AddText(page1, 5.45, 2.92, 0.85, 0.28, "F_LQI", 10, 2, "Times New Roman")
Call AddText(page1, 7.12, 4.00, 0.85, 0.28, "F_nom", 10, 2, "Times New Roman")
Call AddText(page1, 9.23, 4.00, 0.75, 0.28, "F_act", 10, 2, "Times New Roman")
Call AddText(page1, 12.15, 4.20, 1.55, 0.75, "实际位姿 q(t)" & vbCrLf & "支链状态 L, L̇, F", 9.2, 0, "SimSun")
Call AddText(page1, 7.00, 1.86, 2.10, 0.28, "ΔF = F_act − F_raw", 9.5, 2, "Times New Roman")
Call AddText(page1, 7.20, 0.72, 2.20, 0.28, "实际位姿反馈 q(t)", 9.5, 0, "SimSun")
Call AddText(page1, 2.91, 2.76, 0.18, 0.18, "+", 9, 0, "Times New Roman")
Call AddText(page1, 3.00, 2.52, 0.18, 0.18, "−", 9, 0, "Times New Roman")

' Page 2: LQI internal structure and anti-windup.
Set page2 = document.Pages.Add()
page2.Name = "LQI内部结构与抗饱和回灌"
SetPageGeometry page2, 12.0, 6.2

Dim boundary2, inputEq, eqJunction, observer, kx, integrator, ki, sumLqi, outJunctionLqi, outputLqi, inputDelta
Set boundary2 = AddBoundary(page2, 0.65, 0.65, 11.00, 5.72)
Call AddText(page2, 5.83, 5.88, 1.55, 0.30, "LQI控制器", 10.5, 0, "SimSun")
Set inputEq = AddAnchor(page2, 0.20, 3.40)
Set eqJunction = AddJunction(page2, 1.00, 3.40)
Set observer = AddBlock(page2, 3.05, 4.55, 2.65, 0.95, "状态观测器" & vbCrLf & "d x̂/dt=A x̂+B F_LQI+L_o(e_q−C x̂)", 9.2)
Set kx = AddBlock(page2, 5.55, 4.55, 1.00, 0.70, "−K_x", 11)
Set integrator = AddBlock(page2, 3.05, 2.25, 2.65, 1.00, "积分状态更新" & vbCrLf & "d x_I/dt=−e_q−λx_I+K_awΔF", 9.2)
Set ki = AddBlock(page2, 5.55, 2.25, 1.00, 0.70, "−K_I", 11)
Set sumLqi = AddSum(page2, 7.25, 3.40, "+" & vbCrLf & "+")
Set outJunctionLqi = AddJunction(page2, 8.25, 3.40)
Set outputLqi = AddAnchor(page2, 10.65, 3.40)
Set inputDelta = AddAnchor(page2, 0.20, 1.10)

Set c = AddStraight(page2, connectorMaster, inputEq, eqJunction, 1, 0.5, 0, 0.5, 0, False)
Set c = AddOrthogonal(page2, connectorMaster, eqJunction, observer, 0.5, 1, 0, 0.5, 4, False)
Set c = AddOrthogonal(page2, connectorMaster, eqJunction, integrator, 0.5, 0, 0, 0.5, 4, False)
Set c = AddStraight(page2, connectorMaster, observer, kx, 1, 0.5, 0, 0.5, 4, False)
Set c = AddStraight(page2, connectorMaster, integrator, ki, 1, 0.5, 0, 0.5, 4, False)
Set c = AddOrthogonal(page2, connectorMaster, kx, sumLqi, 1, 0.5, 0.5, 1, 4, False)
Set c = AddOrthogonal(page2, connectorMaster, ki, sumLqi, 1, 0.5, 0.5, 0, 4, False)
Set c = AddStraight(page2, connectorMaster, sumLqi, outJunctionLqi, 1, 0.5, 0, 0.5, 0, False)
Set c = AddStraight(page2, connectorMaster, outJunctionLqi, outputLqi, 1, 0.5, 0, 0.5, 4, False)

Dim deltaWp, modelWp1, modelWp2
Set deltaWp = AddAnchor(page2, 1.10, 1.10)
Set c = AddStraight(page2, connectorMaster, inputDelta, deltaWp, 1, 0.5, 0, 0.5, 0, True)
Set c = AddOrthogonal(page2, connectorMaster, deltaWp, integrator, 0.5, 1, 0, 0.15, 4, True)

Set modelWp1 = AddAnchor(page2, 8.25, 5.25)
Set modelWp2 = AddAnchor(page2, 3.05, 5.25)
Set c = AddStraight(page2, connectorMaster, outJunctionLqi, modelWp1, 0.5, 1, 0.5, 0, 0, False)
Set c = AddStraight(page2, connectorMaster, modelWp1, modelWp2, 0, 0.5, 1, 0.5, 0, False)
Set c = AddStraight(page2, connectorMaster, modelWp2, observer, 0.5, 0, 0.5, 1, 4, False)

Call AddText(page2, 0.58, 3.73, 0.80, 0.30, "e_q", 10.5, 2, "Times New Roman")
Call AddText(page2, 4.45, 4.88, 0.55, 0.28, "x̂", 10.5, 2, "Times New Roman")
Call AddText(page2, 4.45, 2.58, 0.55, 0.28, "x_I", 10.5, 2, "Times New Roman")
Call AddText(page2, 9.25, 3.75, 1.30, 0.30, "F_LQI", 10.5, 2, "Times New Roman")
Call AddText(page2, 0.72, 1.42, 1.25, 0.45, "执行器偏差" & vbCrLf & "ΔF", 9.5, 0, "SimSun")
Call AddText(page2, 5.55, 5.42, 2.55, 0.28, "F_LQI（观测器模型输入）", 9.5, 0, "SimSun")

document.SaveAs outputVsdx
page1.Export outputPng1
page2.Export outputPng2

Dim page1Connectors, page1Glued, page2Connectors, page2Glued
page1Connectors = ConnectorCount(page1)
page1Glued = GluedConnectorCount(page1)
page2Connectors = ConnectorCount(page2)
page2Glued = GluedConnectorCount(page2)

WScript.Echo "VSDX=" & outputVsdx
WScript.Echo "PNG1=" & outputPng1
WScript.Echo "PNG2=" & outputPng2
WScript.Echo "PAGE1 shapes=" & page1.Shapes.Count & " connectors=" & page1Connectors & " glued=" & page1Glued
WScript.Echo "PAGE2 shapes=" & page2.Shapes.Count & " connectors=" & page2Connectors & " glued=" & page2Glued
WScript.Echo "PNG1_EXISTS=" & CStr(fso.FileExists(outputPng1))
WScript.Echo "PNG2_EXISTS=" & CStr(fso.FileExists(outputPng2))
