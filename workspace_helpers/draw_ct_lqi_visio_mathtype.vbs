Option Explicit

Const visOpenHidden = 64
Const visOpenRO = 2

Dim outputDirectory, outputVsdx, outputPng1, outputPng2
Dim outputMathSource
outputDirectory = "D:\NPU\并联机器人\控制\控制框图"
outputVsdx = outputDirectory & "\计算力矩_LQI控制框图.vsdx"
outputPng1 = outputDirectory & "\计算力矩_LQI总体控制结构.png"
outputPng2 = outputDirectory & "\LQI内部结构与抗饱和回灌.png"
outputMathSource = outputDirectory & "\计算力矩_LQI控制框图_MathType公式源.docx"

Dim fso
Set fso = CreateObject("Scripting.FileSystemObject")
If Not fso.FolderExists(outputDirectory) Then fso.CreateFolder outputDirectory
If fso.FileExists(outputVsdx) Then fso.DeleteFile outputVsdx, True
If fso.FileExists(outputPng1) Then fso.DeleteFile outputPng1, True
If fso.FileExists(outputPng2) Then fso.DeleteFile outputPng2, True
If fso.FileExists(outputMathSource) Then fso.DeleteFile outputMathSource, True

Dim visio, word, mathDoc, shell, connectorMaster, mathCount, mathProgId
Dim formulaSpecCount, formulaPage(40), formulaX(40), formulaY(40), formulaWidth(40), formulaTex(40)
mathCount = 0
mathProgId = ""
formulaSpecCount = 0

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

Function AddSum(ByVal page, ByVal x, ByVal y)
  Dim sh
  Set sh = page.DrawOval(x-0.27, y-0.27, x+0.27, y+0.27)
  sh.CellsU("FillForegnd").FormulaU = "RGB(255,255,255)"
  sh.CellsU("LineColor").FormulaU = "RGB(0,0,0)"
  sh.CellsU("LineWeight").FormulaU = "1 pt"
  Set AddSum = sh
End Function

Function AddText(ByVal page, ByVal x, ByVal y, ByVal w, ByVal h, ByVal txt, ByVal fontSize, ByVal fontName)
  Dim sh
  Set sh = page.DrawRectangle(x-w/2, y-h/2, x+w/2, y+h/2)
  sh.Text = txt
  sh.CellsU("LinePattern").FormulaU = "0"
  sh.CellsU("FillPattern").FormulaU = "0"
  sh.CellsU("Char.Color").FormulaU = "RGB(0,0,0)"
  sh.CellsU("Char.Size").FormulaU = CStr(fontSize) & " pt"
  sh.CellsU("Char.Font").FormulaU = "FONT(""" & fontName & """)"
  sh.CellsU("Para.HorzAlign").FormulaU = "1"
  sh.CellsU("VerticalAlign").FormulaU = "1"
  Set AddText = sh
End Function

Function AddMath(ByVal page, ByVal x, ByVal y, ByVal targetWidth, ByVal tex)
  formulaSpecCount = formulaSpecCount + 1
  formulaPage(formulaSpecCount) = page.Index
  formulaX(formulaSpecCount) = x
  formulaY(formulaSpecCount) = y
  formulaWidth(formulaSpecCount) = targetWidth
  formulaTex(formulaSpecCount) = tex
  Set AddMath = Nothing
End Function

Sub PasteMathSpec(ByVal specIndex)
  Dim page, equation, sh, originalWidth, originalHeight, scaledHeight
  Set page = document.Pages.Item(formulaPage(specIndex))
  Set equation = mathDoc.InlineShapes(specIndex)
  mathProgId = equation.OLEFormat.ProgID
  equation.Range.Copy
  page.PasteSpecial 14
  Set sh = page.Shapes.Item(page.Shapes.Count)
  sh.CellsU("PinX").FormulaU = CStr(formulaX(specIndex)) & " in"
  sh.CellsU("PinY").FormulaU = CStr(formulaY(specIndex)) & " in"
  If formulaWidth(specIndex) > 0 Then
    originalWidth = sh.CellsU("Width").ResultIU
    originalHeight = sh.CellsU("Height").ResultIU
    scaledHeight = originalHeight * (0.6 * formulaWidth(specIndex)) / originalWidth
    sh.CellsU("Width").FormulaU = CStr(0.6 * formulaWidth(specIndex)) & " in"
    sh.CellsU("Height").FormulaU = CStr(scaledHeight) & " in"
  End If
  shell.SendKeys "{ESC}"
  WScript.Sleep 100
End Sub

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

Function AddStraight(ByVal page, ByVal fromShape, ByVal toShape, ByVal fromX, ByVal fromY, ByVal toX, ByVal toY, ByVal endArrow, ByVal dashed)
  Dim connector
  Set connector = page.Drop(connectorMaster, 0, 0)
  connector.CellsU("BeginX").GlueToPos fromShape, fromX, fromY
  connector.CellsU("EndX").GlueToPos toShape, toX, toY
  If connector.CellExistsU("ShapeRouteStyle", 0) <> 0 Then connector.CellsU("ShapeRouteStyle").FormulaU = "2"
  If connector.CellExistsU("ConLineRouteExt", 0) <> 0 Then connector.CellsU("ConLineRouteExt").FormulaU = "1"
  StyleConnector connector, dashed, endArrow
  Set AddStraight = connector
End Function

Function AddOrthogonal(ByVal page, ByVal fromShape, ByVal toShape, ByVal fromX, ByVal fromY, ByVal toX, ByVal toY, ByVal endArrow, ByVal dashed)
  Dim connector
  Set connector = page.Drop(connectorMaster, 0, 0)
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
  Dim sh, n, bf, ef
  n = 0
  For Each sh In page.Shapes
    If sh.OneD <> 0 Then
      bf = sh.CellsU("BeginX").FormulaU
      ef = sh.CellsU("EndX").FormulaU
      If (InStr(1,bf,"PAR(",1)>0 Or InStr(1,bf,"Sheet.",1)>0) And _
         (InStr(1,ef,"PAR(",1)>0 Or InStr(1,ef,"Sheet.",1)>0) Then n=n+1
    End If
  Next
  GluedConnectorCount = n
End Function

Function MathObjectCount(ByVal page)
  Dim sh, n
  n = 0
  For Each sh In page.Shapes
    If sh.Type = 4 Then n = n + 1
  Next
  MathObjectCount = n
End Function

' Generate every MathType object before starting Visio. This avoids focus changes
' between Word's TeX conversion macro and embedded Visio OLE objects.
Dim mathTex, mathText, i, r
mathTex = Array( _
  "r(t)", _
  "\mathbf{q}_r,\ \dot{\mathbf{q}}_r,\ \ddot{\mathbf{q}}_r", _
  "\mathbf{F}_{\mathrm{CT}}", _
  "\mathbf{e}_q=\mathbf{q}_r-\mathbf{q}", _
  "\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}}", _
  "\mathbf{F}_{\mathrm{LQI}}", _
  "\mathbf{F}_{\mathrm{nom}}", _
  "\mathbf{F}_{\mathrm{act}}", _
  "\Delta\mathbf{F}_{\mathrm{aw}}", _
  "\mathbf{F}_{\mathrm{nom}}=\mathbf{F}_{\mathrm{CT}}+\mathbf{F}_{\mathrm{LQI}}", _
  "\mathbf{q}(t)", _
  "\mathbf{L},\ \dot{\mathbf{L}},\ \mathbf{F}_{\mathrm{act}}", _
  "\mathbf{q}(t)", _
  "\mathbf{e}_q", _
  "\dot{\hat{\mathbf{x}}}=(\mathbf{A}+\mathbf{L}_o\mathbf{C})\hat{\mathbf{x}}+\mathbf{B}\mathbf{F}_{\mathrm{LQI}}+\mathbf{L}_o\mathbf{e}_q", _
  "\hat{\mathbf{x}}", _
  "-\mathbf{K}_x", _
  "\dot{\mathbf{x}}_I=-\mathbf{e}_q-\lambda\mathbf{x}_I+\mathbf{K}_{\mathrm{aw}}\Delta\mathbf{F}_{\mathrm{aw}}", _
  "\mathbf{x}_I", _
  "-\mathbf{K}_I", _
  "\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}}", _
  "\mathbf{F}_{\mathrm{LQI}}=\mathrm{sat}(\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}})", _
  "\mathbf{F}_{\mathrm{LQI}}", _
  "\Delta\mathbf{F}_{\mathrm{aw}}=\mathbf{F}_{\mathrm{LQI}}-\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}}", _
  "\Delta\mathbf{F}_{\mathrm{aw}}", _
  "\mathbf{F}_{\mathrm{LQI}}\ \mathrm{(observer\ input)}")

Set word = CreateObject("Word.Application")
word.Visible = True
Set mathDoc = word.Documents.Add()
mathText = ""
For i = 0 To UBound(mathTex)
  mathText = mathText & "$" & mathTex(i) & "$"
  If i < UBound(mathTex) Then mathText = mathText & vbCr
Next
mathDoc.Content.Text = mathText
word.Activate
For i = UBound(mathTex)+1 To 1 Step -1
  Set r = mathDoc.Paragraphs(i).Range
  r.End = r.End - 1
  r.Select
  word.Run "MTCommand_TeXToggle"
Next
mathCount = mathDoc.InlineShapes.Count
mathDoc.SaveAs2 outputMathSource
word.Visible = False

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

Dim stencil, document, page1, page2, c
Set stencil = visio.Documents.OpenEx("BASFLO_M.VSSX", visOpenHidden + visOpenRO)
Set connectorMaster = stencil.Masters.ItemU("Dynamic connector")
Set document = visio.Documents.Add("")

' PAGE 1: overall computed-torque-LQI structure.
Set page1 = visio.ActivePage
page1.Name = "计算力矩-LQI总体控制结构"
SetPageGeometry page1, 13.7, 6.4

Dim boundary1, input1, trajectory, refJunction, errorSum, ct, lqi, limiter, forceSum, actuator, platform, outputJunction, output1
Set boundary1 = AddBoundary(page1, 2.50, 1.55, 7.18, 5.62)
Call AddText(page1, 4.84, 5.82, 2.50, 0.30, "计算力矩–LQI控制器", 10.5, "SimSun")
Set input1 = AddAnchor(page1, 0.18, 3.65)
Set trajectory = AddBlock(page1, 1.38, 3.65, 1.52, 0.86, "参考轨迹" & vbCrLf & "连续重建", 10.5)
Set refJunction = AddJunction(page1, 2.46, 3.65)
Set errorSum = AddSum(page1, 2.95, 2.40)
Set ct = AddBlock(page1, 4.10, 4.62, 1.68, 0.86, "逆动力学" & vbCrLf & "计算力矩", 10.5)
Set lqi = AddBlock(page1, 3.95, 2.40, 1.55, 0.88, "LQI输出反馈", 10.2)
Set limiter = AddBlock(page1, 5.55, 2.40, 1.42, 0.88, "反馈力限幅" & vbCrLf & "与回灌差值", 9.4)
Set forceSum = AddSum(page1, 6.78, 3.65)
Set actuator = AddBlock(page1, 8.70, 3.65, 2.05, 0.95, "非理想力执行器" & vbCrLf & "延迟/死区/速率/滞后/饱和", 9.2)
Set platform = AddBlock(page1, 10.95, 3.65, 1.55, 0.90, "Stewart" & vbCrLf & "平台", 10.5)
Set outputJunction = AddJunction(page1, 12.05, 3.65)
Set output1 = AddAnchor(page1, 13.48, 3.65)

Set c = AddStraight(page1,input1,trajectory,1,0.5,0,0.5,4,False)
Set c = AddStraight(page1,trajectory,refJunction,1,0.5,0,0.5,0,False)
Set c = AddOrthogonal(page1,refJunction,ct,0.5,1,0,0.5,4,False)
Set c = AddStraight(page1,refJunction,errorSum,0.5,0,0,0.5,4,False)
Set c = AddStraight(page1,errorSum,lqi,1,0.5,0,0.5,4,False)
Set c = AddStraight(page1,lqi,limiter,1,0.5,0,0.5,4,False)
Set c = AddOrthogonal(page1,ct,forceSum,1,0.5,0.5,1,4,False)
Set c = AddOrthogonal(page1,limiter,forceSum,1,0.5,0.5,0,4,False)
Set c = AddStraight(page1,forceSum,actuator,1,0.5,0,0.5,4,False)
Set c = AddStraight(page1,actuator,platform,1,0.5,0,0.5,4,False)
Set c = AddStraight(page1,platform,outputJunction,1,0.5,0,0.5,0,False)
Set c = AddStraight(page1,outputJunction,output1,1,0.5,0,0.5,4,False)

Dim fwp1, fwp2, awp1, awp2
Set fwp1 = AddAnchor(page1,12.05,0.88)
Set fwp2 = AddAnchor(page1,2.95,0.88)
Set c = AddStraight(page1,outputJunction,fwp1,0.5,0,0.5,1,0,False)
Set c = AddStraight(page1,fwp1,fwp2,0,0.5,1,0.5,0,False)
Set c = AddStraight(page1,fwp2,errorSum,0.5,1,0.5,0,4,False)

Set awp1 = AddAnchor(page1,5.55,1.38)
Set awp2 = AddAnchor(page1,3.95,1.38)
Set c = AddStraight(page1,limiter,awp1,0.5,0,0.5,1,0,True)
Set c = AddStraight(page1,awp1,awp2,0,0.5,1,0.5,0,True)
Set c = AddStraight(page1,awp2,lqi,0.5,1,0.5,0,4,True)

' Sum signs are operators, while all variable expressions are MathType objects.
Call AddText(page1,2.86,2.51,0.16,0.16,"+",9,"Times New Roman")
Call AddText(page1,2.95,2.27,0.16,0.16,"−",9,"Times New Roman")
Call AddText(page1,6.78,3.76,0.16,0.16,"+",9,"Times New Roman")
Call AddText(page1,6.78,3.53,0.16,0.16,"+",9,"Times New Roman")
Call AddText(page1,0.58,4.02,1.08,0.30,"期望轨迹",9.5,"SimSun")
Call AddMath(page1,0.58,3.82,0.48,"r(t)")
Call AddMath(page1,2.77,4.03,1.20,"\mathbf{q}_r,\ \dot{\mathbf{q}}_r,\ \ddot{\mathbf{q}}_r")
Call AddMath(page1,5.10,4.94,0.50,"\mathbf{F}_{\mathrm{CT}}")
Call AddMath(page1,3.38,2.90,0.85,"\mathbf{e}_q=\mathbf{q}_r-\mathbf{q}")
Call AddMath(page1,4.80,2.76,0.80,"\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}}")
Call AddMath(page1,6.12,2.76,0.65,"\mathbf{F}_{\mathrm{LQI}}")
Call AddMath(page1,7.65,4.00,0.65,"\mathbf{F}_{\mathrm{nom}}")
Call AddMath(page1,9.82,4.00,0.58,"\mathbf{F}_{\mathrm{act}}")
Call AddMath(page1,4.75,1.14,0.70,"\Delta\mathbf{F}_{\mathrm{aw}}")
Call AddMath(page1,8.15,5.28,2.00,"\mathbf{F}_{\mathrm{nom}}=\mathbf{F}_{\mathrm{CT}}+\mathbf{F}_{\mathrm{LQI}}")
Call AddText(page1,12.78,4.26,1.32,0.28,"实际位姿",9.2,"SimSun")
Call AddMath(page1,12.78,4.04,0.48,"\mathbf{q}(t)")
Call AddText(page1,12.78,3.83,1.32,0.28,"支链状态",9.2,"SimSun")
Call AddMath(page1,12.78,3.60,0.92,"\mathbf{L},\ \dot{\mathbf{L}},\ \mathbf{F}_{\mathrm{act}}")
Call AddText(page1,7.40,0.66,1.55,0.28,"实际位姿反馈",9.2,"SimSun")
Call AddMath(page1,8.23,0.66,0.46,"\mathbf{q}(t)")

' PAGE 2: internal LQI, limiter, and exact anti-windup difference.
Set page2 = document.Pages.Add()
page2.Name = "LQI内部结构与抗饱和回灌"
SetPageGeometry page2, 13.2, 6.5

Dim boundary2, inEq, eqJ, observer, kx, integrator, ki, sumRaw, rawJ, limiter2, satJ, out2, difference
Set boundary2 = AddBoundary(page2,0.55,0.55,12.65,5.95)
Call AddText(page2,6.60,6.12,1.55,0.28,"LQI控制器",10.5,"SimSun")
Set inEq = AddAnchor(page2,0.18,3.55)
Set eqJ = AddJunction(page2,0.95,3.55)
Set observer = AddBlock(page2,2.85,4.72,2.85,1.25,"状态观测器",10.2)
Set kx = AddBlock(page2,5.20,4.72,0.95,0.70,"",10)
Set integrator = AddBlock(page2,2.85,2.25,2.85,1.25,"积分状态更新",10.2)
Set ki = AddBlock(page2,5.20,2.25,0.95,0.70,"",10)
Set sumRaw = AddSum(page2,6.60,3.55)
Set rawJ = AddJunction(page2,7.25,3.55)
Set limiter2 = AddBlock(page2,8.45,3.55,1.55,0.88,"反馈力限幅",10.2)
Set satJ = AddJunction(page2,9.55,3.55)
Set out2 = AddAnchor(page2,12.35,3.55)
Set difference = AddBlock(page2,8.45,1.72,2.35,0.95,"抗饱和差值",10.0)

Set c = AddStraight(page2,inEq,eqJ,1,0.5,0,0.5,0,False)
Set c = AddOrthogonal(page2,eqJ,observer,0.5,1,0,0.5,4,False)
Set c = AddOrthogonal(page2,eqJ,integrator,0.5,0,0,0.5,4,False)
Set c = AddStraight(page2,observer,kx,1,0.5,0,0.5,4,False)
Set c = AddStraight(page2,integrator,ki,1,0.5,0,0.5,4,False)
Set c = AddOrthogonal(page2,kx,sumRaw,1,0.5,0.5,1,4,False)
Set c = AddOrthogonal(page2,ki,sumRaw,1,0.5,0.5,0,4,False)
Set c = AddStraight(page2,sumRaw,rawJ,1,0.5,0,0.5,0,False)
Set c = AddStraight(page2,rawJ,limiter2,1,0.5,0,0.5,4,False)
Set c = AddStraight(page2,limiter2,satJ,1,0.5,0,0.5,0,False)
Set c = AddStraight(page2,satJ,out2,1,0.5,0,0.5,4,False)

' The saturated LQI force enters the observer model. In the implemented
' realization F_LQI_raw + DeltaF_aw = F_LQI.
Dim obWp1, obWp2
Set obWp1 = AddAnchor(page2,9.55,5.55)
Set obWp2 = AddAnchor(page2,2.85,5.55)
Set c = AddStraight(page2,satJ,obWp1,0.5,1,0.5,0,0,False)
Set c = AddStraight(page2,obWp1,obWp2,0,0.5,1,0.5,0,False)
Set c = AddStraight(page2,obWp2,observer,0.5,0,0.5,1,4,False)

' Difference block receives saturated (+) and raw (-) feedback force.
Set c = AddOrthogonal(page2,satJ,difference,0.5,0,1,0.67,4,False)
Set c = AddOrthogonal(page2,rawJ,difference,0.5,0,0,0.33,4,False)
Dim diffWp1, diffWp2
Set diffWp1 = AddAnchor(page2,6.60,0.88)
Set diffWp2 = AddAnchor(page2,2.85,0.88)
Set c = AddOrthogonal(page2,difference,diffWp1,0,0.5,1,0.5,0,True)
Set c = AddStraight(page2,diffWp1,diffWp2,0,0.5,1,0.5,0,True)
Set c = AddStraight(page2,diffWp2,integrator,0.5,1,0.5,0,4,True)

Call AddText(page2,6.60,3.66,0.16,0.16,"+",9,"Times New Roman")
Call AddText(page2,6.60,3.42,0.16,0.16,"+",9,"Times New Roman")
Call AddMath(page2,0.54,3.84,0.38,"\mathbf{e}_q")
Call AddMath(page2,2.85,4.54,3.20,"\dot{\hat{\mathbf{x}}}=(\mathbf{A}+\mathbf{L}_o\mathbf{C})\hat{\mathbf{x}}+\mathbf{B}\mathbf{F}_{\mathrm{LQI}}+\mathbf{L}_o\mathbf{e}_q")
Call AddMath(page2,4.32,5.08,0.38,"\hat{\mathbf{x}}")
Call AddMath(page2,5.20,4.72,0.52,"-\mathbf{K}_x")
Call AddMath(page2,2.85,2.05,3.05,"\dot{\mathbf{x}}_I=-\mathbf{e}_q-\lambda\mathbf{x}_I+\mathbf{K}_{\mathrm{aw}}\Delta\mathbf{F}_{\mathrm{aw}}")
Call AddMath(page2,4.32,2.60,0.38,"\mathbf{x}_I")
Call AddMath(page2,5.20,2.25,0.52,"-\mathbf{K}_I")
Call AddMath(page2,7.70,3.88,0.95,"\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}}")
Call AddMath(page2,8.45,3.36,1.65,"\mathbf{F}_{\mathrm{LQI}}=\mathrm{sat}(\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}})")
Call AddMath(page2,10.85,3.88,0.82,"\mathbf{F}_{\mathrm{LQI}}")
Call AddMath(page2,8.45,1.52,2.40,"\Delta\mathbf{F}_{\mathrm{aw}}=\mathbf{F}_{\mathrm{LQI}}-\mathbf{F}_{\mathrm{LQI}}^{\mathrm{raw}}")
Call AddMath(page2,4.65,1.05,0.82,"\Delta\mathbf{F}_{\mathrm{aw}}")
Call AddMath(page2,6.20,5.74,1.45,"\mathbf{F}_{\mathrm{LQI}}\ \mathrm{(observer\ input)}")

' Every formula is rendered by MathType and pasted into Visio as an enhanced
' metafile for stable page rendering. The companion DOCX retains the actual
' editable Equation.DSMT4 objects in the same order.
Dim specIndex
For specIndex = 1 To formulaSpecCount
  PasteMathSpec specIndex
Next

document.SaveAs outputVsdx
page1.Export outputPng1
page2.Export outputPng2
document.Save

Dim p1c,p1g,p1m,p2c,p2g,p2m
p1c=ConnectorCount(page1): p1g=GluedConnectorCount(page1): p1m=MathObjectCount(page1)
p2c=ConnectorCount(page2): p2g=GluedConnectorCount(page2): p2m=MathObjectCount(page2)

WScript.Echo "VSDX=" & outputVsdx
WScript.Echo "PAGE1 connectors=" & p1c & " glued=" & p1g & " MathTypeFormulaGraphics=" & p1m
WScript.Echo "PAGE2 connectors=" & p2c & " glued=" & p2g & " MathTypeFormulaGraphics=" & p2m
WScript.Echo "MATH_SOURCE_OBJECTS=" & mathCount
WScript.Echo "MATH_PROGID=" & mathProgId
WScript.Echo "MATH_SOURCE=" & outputMathSource
WScript.Echo "PNG1_EXISTS=" & CStr(fso.FileExists(outputPng1))
WScript.Echo "PNG2_EXISTS=" & CStr(fso.FileExists(outputPng2))

mathDoc.Close False
word.Quit
