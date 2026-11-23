Option Explicit
Dim w,d,r,eq,v,vd,p,s
Set w=CreateObject("Word.Application")
w.Visible=False
Set d=w.Documents.Add()
Set r=d.Range(0,0)
r.Text="$F_{\mathrm{nom}}=F_{\mathrm{CT}}+F_{\mathrm{LQI}}$"
r.Select
w.Run "MTCommand_TeXToggle"
Set eq=d.InlineShapes(1)
eq.Range.Copy
Set v=CreateObject("Visio.Application")
WScript.Sleep 5000
Set s=CreateObject("WScript.Shell")
s.AppActivate "Microsoft Visio"
s.SendKeys "{ESC}"
WScript.Sleep 2000
v.Visible=True
Set vd=v.Documents.Add("")
Set p=v.ActivePage
p.Paste
Set eq=v.ActiveWindow.Selection.PrimaryItem
eq.CellsU("PinX").FormulaU="3 in"
eq.CellsU("PinY").FormulaU="2 in"
eq.CellsU("Width").FormulaU="3 in"
WScript.Echo "foreign=" & eq.ForeignType & " type=" & eq.Type & " width=" & eq.CellsU("Width").ResultIU & " height=" & eq.CellsU("Height").ResultIU
vd.SaveAs "D:\NPU\Matlab_code\test_mathtype_visio.vsdx"
p.Export "D:\NPU\Matlab_code\test_mathtype_visio.png"
d.Close False
w.Quit
