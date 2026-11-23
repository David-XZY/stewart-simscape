Option Explicit
Dim w,d,r,sh
Set w=CreateObject("Word.Application")
w.Visible=True
Set d=w.Documents.Add()
Set r=d.Range(0,0)
r.Text="$F_{\mathrm{nom}}=F_{\mathrm{CT}}+F_{\mathrm{LQI}}$"
r.Select
On Error Resume Next
w.Run "MTCommand_TeXToggle"
WScript.Sleep 3000
WScript.Echo "err=" & Err.Number & " " & Err.Description
WScript.Echo "inline=" & d.InlineShapes.Count & " shapes=" & d.Shapes.Count
If d.InlineShapes.Count>0 Then
  WScript.Echo "type=" & d.InlineShapes(1).Type
  WScript.Echo "progid=" & d.InlineShapes(1).OLEFormat.ProgID
End If
d.SaveAs2 "D:\NPU\Matlab_code\test_mathtype_word.docx"
