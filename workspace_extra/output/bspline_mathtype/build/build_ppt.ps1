$ErrorActionPreference='Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type @"
using System;using System.Runtime.InteropServices;public class MTMsg{[DllImport("user32.dll")]public static extern IntPtr SendMessage(IntPtr h,int m,IntPtr w,IntPtr l);}
"@
$hw=(Get-Process MathType).MainWindowHandle
function Cmd([int]$id){[void][MTMsg]::SendMessage($hw,273,[IntPtr]$id,[IntPtr]::Zero)}
$rows=@(
's=t/T,\qquad p=5,\qquad n=16,',
'U=[0^{\times6},1/11,\ldots,10/11,1^{\times6}],',
'P_0=P_1=P_2=q_0,\qquad P_{13}=P_{14}=P_{15}=q_f,',
'z=[P_3^{\top},\ldots,P_{12}^{\top}]^{\top}\in\mathbb{R}^{60},',
'q(t)=\sum_{i=0}^{15}N_{i,5}(t/T)P_i.'
)
$ppt=New-Object -ComObject PowerPoint.Application
$pres=$ppt.Presentations.Add(0)
$pres.PageSetup.SlideWidth=960
$pres.PageSetup.SlideHeight=540
$slide=$pres.Slides.Add(1,12)
$shapes=@()
for($i=0;$i -lt $rows.Count;$i++){
 Cmd 513
 Cmd 506
 [Windows.Forms.Clipboard]::SetText(('\['+$rows[$i]+'\]'))
 Cmd 505
 Cmd 513
 Cmd 504
 $data=[Windows.Forms.Clipboard]::GetDataObject()
 $math=$data.GetData('MathML Presentation')
 if($math -is [string]){Set-Content ('D:/NPU/Matlab_code/output/bspline_mathtype/build/row'+$i+'.mml') $math}
 $range=$slide.Shapes.PasteSpecial(10)
 $shape=$range.Item(1)
 $shape.Name='MathType_Formula_'+($i+1)
 $shape.AlternativeText=$rows[$i]
 Write-Output ('ROW '+$i+' '+$shape.OLEFormat.ProgID+' '+$shape.Width+'x'+$shape.Height)
 $shapes+=,$shape
}
$scale=680.0/$shapes[2].Width
$totalH=0
foreach($s in $shapes){$s.LockAspectRatio=-1;$s.Width=$s.Width*$scale;$totalH+=$s.Height}
$gap=24
$y=(540-$totalH-4*$gap)/2
foreach($s in $shapes){$s.Left=(960-$s.Width)/2;$s.Top=$y;$y+=$s.Height+$gap}
$out='D:\NPU\Matlab_code\output\bspline_mathtype\BSpline_MathType.pptx'
$pres.SaveAs($out,24)
$slide.Export('D:\NPU\Matlab_code\output\bspline_mathtype\preview.png','PNG',1600,900)
$pres.Close()
$check=$ppt.Presentations.Open($out, -1,0,0)
foreach($s in $check.Slides.Item(1).Shapes){Write-Output ('REOPEN '+$s.Name+' '+$s.OLEFormat.ProgID)}
$check.Close()
Write-Output $out

