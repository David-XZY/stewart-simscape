from pathlib import Path
import json
import subprocess

from docx import Document
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


ROOT = Path(r"D:\NPU\Matlab_code")
OUTPUT = ROOT / "output" / "documents" / "第四章_PID控制_公式内容与结论修订版.docx"
EQ_JSON = ROOT / "tmp" / "docx_build" / "pid_content_equations.json"
FINALIZER = ROOT / "tmp" / "docx_build" / "pid_content_finalize.ps1"

EQUATIONS = {
    "E1": {"linear": "𝐞(t)=𝐥_d(t)-𝐥(t)"},
    "E2": {"linear": "𝐟_c(t)=𝐊_p 𝐞(t)+𝐊_i ∫_0^t 𝐞(τ)dτ+𝐊_d (d𝐞(t))/(dt)"},
    "E3": {"linear": "𝐟(t)=𝐟_r(t)+𝐟_c(t)"},
}


def set_font(run, east="宋体", latin="Times New Roman", size=12,
             bold=None, color="000000"):
    run.font.name = latin
    rpr = run._element.get_or_add_rPr()
    rpr.rFonts.set(qn("w:ascii"), latin)
    rpr.rFonts.set(qn("w:hAnsi"), latin)
    rpr.rFonts.set(qn("w:eastAsia"), east)
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    run.font.color.rgb = RGBColor.from_string(color)


def set_style(style, east, latin, size, bold=False):
    style.font.name = latin
    rpr = style._element.get_or_add_rPr()
    rpr.rFonts.set(qn("w:ascii"), latin)
    rpr.rFonts.set(qn("w:hAnsi"), latin)
    rpr.rFonts.set(qn("w:eastAsia"), east)
    style.font.size = Pt(size)
    style.font.bold = bold


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = " PAGE "
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, end])
    set_font(run, size=9)


def add_body(doc, text):
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p.paragraph_format.first_line_indent = Pt(24)
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(5)
    p.paragraph_format.line_spacing = 1.35
    set_font(p.add_run(text))
    return p


def add_equation(doc, eq_id, number):
    p = doc.add_paragraph(style="Equation")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.tab_stops.add_tab_stop(Cm(14.8), WD_TAB_ALIGNMENT.RIGHT)
    p.add_run(f"[[EQ:{eq_id}]]")
    r = p.add_run(f"\t({number})")
    set_font(r, size=10.5)


def configure(doc):
    normal = doc.styles["Normal"]
    set_style(normal, "宋体", "Times New Roman", 12)

    title = doc.styles["Title"]
    set_style(title, "黑体", "Times New Roman", 18, True)
    title.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.space_before = Pt(8)
    title.paragraph_format.space_after = Pt(8)

    h2 = doc.styles["Heading 2"]
    set_style(h2, "黑体", "Times New Roman", 14, True)
    h2.paragraph_format.space_before = Pt(7)
    h2.paragraph_format.space_after = Pt(5)

    if "Equation" not in [s.name for s in doc.styles]:
        eq = doc.styles.add_style("Equation", WD_STYLE_TYPE.PARAGRAPH)
    else:
        eq = doc.styles["Equation"]
    set_style(eq, "宋体", "Cambria Math", 12)


def build_docx():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc = Document()
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21), Cm(29.7)
    sec.top_margin, sec.bottom_margin = Cm(2.1), Cm(2.1)
    sec.left_margin, sec.right_margin = Cm(3.0), Cm(2.5)
    sec.header_distance, sec.footer_distance = Cm(1.2), Cm(1.2)
    configure(doc)

    hp = sec.header.paragraphs[0]
    hp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_font(hp.add_run("面向载荷挂装的并联机器人运动规划与控制方法研究"), size=9, color="777777")
    add_page_number(sec.footer.paragraphs[0])

    doc.core_properties.title = "第四章PID控制公式内容与结论修订版"
    doc.core_properties.author = "谢振宇"

    title = doc.add_paragraph("第4章 基于Simscape的并联机器人轨迹跟踪控制研究", style="Title")
    title._p.get_or_add_pPr().append(OxmlElement("w:keepNext"))

    add_body(
        doc,
        "本章基于上一章IHSID方法获得的参考轨迹和支腿驱动力，建立Stewart并联机器人Simscape多体动力学模型，并采用驱动力前馈与腿长误差反馈相结合的控制结构，验证规划轨迹的跟踪性能。"
    )

    h2 = doc.add_paragraph("4.1 Stewart并联机器人PID控制", style="Heading 2")
    h2._p.get_or_add_pPr().append(OxmlElement("w:keepNext"))

    add_body(
        doc,
        "本文采用关节空间控制方法，将动平台位姿跟踪转化为六条支腿的长度跟踪。参考位姿经逆运动学求解后得到支腿参考长度。为避免逐条支腿书写造成下标冗长，以下统一采用六维向量描述参考腿长、实际腿长及其误差："
    )
    add_equation(doc, "E1", "4.1")

    add_body(
        doc,
        "各支腿仍由相互独立的PIDF通道进行调节。设反馈修正力为六维向量，比例、积分和微分增益为由六组控制器参数构成的对角矩阵，则基本控制律为："
    )
    add_equation(doc, "E2", "4.2")

    add_body(
        doc,
        "式中，比例、积分和微分项分别用于提高响应速度、减小稳态误差和改善动态响应。实际仿真采用带微分滤波的PIDF实现，以限制高频噪声放大；在初始工作点对Simscape模型进行线性化并自动整定六组控制器，目标闭环带宽设为10 Hz。"
    )

    add_body(
        doc,
        "为利用轨迹优化所得的动力学信息，将IHSID计算得到的参考驱动力作为前馈项，则六条支腿的总驱动力为："
    )
    add_equation(doc, "E3", "4.3")

    add_body(
        doc,
        "其中，参考驱动力主要承担平台重力和运动载荷的前馈补偿，PID反馈修正力用于抑制腿长跟踪偏差。在理想模型条件下，对完整7.5 s挂装轨迹进行闭环仿真。最大平移跟踪误差为3.375 mm，最大姿态跟踪误差为0.203°，最大腿长跟踪误差为1.152 mm；支腿长度范围为1.078～1.905 m，最大总驱动力约为1105 N，未超过±2000 N的限制。"
    )

    add_body(
        doc,
        "上述结果仅说明该控制结构在模型准确、前馈驱动力已知且无外部扰动的理想条件下能够完成轨迹跟踪，可作为后续方法的性能基准。独立PID主要依据局部腿长误差调节，未显式利用Stewart机构的非线性动力学和通道耦合信息；固定增益又由初始工作点整定，前馈补偿也依赖模型准确性，因而在载荷变化、参数偏差或外部扰动下性能可能下降。为此，后续将引入计算力矩—LQI控制：利用动力学模型补偿惯性、科氏与离心及重力等非线性项以减弱耦合，再通过LQI的状态反馈和误差积分改善动态响应与稳态精度，从而提高系统对工况变化和扰动的适应能力。"
    )

    doc.save(OUTPUT)


def write_finalizer():
    script = r'''param(
    [Parameter(Mandatory=$true)][string]$DocxPath,
    [Parameter(Mandatory=$true)][string]$EquationJson
)
$ErrorActionPreference = "Stop"
$equations = Get-Content -Raw -Encoding UTF8 -LiteralPath $EquationJson | ConvertFrom-Json
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$doc = $null
$equationDoc = $null
try {
    $doc = $word.Documents.Open($DocxPath, $false, $false)
    $equationDoc = $word.Documents.Add()
    $properties = @($equations.PSObject.Properties)
    for ($propertyIndex = $properties.Count - 1; $propertyIndex -ge 0; $propertyIndex--) {
        $property = $properties[$propertyIndex]
        $id = [string]$property.Name
        $marker = "[[EQ:$id]]"
        $equationRange = $doc.Content.Duplicate
        $find = $equationRange.Find
        $find.Text = $marker
        $find.Forward = $true
        $find.Wrap = 0
        $find.MatchCase = $true
        if (-not $find.Execute()) { throw "Equation marker not found: $id" }
        $equationDoc.Content.Text = ""
        $sourceRange = $equationDoc.Paragraphs.Item(1).Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End-- }
        $sourceRange.Text = [string]$property.Value.linear
        $sourceRange = $equationDoc.Paragraphs.Item(1).Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End-- }
        $sourceRange.Font.Name = "Cambria Math"
        $sourceRange.Font.Size = 12
        $addedRange = $equationDoc.OMaths.Add($sourceRange)
        $addedRange.OMaths.Item(1).BuildUp()
        $sourceRange = $equationDoc.Paragraphs.Item(1).Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End-- }
        $equationRange.FormattedText = $sourceRange.FormattedText
    }
    $equationDoc.Close($false)
    $equationDoc = $null
    [void]$doc.Fields.Update()
    $doc.Repaginate()
    $doc.Save()
    Write-Output ("OMaths=" + $doc.OMaths.Count)
    Write-Output ("Pages=" + $doc.ComputeStatistics(2))
    $doc.Close($false)
    $doc = $null
}
finally {
    if ($null -ne $equationDoc) { try { $equationDoc.Close($false) } catch {} }
    if ($null -ne $doc) { try { $doc.Close($false) } catch {} }
    $word.Quit()
}
'''
    FINALIZER.write_text(script, encoding="utf-8-sig")


if __name__ == "__main__":
    build_docx()
    EQ_JSON.write_text(json.dumps(EQUATIONS, ensure_ascii=False, indent=2), encoding="utf-8-sig")
    write_finalizer()
    subprocess.run([
        "powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", str(FINALIZER), "-DocxPath", str(OUTPUT),
        "-EquationJson", str(EQ_JSON)
    ], check=True)
    print(OUTPUT)
