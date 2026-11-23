from pathlib import Path

from docx import Document
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


OUTPUT = Path(r"D:\NPU\Matlab_code\output\documents\第四章_PID控制_精简公式兼容版.docx")


def set_run_font(run, east="宋体", latin="Times New Roman", size=12,
                 bold=None, italic=None, color="000000"):
    run.font.name = latin
    rpr = run._element.get_or_add_rPr()
    rpr.rFonts.set(qn("w:ascii"), latin)
    rpr.rFonts.set(qn("w:hAnsi"), latin)
    rpr.rFonts.set(qn("w:eastAsia"), east)
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    run.font.color.rgb = RGBColor.from_string(color)


def set_style(style, east, latin, size, bold=False, color="000000"):
    style.font.name = latin
    rpr = style._element.get_or_add_rPr()
    rpr.rFonts.set(qn("w:ascii"), latin)
    rpr.rFonts.set(qn("w:hAnsi"), latin)
    rpr.rFonts.set(qn("w:eastAsia"), east)
    style.font.size = Pt(size)
    style.font.bold = bold
    style.font.color.rgb = RGBColor.from_string(color)


def add_page_field(paragraph):
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
    set_run_font(run, size=9)


def keep_next(paragraph):
    paragraph._p.get_or_add_pPr().append(OxmlElement("w:keepNext"))


def body(doc, text):
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p.paragraph_format.first_line_indent = Pt(24)
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(5)
    p.paragraph_format.line_spacing = 1.35
    set_run_font(p.add_run(text))
    return p


def equation_marker(doc, unicode_math, number, tex):
    p = doc.add_paragraph(style="Equation Marker")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.tab_stops.add_tab_stop(Cm(14.8), WD_TAB_ALIGNMENT.RIGHT)
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(2)
    # PowerShell/Word COM converts this marker into a native OMML equation.
    p.add_run(f"EQ::{unicode_math}||{number}")

    src = doc.add_paragraph(style="TeX Source")
    src.alignment = WD_ALIGN_PARAGRAPH.CENTER
    src.paragraph_format.space_before = Pt(0)
    src.paragraph_format.space_after = Pt(4)
    run = src.add_run("TeX：" + tex)
    set_run_font(run, east="等线", latin="Consolas", size=8.5, color="666666")


def configure(doc):
    styles = doc.styles
    normal = styles["Normal"]
    set_style(normal, "宋体", "Times New Roman", 12)

    title = styles["Title"]
    set_style(title, "黑体", "Times New Roman", 18, True)
    title.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.space_before = Pt(10)
    title.paragraph_format.space_after = Pt(10)
    ppr = title._element.get_or_add_pPr()
    old = ppr.find(qn("w:pBdr"))
    if old is not None:
        ppr.remove(old)
    borders = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "nil")
    borders.append(bottom)
    ppr.append(borders)

    h2 = styles["Heading 2"]
    set_style(h2, "黑体", "Times New Roman", 14, True)
    h2.paragraph_format.space_before = Pt(8)
    h2.paragraph_format.space_after = Pt(6)

    for name, font, size, color in [
        ("Equation Marker", "Cambria Math", 11.5, "000000"),
        ("TeX Source", "Consolas", 8.5, "666666"),
    ]:
        if name not in [s.name for s in styles]:
            style = styles.add_style(name, WD_STYLE_TYPE.PARAGRAPH)
        else:
            style = styles[name]
        set_style(style, "等线", font, size, False, color)


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc = Document()
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21), Cm(29.7)
    sec.top_margin, sec.bottom_margin = Cm(2.2), Cm(2.2)
    sec.left_margin, sec.right_margin = Cm(3.0), Cm(2.5)
    sec.header_distance, sec.footer_distance = Cm(1.25), Cm(1.25)
    configure(doc)

    hp = sec.header.paragraphs[0]
    hp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_run_font(hp.add_run("面向载荷挂装的并联机器人运动规划与控制方法研究"), size=9, color="777777")
    add_page_field(sec.footer.paragraphs[0])

    doc.core_properties.title = "第四章PID控制精简版"
    doc.core_properties.author = "谢振宇"

    title = doc.add_paragraph("第4章 基于Simscape的并联机器人轨迹跟踪控制研究", style="Title")
    keep_next(title)

    body(doc, "本章基于上一章IHSID方法获得的参考轨迹和支腿驱动力，建立Stewart并联机器人Simscape多体动力学模型，并采用驱动力前馈与腿长误差反馈相结合的控制结构，验证规划轨迹的跟踪性能。")

    h2 = doc.add_paragraph("4.1 Stewart并联机器人PID控制", style="Heading 2")
    keep_next(h2)

    body(doc, "本文采用关节空间控制方法，将动平台位姿跟踪转化为六条支腿的长度跟踪。根据参考位姿进行逆运动学求解，得到各支腿的参考长度，并以参考腿长与实际腿长之差作为反馈误差。第i条支腿的腿长误差为：")
    equation_marker(
        doc,
        "e_i(t)=L_(r,i)(t)-L_i(t)",
        "(4.1)",
        r"e_i(t)=L_{r,i}(t)-L_i(t)",
    )

    body(doc, "针对各支腿分别设计PID控制器。考虑理想微分环节容易放大高频噪声，仿真中采用带微分滤波的PIDF形式，其反馈驱动力为：")
    equation_marker(
        doc,
        "F_(PID,i)(t)=K_(p,i)e_i(t)+K_(i,i)∫_0^t e_i(τ)dτ+K_(d,i) (de_i(t))/(dt)",
        "(4.2)",
        r"F_{\mathrm{PID},i}(t)=K_{p,i}e_i(t)+K_{i,i}\int_0^t e_i(\tau)\,\mathrm{d}\tau+K_{d,i}\frac{\mathrm{d}e_i(t)}{\mathrm{d}t}",
    )

    body(doc, "其中，比例、积分和微分环节分别用于提高响应速度、减小稳态误差和改善动态响应。本文在初始工作点对Simscape模型进行线性化，自动整定六组PIDF控制器，目标闭环带宽设为10 Hz；闭环极点实部均小于零，满足线性稳定性要求。")

    body(doc, "为利用轨迹优化所得的动力学信息，支腿总驱动力由优化驱动力前馈与PIDF反馈共同构成：")
    equation_marker(
        doc,
        "F_i(t)=F_(ff,i)(t)+F_(PID,i)(t)",
        "(4.3)",
        r"F_i(t)=F_{\mathrm{ff},i}(t)+F_{\mathrm{PID},i}(t)",
    )

    body(doc, "其中，前馈项主要补偿平台重力和运动载荷，反馈项用于修正腿长跟踪偏差。在理想模型条件下，对完整7.5 s挂装轨迹进行闭环仿真。最大平移跟踪误差为3.375 mm，最大姿态跟踪误差为0.203°，最大腿长跟踪误差为1.152 mm；支腿长度范围为1.078～1.905 m，最大总驱动力约为1105 N，未超过±2000 N的限制。")

    body(doc, "结果表明，该方法能够在理想条件下稳定跟踪规划轨迹，可作为后续鲁棒控制的基准方案；其在非理想条件下的性能仍需进一步验证。")

    doc.save(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    main()
