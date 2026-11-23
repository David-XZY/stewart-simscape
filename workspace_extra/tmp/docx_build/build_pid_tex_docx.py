from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


OUTPUT = Path(r"D:\NPU\Matlab_code\output\documents\第四章_PID控制_TeX公式版.docx")


def set_run_fonts(run, east_asia="宋体", latin="Times New Roman", size=12,
                  bold=None, italic=None, color="000000"):
    run.font.name = latin
    run._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), latin)
    run._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), latin)
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), east_asia)
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    run.font.color.rgb = RGBColor.from_string(color)


def set_style_fonts(style, east_asia, latin, size, bold=False, color="000000"):
    style.font.name = latin
    style._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), latin)
    style._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), latin)
    style._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), east_asia)
    style.font.size = Pt(size)
    style.font.bold = bold
    style.font.color.rgb = RGBColor.from_string(color)


def add_page_field(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run()
    fld_char_begin = OxmlElement("w:fldChar")
    fld_char_begin.set(qn("w:fldCharType"), "begin")
    instr_text = OxmlElement("w:instrText")
    instr_text.set(qn("xml:space"), "preserve")
    instr_text.text = " PAGE "
    fld_char_end = OxmlElement("w:fldChar")
    fld_char_end.set(qn("w:fldCharType"), "end")
    run._r.append(fld_char_begin)
    run._r.append(instr_text)
    run._r.append(fld_char_end)
    set_run_fonts(run, east_asia="宋体", latin="Times New Roman", size=9)


def keep_with_next(paragraph):
    p_pr = paragraph._p.get_or_add_pPr()
    p_pr.append(OxmlElement("w:keepNext"))


def add_body(doc, text):
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p.paragraph_format.first_line_indent = Pt(24)
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(8)
    p.paragraph_format.line_spacing = 1.5
    run = p.add_run(text)
    set_run_fonts(run)
    return p


def add_tex_formula(doc, tex_lines):
    p = doc.add_paragraph(style="TeX Formula")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.left_indent = Pt(0)
    p.paragraph_format.right_indent = Pt(0)
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(8)
    p.paragraph_format.line_spacing = 1.15
    for index, line in enumerate(tex_lines):
        if index:
            p.add_run().add_break()
        run = p.add_run(line)
        set_run_fonts(run, east_asia="等线", latin="Consolas", size=10.5)
    return p


def configure_styles(doc):
    styles = doc.styles

    normal = styles["Normal"]
    set_style_fonts(normal, "宋体", "Times New Roman", 12)
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(8)
    normal.paragraph_format.line_spacing = 1.5

    title = styles["Title"]
    set_style_fonts(title, "黑体", "Times New Roman", 18, bold=True)
    title.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.space_before = Pt(10)
    title.paragraph_format.space_after = Pt(8)
    # Explicitly suppress the decorative bottom rule carried by some Word templates.
    title_p_pr = title._element.get_or_add_pPr()
    existing_borders = title_p_pr.find(qn("w:pBdr"))
    if existing_borders is not None:
        title_p_pr.remove(existing_borders)
    title_borders = OxmlElement("w:pBdr")
    title_bottom = OxmlElement("w:bottom")
    title_bottom.set(qn("w:val"), "nil")
    title_borders.append(title_bottom)
    title_p_pr.append(title_borders)

    subtitle = styles["Subtitle"]
    set_style_fonts(subtitle, "楷体", "Times New Roman", 10.5, color="666666")
    subtitle.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    subtitle.paragraph_format.space_before = Pt(0)
    subtitle.paragraph_format.space_after = Pt(18)

    heading1 = styles["Heading 1"]
    set_style_fonts(heading1, "黑体", "Times New Roman", 16, bold=True)
    heading1.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    heading1.paragraph_format.space_before = Pt(18)
    heading1.paragraph_format.space_after = Pt(10)

    heading2 = styles["Heading 2"]
    set_style_fonts(heading2, "黑体", "Times New Roman", 14, bold=True)
    heading2.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
    heading2.paragraph_format.space_before = Pt(14)
    heading2.paragraph_format.space_after = Pt(8)

    if "TeX Formula" not in [style.name for style in styles]:
        tex_style = styles.add_style("TeX Formula", WD_STYLE_TYPE.PARAGRAPH)
    else:
        tex_style = styles["TeX Formula"]
    set_style_fonts(tex_style, "等线", "Consolas", 10.5)
    tex_style.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    tex_style.paragraph_format.space_before = Pt(6)
    tex_style.paragraph_format.space_after = Pt(8)
    tex_style.paragraph_format.line_spacing = 1.15


def build_document():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc = Document()
    section = doc.sections[0]

    # narrative_proposal preset with named Chinese academic overrides:
    # A4 page, Chinese thesis typography, restrained black hierarchy.
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.54)
    section.bottom_margin = Cm(2.54)
    section.left_margin = Cm(3.0)
    section.right_margin = Cm(2.5)
    section.header_distance = Cm(1.25)
    section.footer_distance = Cm(1.25)

    configure_styles(doc)

    header = section.header
    header_p = header.paragraphs[0]
    header_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    header_p.paragraph_format.space_after = Pt(0)
    header_run = header_p.add_run("面向载荷挂装的并联机器人运动规划与控制方法研究")
    set_run_fonts(header_run, east_asia="宋体", latin="Times New Roman", size=9, color="666666")

    footer_p = section.footer.paragraphs[0]
    add_page_field(footer_p)

    doc.core_properties.title = "第4章 基于Simscape的并联机器人轨迹跟踪控制研究"
    doc.core_properties.subject = "第四章章首引言与PID/PIDF控制部分"
    doc.core_properties.author = "谢振宇"

    title = doc.add_paragraph("第4章 基于Simscape的并联机器人轨迹跟踪控制研究", style="Title")
    keep_with_next(title)
    subtitle = doc.add_paragraph("章节草稿｜第四章开头与PID控制部分｜公式采用TeX线性格式", style="Subtitle")
    keep_with_next(subtitle)

    add_body(
        doc,
        "本章进一步研究面向载荷挂装任务的Stewart并联机器人轨迹跟踪控制问题。首先，基于上一章采用IHSID方法获得的参考位姿、参考腿长及参考支腿驱动力，建立6-UCU型Stewart并联机器人的Simscape多体动力学模型。随后，将参考位姿通过逆运动学转换为六条支腿的参考伸缩量，并采用驱动力前馈与腿长误差反馈相结合的控制结构，实现六条支腿的协调驱动以及动平台对规划轨迹的闭环跟踪。最后，在理想模型条件下对完整挂装轨迹进行仿真，并从平台位姿跟踪误差、支腿长度跟踪误差及驱动力约束等方面分析控制效果，以验证所规划轨迹在多体动力学模型中的可执行性，并为后续开展参数不确定性、外部扰动及测量噪声条件下的鲁棒控制研究奠定基础。"
    )

    h2 = doc.add_paragraph("4.1 Stewart并联机器人PID控制", style="Heading 2")
    keep_with_next(h2)

    add_body(
        doc,
        "PID控制器具有结构简单、参数物理意义明确和工程实现方便等特点，广泛应用于机电系统的轨迹跟踪控制。针对本文研究的Stewart并联机器人，采用关节空间控制方法，将动平台位姿跟踪问题转化为六条支腿的长度跟踪问题。首先，根据规划得到的动平台参考位姿，通过逆运动学计算六条支腿的参考长度；随后，以参考腿长与实际腿长之间的偏差作为反馈量，通过PID控制器生成反馈驱动力，从而实现动平台对规划轨迹的跟踪。"
    )

    add_body(
        doc,
        "设第i条支腿的参考长度和实际长度分别为Lr,i(t)和Li(t)，则腿长跟踪误差定义为："
    )
    add_tex_formula(doc, [r"e_i(t)=L_{r,i}(t)-L_i(t) \tag{4.1}"])

    add_body(doc, "PID控制器产生的反馈驱动力可以表示为：")
    add_tex_formula(
        doc,
        [
            r"F_{\mathrm{PID},i}(t)=K_{p,i}e_i(t)+K_{i,i}\int_0^t e_i(\tau)\,\mathrm{d}\tau",
            r"\qquad{}+K_{d,i}\frac{\mathrm{d}e_i(t)}{\mathrm{d}t} \tag{4.2}",
        ],
    )

    add_body(
        doc,
        "式中，Kp,i、Ki,i和Kd,i分别为第i条支腿控制器的比例、积分和微分系数。比例环节根据当前腿长误差快速产生控制作用，用于改善系统的响应速度；积分环节对误差进行累积，用于减小系统的稳态误差；微分环节根据误差变化趋势提供阻尼作用，从而抑制超调并改善系统的动态响应。"
    )

    add_body(
        doc,
        "考虑到理想微分环节容易放大高频噪声，本文在实际仿真中采用带微分滤波的PID控制器，即PIDF控制器。第i条支腿PIDF控制器的传递函数可以写为："
    )
    add_tex_formula(
        doc,
        [r"C_i(s)=K_{p,i}+\frac{K_{i,i}}{s}+\frac{K_{d,i}s}{T_{f,i}s+1} \tag{4.3}"],
    )

    add_body(
        doc,
        "式中，Tf,i为微分滤波时间常数。与直接依据经验选取PID参数不同，本文首先在初始工作点对Simscape多体动力学模型进行线性化，获得支腿驱动力至支腿长度的六输入六输出线性模型："
    )
    add_tex_formula(
        doc,
        [r"G_{FL}(s)=\frac{\Delta L(s)}{\Delta F(s)}\in\mathbb{R}^{6\times6} \tag{4.4}"],
    )

    add_body(
        doc,
        "随后选取各支腿对应的对角通道，采用自动整定方法设计六组PIDF控制器，并将目标闭环带宽设置为10 Hz。六支腿控制器组成的对角控制矩阵为："
    )
    add_tex_formula(
        doc,
        [r"C(s)=\operatorname{diag}\!\left(C_1(s),C_2(s),\ldots,C_6(s)\right) \tag{4.5}"],
    )

    add_body(
        doc,
        "整定完成后，对线性闭环系统的极点位置和低频秩进行检查。计算结果表明，线性化对象的低频秩为6，闭环极点实部均小于零，说明所设计的PIDF控制器在该工作点附近满足线性稳定性要求。"
    )

    add_body(
        doc,
        "为了充分利用轨迹优化过程中获得的动力学信息，本文采用驱动力前馈与PIDF反馈相结合的控制结构。第i条支腿的总驱动力为："
    )
    add_tex_formula(
        doc,
        [r"F_i(t)=F_{\mathrm{ff},i}(t)+F_{\mathrm{PID},i}(t) \tag{4.6}"],
    )

    add_body(
        doc,
        "式中，Fff,i(t)为IHSID轨迹优化得到的参考支腿驱动力，主要用于补偿平台重力和运动载荷；FPID,i(t)为腿长误差反馈驱动力，主要用于修正多体动力学响应及轨迹跟踪过程中产生的偏差。该控制结构既保留了优化驱动力的动力学补偿作用，又利用PIDF反馈提高了系统的轨迹跟踪能力。"
    )

    add_body(
        doc,
        "在理想模型条件下，对完整的7.5 s挂装轨迹进行闭环跟踪仿真。仿真过程中，动平台实际运动轨迹与参考轨迹整体保持一致，平移误差、姿态误差和腿长误差均保持有界。最大平移跟踪误差为3.375 mm，最大姿态跟踪误差为0.203°，六条支腿的最大腿长跟踪误差为1.152 mm。支腿实际长度范围为1.078～1.905 m，最大总驱动力约为1105 N，未超过±2000 N的驱动力限制。"
    )

    add_body(
        doc,
        "上述结果表明，在模型参数准确且未考虑外部扰动、测量噪声和执行器动态的理想条件下，驱动力前馈与PIDF反馈相结合的控制方法能够稳定完成规划轨迹的跟踪，并满足支腿运动范围和驱动力约束。因此，该控制方法可以作为后续改进控制算法的基准方案。但由于PID控制器对模型不确定性和外部扰动的适应能力有限，其在非理想条件下的跟踪精度和鲁棒性仍需进一步研究。"
    )

    doc.save(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    build_document()
