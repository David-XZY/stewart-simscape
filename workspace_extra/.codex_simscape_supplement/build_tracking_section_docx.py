from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


OUTPUT = Path(r"D:\NPU\并联机器人\中期\Simscape跟踪控制补充\第3节_Simscape跟踪控制补充正文.docx")


def set_run_font(run, name="宋体", size=Pt(12), bold=False, color=None):
    run.font.name = name
    run._element.rPr.rFonts.set(qn("w:eastAsia"), name)
    run.font.size = size
    run.font.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor(*color)


def set_paragraph_spacing(paragraph, first_line=True, before=0, after=0):
    fmt = paragraph.paragraph_format
    fmt.line_spacing = 1.35
    fmt.space_before = Pt(before)
    fmt.space_after = Pt(after)
    if first_line:
        fmt.first_line_indent = Pt(22)


def add_body(document, text):
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    set_paragraph_spacing(paragraph)
    run = paragraph.add_run(text)
    set_run_font(run, size=Pt(11))
    return paragraph


def add_heading(document, text):
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.LEFT
    set_paragraph_spacing(paragraph, first_line=False, before=8, after=4)
    run = paragraph.add_run(text)
    set_run_font(run, name="黑体", size=Pt(13), bold=True)
    return paragraph


def add_figure_placeholder(document, file_name, caption, note=None):
    placeholder = document.add_paragraph()
    placeholder.alignment = WD_ALIGN_PARAGRAPH.CENTER
    placeholder.paragraph_format.space_before = Pt(8)
    placeholder.paragraph_format.space_after = Pt(3)
    run = placeholder.add_run(f"【此处插入：{file_name}】")
    set_run_font(run, name="宋体", size=Pt(10.5), color=(100, 100, 100))

    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_before = Pt(0)
    paragraph.paragraph_format.space_after = Pt(3)
    run = paragraph.add_run(caption)
    set_run_font(run, name="宋体", size=Pt(10.5))

    if note:
        paragraph = document.add_paragraph()
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
        paragraph.paragraph_format.space_before = Pt(0)
        paragraph.paragraph_format.space_after = Pt(6)
        run = paragraph.add_run(note)
        set_run_font(run, name="宋体", size=Pt(9), color=(90, 90, 90))


def add_bottom_border(paragraph, color="B7C9DF", size="8"):
    p_pr = paragraph._p.get_or_add_pPr()
    borders = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), size)
    bottom.set(qn("w:space"), "1")
    bottom.set(qn("w:color"), color)
    borders.append(bottom)
    p_pr.append(borders)


def build():
    document = Document()
    section = document.sections[0]
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.5)
    section.bottom_margin = Cm(2.5)
    section.left_margin = Cm(2.7)
    section.right_margin = Cm(2.7)

    normal = document.styles["Normal"]
    normal.font.name = "宋体"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    normal.font.size = Pt(11)

    title = document.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.space_after = Pt(8)
    title_run = title.add_run("第3节 Simscape跟踪控制仿真补充正文")
    set_run_font(title_run, name="黑体", size=Pt(16), bold=True)
    add_bottom_border(title)

    note = document.add_paragraph()
    note.alignment = WD_ALIGN_PARAGRAPH.LEFT
    note.paragraph_format.space_before = Pt(4)
    note.paragraph_format.space_after = Pt(8)
    note_run = note.add_run(
        "使用说明：以下内容建议接在原报告图6后；原文“目前尚未开展Simscape对于轨迹的跟踪控制”可由本稿替换。"
    )
    set_run_font(note_run, name="宋体", size=Pt(10.5), color=(70, 90, 115))

    add_heading(document, "3.1 跟踪控制方法")
    add_body(
        document,
        "为验证轨迹优化结果在多体动力学模型中的可执行性，将IHSID方法得到的参考位姿、参考腿长及参考支腿驱动力映射至Simscape并联平台模型。"
        "其中，参考位姿用于描述平台的期望运动过程，参考腿长作为闭环控制目标，优化得到的支腿驱动力则作为前馈控制量，从而建立轨迹优化结果与物理仿真模型之间的控制接口。"
    )
    add_body(
        document,
        "跟踪控制采用“驱动力前馈与腿长误差反馈相结合”的控制结构。实际腿长与参考腿长之差经腿长反馈控制器转换为反馈驱动力，并与优化轨迹给出的前馈驱动力叠加后共同作用于六条支腿。"
        "反馈控制器带宽设置为10 Hz，以提高系统对腿长偏差的修正能力，同时保留优化驱动力对平台重力和运动载荷的补偿作用。"
        "平台实际位姿仅用于计算跟踪误差和进行结果验收，不参与闭环控制，其控制结构如图7所示。"
    )
    add_figure_placeholder(
        document,
        "图7_Simscape跟踪控制结构.png",
        "图7 优化轨迹驱动力前馈与腿长误差反馈的Simscape跟踪控制结构",
    )

    add_heading(document, "3.2 跟踪控制仿真结果")
    add_body(
        document,
        "在理想模型条件下对完整7.5 s参考轨迹进行闭环跟踪仿真，并在5.0 s处标记两阶段运动过程的切换时刻。"
        "由图8(a)可见，平台实际运动轨迹与参考轨迹整体保持一致；图8(b)和图8(c)进一步给出了平移与姿态跟踪误差，误差在完整运动过程中均保持有界，未出现不可控的姿态或位姿偏离。"
    )
    add_body(
        document,
        "仿真结果表明，最大平移跟踪误差为3.375 mm，低于10 mm验收要求；最大姿态跟踪误差为0.203°，低于1°验收要求。"
        "六条支腿的最大腿长跟踪误差为1.152 mm，实际腿长范围为1.078～1.905 m，均处于模型允许范围内。"
        "由图8(d)可知，支腿总驱动力峰值为1105 N，未超过±2000 N驱动力限制。"
        "因此，所构建的控制方案能够在当前理想仿真条件下稳定完成优化参考轨迹的跟踪。"
    )
    add_figure_placeholder(
        document,
        "图8_Simscape跟踪控制仿真结果.png",
        "图8 Simscape参考轨迹跟踪控制仿真结果",
        "注：(a) 平台参考与实际三维运动轨迹；(b) 三轴平移跟踪误差；(c) 三轴姿态跟踪误差；(d) 六条支腿总驱动力。",
    )

    add_body(
        document,
        "目前已完成理想条件下优化轨迹与Simscape模型之间的基础闭环跟踪验证。"
        "后续研究将在此基础上进一步引入执行器动态、参数不确定性、外部扰动和测量噪声，分析控制器的鲁棒性，并研究更适用于实际并联机器人系统的跟踪控制方法。"
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    document.save(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    build()
