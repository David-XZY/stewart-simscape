from pathlib import Path
from copy import deepcopy

from PIL import Image, ImageChops, ImageDraw, ImageFont
from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.table import WD_ALIGN_VERTICAL, WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


ROOT = Path(r"D:\NPU\Matlab_code")
WORK = ROOT / "_docx_work"
ASSET_DIR = WORK / "assets"
OUT = ROOT / "第四章第一节_Simscape联合仿真平台设计_论文稿_定稿.docx"
SOURCE_FIG_DIR = ROOT / "stewart-simscape" / "docs" / "controller_workflow" / "figures" / "simulation_workflow"

ASSET_DIR.mkdir(parents=True, exist_ok=True)


def font_path() -> Path:
    candidates = [
        Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
        Path(r"C:\Windows\Fonts\simsun.ttc"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("No Chinese font found")


def crop_white(input_path: Path, output_path: Path, padding: int = 24) -> None:
    image = Image.open(input_path).convert("RGB")
    background = Image.new("RGB", image.size, "white")
    difference = ImageChops.difference(image, background).convert("L")
    difference = difference.point(lambda value: 255 if value > 12 else 0)
    box = difference.getbbox()
    if box is None:
        image.save(output_path)
        return
    left = max(0, box[0] - padding)
    top = max(0, box[1] - padding)
    right = min(image.width, box[2] + padding)
    bottom = min(image.height, box[3] + padding)
    image.crop((left, top, right, bottom)).save(output_path)


def draw_centered_text(draw, box, text, font, fill=(35, 35, 35), spacing=4):
    x0, y0, x1, y1 = box
    max_width = x1 - x0 - 26
    lines = []
    for source_line in text.split("\n"):
        line = ""
        for char in source_line:
            trial = line + char
            if draw.textlength(trial, font=font) <= max_width:
                line = trial
            else:
                lines.append(line)
                line = char
        lines.append(line)
    line_height = font.size + spacing
    total_height = len(lines) * line_height - spacing
    y = y0 + (y1 - y0 - total_height) / 2
    for line in lines:
        width = draw.textlength(line, font=font)
        draw.text((x0 + (x1 - x0 - width) / 2, y), line, font=font, fill=fill)
        y += line_height


def rounded_box(draw, box, fill, outline=(70, 70, 70), radius=18, width=3):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def arrow(draw, start, end, fill=(70, 70, 70), width=4, head=14):
    draw.line([start, end], fill=fill, width=width)
    x1, y1 = start
    x2, y2 = end
    dx, dy = x2 - x1, y2 - y1
    length = max((dx * dx + dy * dy) ** 0.5, 1)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    p1 = (x2 - head * ux + 0.55 * head * px, y2 - head * uy + 0.55 * head * py)
    p2 = (x2 - head * ux - 0.55 * head * px, y2 - head * uy - 0.55 * head * py)
    draw.polygon([end, p1, p2], fill=fill)


def build_architecture_figure(output_path: Path) -> None:
    canvas = Image.new("RGB", (2400, 980), "white")
    draw = ImageDraw.Draw(canvas)
    fp = font_path()
    title_font = ImageFont.truetype(str(fp), 50)
    box_font = ImageFont.truetype(str(fp), 30)
    small_font = ImageFont.truetype(str(fp), 27)
    draw.text((1200, 55), "参考轨迹之后的统一 Simscape 验证链", font=title_font, fill=(30, 30, 30), anchor="mm")

    boxes = [
        (70, 220, 350, 410, "轨迹数据\nt, q, q̇, q̈", (238, 240, 243)),
        (405, 220, 705, 410, "参考接口\nHermite/IK/Jacobian", (214, 231, 247)),
        (760, 220, 1045, 410, "名义控制\n计算力矩-LQI", (214, 231, 247)),
        (1100, 220, 1395, 410, "安全滤波\nCLF-CBF-QP", (226, 237, 249)),
        (1450, 220, 1735, 410, "执行器 Variant\n理想/非理想力", (251, 221, 181)),
        (1790, 220, 2095, 410, "Simscape 多体对象\n平台/支链/载荷/重力", (213, 220, 231)),
        (2150, 220, 2350, 410, "测量与日志", (203, 235, 211)),
    ]
    for x0, y0, x1, y1, label, color in boxes:
        rounded_box(draw, (x0, y0, x1, y1), color)
        draw_centered_text(draw, (x0, y0, x1, y1), label, box_font)
    for left, right in zip(boxes[:-1], boxes[1:]):
        arrow(draw, (left[2] + 10, 315), (right[0] - 10, 315))

    rounded_box(draw, (295, 620, 905, 810), (238, 240, 243))
    draw_centered_text(draw, (295, 620, 905, 810), "统一参数源\n几何、质量、惯量、重力、边界", small_font)
    rounded_box(draw, (1010, 620, 1530, 810), (242, 244, 247))
    draw_centered_text(draw, (1010, 620, 1530, 810), "一致性审计\n初始腿长、质心与惯量映射", small_font)
    rounded_box(draw, (1640, 620, 2220, 810), (218, 239, 222))
    draw_centered_text(draw, (1640, 620, 2220, 810), "逐点验收\n有限性、跟踪、约束、可行性与耗时", small_font)
    arrow(draw, (905, 715), (1000, 715))
    arrow(draw, (1530, 715), (1630, 715))
    arrow(draw, (600, 610), (600, 430))
    arrow(draw, (1270, 610), (1270, 430))
    arrow(draw, (1930, 430), (1930, 610))

    draw.arc((925, 410, 2225, 690), 0, 180, fill=(66, 103, 140), width=4)
    arrow(draw, (927, 550), (925, 548), fill=(66, 103, 140), width=4)
    draw.text((1525, 520), "状态反馈与约束量", font=small_font, fill=(66, 103, 140), anchor="mm")
    canvas.save(output_path)


def prepare_assets():
    build_architecture_figure(ASSET_DIR / "fig_4_1_architecture.png")
    mappings = {
        "07_simscape_top_level.png": "fig_4_2_top_level.png",
        "09_simscape_strut_layer.png": "fig_4_3_strut.png",
        "10_parameter_mapping.png": "fig_4_4_mapping.png",
        "12_reference_dataflow.png": "fig_4_5_reference_flow.png",
    }
    for source_name, target_name in mappings.items():
        crop_white(SOURCE_FIG_DIR / source_name, ASSET_DIR / target_name)


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for margin, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{margin}"))
        if node is None:
            node = OxmlElement(f"w:{margin}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def shade_cell(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shading = tc_pr.find(qn("w:shd"))
    if shading is None:
        shading = OxmlElement("w:shd")
        tc_pr.append(shading)
    shading.set(qn("w:fill"), fill)


def set_cell_width(cell, width_dxa):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_w = tc_pr.find(qn("w:tcW"))
    if tc_w is None:
        tc_w = OxmlElement("w:tcW")
        tc_pr.append(tc_w)
    tc_w.set(qn("w:w"), str(width_dxa))
    tc_w.set(qn("w:type"), "dxa")


def set_table_geometry(table, widths, indent=120):
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    table.autofit = False
    tbl_pr = table._tbl.tblPr
    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(sum(widths)))
    tbl_w.set(qn("w:type"), "dxa")
    tbl_ind = tbl_pr.find(qn("w:tblInd"))
    if tbl_ind is None:
        tbl_ind = OxmlElement("w:tblInd")
        tbl_pr.append(tbl_ind)
    tbl_ind.set(qn("w:w"), str(indent))
    tbl_ind.set(qn("w:type"), "dxa")
    layout = tbl_pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        tbl_pr.append(layout)
    layout.set(qn("w:type"), "fixed")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths:
        grid_col = OxmlElement("w:gridCol")
        grid_col.set(qn("w:w"), str(width))
        grid.append(grid_col)
    for row in table.rows:
        for index, cell in enumerate(row.cells):
            set_cell_width(cell, widths[index])
            cell.width = width_dxa_to_inches(widths[index])


def width_dxa_to_inches(value):
    return Inches(value / 1440)


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    header = OxmlElement("w:tblHeader")
    header.set(qn("w:val"), "true")
    tr_pr.append(header)


def set_font(run, east="宋体", west="Times New Roman", size=12, bold=None, italic=None, color=None):
    run.font.name = west
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    if color:
        run.font.color.rgb = RGBColor.from_string(color)
    r_pr = run._element.get_or_add_rPr()
    r_fonts = r_pr.rFonts
    if r_fonts is None:
        r_fonts = OxmlElement("w:rFonts")
        r_pr.insert(0, r_fonts)
    r_fonts.set(qn("w:ascii"), west)
    r_fonts.set(qn("w:hAnsi"), west)
    r_fonts.set(qn("w:eastAsia"), east)
    r_fonts.set(qn("w:cs"), west)


def set_style_font(style, east, west, size, bold=None, color="000000"):
    style.font.name = west
    style.font.size = Pt(size)
    style.font.bold = bold
    style.font.color.rgb = RGBColor.from_string(color)
    r_pr = style.element.get_or_add_rPr()
    r_fonts = r_pr.rFonts
    if r_fonts is None:
        r_fonts = OxmlElement("w:rFonts")
        r_pr.insert(0, r_fonts)
    r_fonts.set(qn("w:ascii"), west)
    r_fonts.set(qn("w:hAnsi"), west)
    r_fonts.set(qn("w:eastAsia"), east)
    r_fonts.set(qn("w:cs"), west)


def set_outline_level(style, level):
    p_pr = style.element.get_or_add_pPr()
    outline = p_pr.find(qn("w:outlineLvl"))
    if outline is None:
        outline = OxmlElement("w:outlineLvl")
        p_pr.append(outline)
    outline.set(qn("w:val"), str(level))


def configure_styles(doc):
    styles = doc.styles
    normal = styles["Normal"]
    set_style_font(normal, "宋体", "Times New Roman", 12)
    normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    normal.paragraph_format.first_line_indent = Pt(24)
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(0)
    normal.paragraph_format.line_spacing = 1.5

    for name, size, before, after, level in [
        ("Heading 1", 15, 12, 7, 1),
        ("Heading 2", 14, 9, 5, 2),
        ("Heading 3", 12, 7, 4, 3),
    ]:
        style = styles[name]
        set_style_font(style, "黑体", "Times New Roman", size, bold=False)
        style.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
        style.paragraph_format.first_line_indent = Pt(0)
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.line_spacing = 1.0
        style.paragraph_format.keep_with_next = True
        style.paragraph_format.keep_together = True
        set_outline_level(style, level)

    chapter = styles.add_style("Chapter Title Thesis", WD_STYLE_TYPE.PARAGRAPH)
    set_style_font(chapter, "黑体", "Times New Roman", 16, bold=False)
    chapter.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    chapter.paragraph_format.first_line_indent = Pt(0)
    chapter.paragraph_format.space_before = Pt(18)
    chapter.paragraph_format.space_after = Pt(12)
    chapter.paragraph_format.line_spacing = 1.0
    chapter.paragraph_format.keep_with_next = True
    set_outline_level(chapter, 0)

    caption = styles.add_style("Caption Thesis", WD_STYLE_TYPE.PARAGRAPH)
    set_style_font(caption, "宋体", "Times New Roman", 10.5)
    caption.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    caption.paragraph_format.first_line_indent = Pt(0)
    caption.paragraph_format.space_before = Pt(3)
    caption.paragraph_format.space_after = Pt(6)
    caption.paragraph_format.line_spacing = 1.0

    equation = styles.add_style("Equation Thesis", WD_STYLE_TYPE.PARAGRAPH)
    set_style_font(equation, "Cambria Math", "Cambria Math", 11)
    equation.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
    equation.paragraph_format.first_line_indent = Pt(0)
    equation.paragraph_format.space_before = Pt(4)
    equation.paragraph_format.space_after = Pt(4)
    equation.paragraph_format.line_spacing = 1.0

    note = styles.add_style("Note Thesis", WD_STYLE_TYPE.PARAGRAPH)
    set_style_font(note, "宋体", "Times New Roman", 10.5)
    note.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    note.paragraph_format.first_line_indent = Pt(0)
    note.paragraph_format.left_indent = Pt(12)
    note.paragraph_format.right_indent = Pt(12)
    note.paragraph_format.space_before = Pt(3)
    note.paragraph_format.space_after = Pt(5)
    note.paragraph_format.line_spacing = 1.25


def add_field(paragraph, instruction, placeholder="1"):
    begin_run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    begin_run._r.append(begin)

    instruction_run = paragraph.add_run()
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = instruction
    instruction_run._r.append(instr)

    separate_run = paragraph.add_run()
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    separate_run._r.append(separate)

    result_run = paragraph.add_run()
    text = OxmlElement("w:t")
    text.text = placeholder
    result_run._r.append(text)

    end_run = paragraph.add_run()
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    end_run._r.append(end)
    for run in (begin_run, instruction_run, separate_run, result_run, end_run):
        set_font(run, east="宋体", west="Times New Roman", size=9)


def configure_page(doc):
    section = doc.sections[0]
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.5)
    section.bottom_margin = Cm(2.5)
    section.left_margin = Cm(2.5)
    section.right_margin = Cm(2.5)
    section.header_distance = Cm(1.5)
    section.footer_distance = Cm(1.75)
    header = section.header
    p = header.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run("第4章  Simscape多体建模与统一控制验证平台")
    set_font(r, east="宋体", west="Times New Roman", size=9)
    footer = section.footer
    fp = footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    fp.paragraph_format.space_after = Pt(0)
    footer_run = fp.add_run("1")
    set_font(footer_run, east="宋体", west="Times New Roman", size=9)


def add_body(doc, text):
    paragraph = doc.add_paragraph(style="Normal")
    run = paragraph.add_run(text)
    set_font(run, east="宋体", west="Times New Roman", size=12)
    return paragraph


def add_heading(doc, text, level=1):
    paragraph = doc.add_paragraph(style=f"Heading {level}")
    run = paragraph.add_run(text)
    set_font(run, east="黑体", west="Times New Roman", size={1: 15, 2: 14, 3: 12}[level])
    return paragraph


def add_equation(doc, expression, number):
    paragraph = doc.add_paragraph(style="Equation Thesis")
    tabs = paragraph.paragraph_format.tab_stops
    tabs.add_tab_stop(Cm(8.0), WD_TAB_ALIGNMENT.CENTER)
    tabs.add_tab_stop(Cm(16.0), WD_TAB_ALIGNMENT.RIGHT)
    run = paragraph.add_run(f"\t{expression}\t({number})")
    set_font(run, east="Cambria Math", west="Cambria Math", size=11)
    paragraph.paragraph_format.keep_together = True
    return paragraph


def add_figure(doc, image_path, caption_text, width_in=6.1, alt_text=None):
    picture_paragraph = doc.add_paragraph()
    picture_paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    picture_paragraph.paragraph_format.first_line_indent = Pt(0)
    picture_paragraph.paragraph_format.space_before = Pt(4)
    picture_paragraph.paragraph_format.space_after = Pt(2)
    picture_paragraph.paragraph_format.keep_with_next = True
    run = picture_paragraph.add_run()
    inline_shape = run.add_picture(str(image_path), width=Inches(width_in))
    if alt_text:
        doc_pr = inline_shape._inline.docPr
        doc_pr.set("descr", alt_text)
        doc_pr.set("title", caption_text)
    caption = doc.add_paragraph(style="Caption Thesis")
    caption.paragraph_format.keep_together = True
    r = caption.add_run(caption_text)
    set_font(r, east="宋体", west="Times New Roman", size=10.5)
    return caption


def add_table_caption(doc, caption_text):
    p = doc.add_paragraph(style="Caption Thesis")
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.keep_with_next = True
    r = p.add_run(caption_text)
    set_font(r, east="宋体", west="Times New Roman", size=10.5)
    return p


def add_table(doc, headers, rows, widths, alignments=None, font_size=10.5):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    set_table_geometry(table, widths)
    header_row = table.rows[0]
    set_repeat_table_header(header_row)
    for index, value in enumerate(headers):
        cell = header_row.cells[index]
        shade_cell(cell, "E7E6E6")
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        compact = font_size <= 9.5
        set_cell_margins(cell, top=45 if compact else 80, bottom=45 if compact else 80)
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.first_line_indent = Pt(0)
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.line_spacing = 1.0
        r = p.add_run(str(value))
        set_font(r, east="黑体", west="Times New Roman", size=font_size)
    for row_values in rows:
        row = table.add_row()
        for index, value in enumerate(row_values):
            cell = row.cells[index]
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            set_cell_margins(cell, top=45 if compact else 80, bottom=45 if compact else 80)
            p = cell.paragraphs[0]
            p.paragraph_format.first_line_indent = Pt(0)
            p.paragraph_format.space_before = Pt(0)
            p.paragraph_format.space_after = Pt(0)
            p.paragraph_format.line_spacing = 1.0 if compact else 1.1
            if alignments:
                p.alignment = alignments[index]
            else:
                p.alignment = WD_ALIGN_PARAGRAPH.LEFT
            r = p.add_run(str(value))
            set_font(r, east="宋体", west="Times New Roman", size=font_size)
    after = doc.add_paragraph()
    after.paragraph_format.first_line_indent = Pt(0)
    after.paragraph_format.space_before = Pt(0)
    after.paragraph_format.space_after = Pt(0)
    after.paragraph_format.line_spacing = 1.0
    return table


def add_note(doc, text):
    p = doc.add_paragraph(style="Note Thesis")
    r = p.add_run(text)
    set_font(r, east="宋体", west="Times New Roman", size=10.5)
    return p


def build_document():
    prepare_assets()
    doc = Document()
    configure_page(doc)
    configure_styles(doc)
    doc.core_properties.title = "第四章第一节：面向控制算法验证的Simscape联合仿真平台设计"
    doc.core_properties.subject = "Stewart并联机器人Simscape多体建模与统一控制验证平台"
    doc.core_properties.author = ""
    doc.core_properties.keywords = "Stewart; Simscape; Multibody; 计算力矩; LQI; CLF-CBF-QP"

    p = doc.add_paragraph(style="Chapter Title Thesis")
    r = p.add_run("第4章  Simscape多体建模与统一控制验证平台")
    set_font(r, east="黑体", west="Times New Roman", size=16)

    add_heading(doc, "4.1 面向控制算法验证的Simscape联合仿真平台设计", 1)
    add_body(doc, "上一章已经获得由时间、位姿、速度及相关支链量组成的参考轨迹。该结果回答了动平台“应当如何运动”，但尚不能说明在重力、载荷惯性、六支链耦合、执行器动态和测量闭环同时存在时，所设计的控制器能否实现该运动，也不能据此判断腿长、腿速、支链力、碰撞间隙和奇异性边界能否在闭环过程中持续满足。因此，在开展计算力矩-LQI和CLF-CBF-QP控制设计之前，需要先建立一个统一的物理被控对象，使后续各控制方案只改变控制律而不改变机构、载荷、参考轨迹和评价口径。")
    add_body(doc, "本文采用Simulink与Simscape Multibody构建Stewart并联机器人联合仿真平台[17]。平台并不以复现全部实物细节为目标，而是按照“物理耦合可解释、控制接口可替换、模型参数可追溯、约束指标可验收”的原则确定建模深度：多体层显式描述固定平台、六条支链、动平台、刚性载荷和重力；执行器层提供理想力与非理想力两类Variant；测量层提供位姿、速度、腿长和实际支链力；算法层允许计算力矩-LQI与CLF-CBF-QP在同一输入输出契约下切换。这样的层次化设计既保留控制算法研究所需的可重复性，又引入能够显著影响工程闭环的主要非理想因素。")

    add_heading(doc, "4.1.1 建模任务、研究问题与覆盖边界", 2)
    add_body(doc, "Simscape平台的建模内容并非机构零部件的简单堆叠，而是由论文需要回答的研究问题反向确定。对于本文的载荷挂装任务，平台至少需要覆盖空间刚体耦合、负载重力与惯性、支链输入可实现性、反馈信号因果性、轨迹接口连续性、多类安全边界以及不同控制器之间的公平比较。各问题与模型模块的对应关系如表4-1所示。")

    add_table_caption(doc, "表4-1  研究问题与Simscape平台组成的对应关系")
    add_table(
        doc,
        ["研究方面", "需要回答的问题", "对应模块", "设计作用"],
        [
            ["空间耦合", "六支链共同作用时，平台是否按预期产生六自由度运动", "固定平台、六支链、动平台及关节拓扑", "保留并联机构几何闭链和力/运动耦合"],
            ["负载动力学", "计算力矩前馈能否承担重力与参考运动载荷", "动平台质量、圆柱载荷、质心、惯量与重力", "检验模型前馈的物理一致性和参数敏感性"],
            ["执行器能力", "控制器给出的支链力是否可施加，滞后和限幅是否改变闭环", "理想力与非理想力执行器Variant", "分离控制算法能力与执行器非理想影响"],
            ["测量因果性", "控制器使用的状态是否能够由传感器链获得", "相对/绝对运动传感器、腿长与关节力测量", "避免使用未来信息或以差分替代可测速度"],
            ["参考连续性", "离散轨迹节点如何转化为连续、同步的控制参考", "Hermite重建、逆运动学与Jacobian接口", "抑制节点速度不连续引起的虚假加速度与高频力"],
            ["安全边界", "良好跟踪是否同时满足机构、执行器和环境约束", "腿长/速度/加速度、力、碰撞与奇异性诊断", "为后续CLF-CBF-QP提供统一状态和验收量"],
            ["公平复现", "不同控制方案的差异是否仅来自控制律", "共享参数源、Variant切换、统一日志与验收", "防止对象、初值或评价口径变化造成混淆"],
        ],
        [1300, 2450, 2450, 2752],
        [WD_ALIGN_PARAGRAPH.CENTER, WD_ALIGN_PARAGRAPH.LEFT, WD_ALIGN_PARAGRAPH.LEFT, WD_ALIGN_PARAGRAPH.LEFT],
        font_size=9.5,
    )
    add_body(doc, "由表4-1可知，本文所称“覆盖现实情况”是指覆盖对控制结论有一阶影响的主要物理与信息环节，而不是把仿真模型等同于硬件数字孪生。当前主模型保留刚体几何、负载惯量、重力、支链轴向输入、执行器动态和可测输出；支链柔性、关节间隙、复杂摩擦、结构振动和接触冲击则作为模型边界明确保留。该边界使本章的仿真结论能够被解释为给定模型和工况下的闭环证据，而不能直接外推为实物安全认证。")

    add_heading(doc, "4.1.2 平台总体架构与控制接口", 2)
    add_body(doc, "平台按照参考接口、控制算法、执行器、多体对象、测量日志和验收六个层次组织，整体信息流如图4-1所示。上游轨迹文件首先经过连续重建和运动学同步计算，形成规则时间网格上的参考位姿、速度、加速度及支链参考量；计算力矩-LQI生成名义支链力，严格CLF-CBF-QP在后续章节中作为安全滤波层对名义力作最小必要修正；经执行器Variant处理后的实际支链力作用于Simscape多体对象；传感器输出再回到控制器和安全约束计算模块，同时进入统一日志。")

    add_figure(doc, ASSET_DIR / "fig_4_1_architecture.png", "图4-1  参考轨迹之后的统一Simscape验证链", 6.2, "从参考轨迹、控制器、安全滤波、执行器到Simscape对象、测量日志和验收的总体架构")

    add_body(doc, "为保证控制器比较的可解释性，本文把“名义控制”与“物理施加”严格分开。基线控制器输出为计算力矩项与LQI反馈项之和；引入QP后，QP只在该名义力之后进行约束修正。相应关系写为")
    add_equation(doc, "F_nom = F_CT + F_LQI，    F_cmd = F_nom（基线）或 F*_QP（安全滤波）", "4-1")
    add_body(doc, "执行器模块接收F_cmd并输出实际支链力F_act。这样，计算力矩-LQI和CLF-CBF-QP共享同一个多体对象、测量反馈和执行器模型；当约束不活动时，可以检查QP是否保持基线行为；当约束趋于活动时，又可以把差异归因于在线约束协调，而不是被控对象变化。")
    add_body(doc, "实际Simscape顶层模型如图4-2所示。Reference模块提供参考信号，Controller模块生成反馈修正，Force Feedforward或计算力矩模块提供前馈力，两者在支链输入端合成；Stewart Platform子系统负责六支链与平台的多体运动，Relative Motion Sensor和Absolute Motion Sensor分别输出相对与绝对运动量，To Workspace链路统一保存控制量和状态量。负载、外力接口和重力配置作为独立模块存在，便于在不改变控制器结构的情况下设置不同工况。")
    add_figure(doc, ASSET_DIR / "fig_4_2_top_level.png", "图4-2  Stewart并联机器人Simscape顶层模型", 5.05, "Simscape顶层模型真实截图，包含参考、控制器、前馈、执行器、多体对象、载荷、运动传感器和日志接口")

    add_heading(doc, "4.1.3 Stewart多体被控对象的构建", 2)
    add_body(doc, "固定平台坐标系记为{A}，动平台坐标系记为{B}。动平台广义位姿采用平移与ZYX欧拉角组合表示，定义为")
    add_equation(doc, "q = [x  y  z  φ  θ  ψ]ᵀ，    R(q) = R_z(ψ)R_y(θ)R_x(φ)", "4-2")
    add_body(doc, "设第i条支链在固定平台上的铰点为A_i，在动平台坐标系中的对应铰点为B_i，则其空间向量、长度和单位方向分别为")
    add_equation(doc, "s_i = p + R(q)B_i - A_i，    L_i = ‖s_i‖，    u_i = s_i/L_i", "4-3")
    add_body(doc, "Simscape中分别建立固定平台刚体、动平台刚体与六条结构相同而参数索引不同的支链。每条支链由固定端关节、固定端杆件、Prismatic Joint、移动端杆件和移动端关节组成。固定端采用理想万向铰，移动端采用理想球铰，从而保留6-UCU机构的主要运动拓扑；Prismatic Joint负责轴向驱动力输入、伸缩运动和支链力测量。六条支链的上下铰点并非按相同序号直接相连，而是采用[6,1,2,3,4,5]的循环映射，因此铰点编号、初始方向、逆运动学和Simscape装配必须使用同一连接关系。")
    add_figure(doc, ASSET_DIR / "fig_4_3_strut.png", "图4-3  单条支链的Simscape组成与测量接口", 6.05, "单条支链模型真实截图，包含关节、杆件、执行器、惯性传感器以及腿长和关节力输出")
    add_body(doc, "动平台空间速度记为V=[vᵀ,ωᵀ]ᵀ。由支链轴向速度投影可得到腿速度Jacobian J_v，并由虚功原理建立六条支链力与平台Wrench之间的关系")
    add_equation(doc, "L̇ = J_v(q)V，    W_act = J_v(q)ᵀF_act", "4-4")
    add_body(doc, "上述关系说明Simscape平台并非六个相互独立的一维执行器：任一支链力变化都会通过J_vᵀ影响平台合力和合力矩，也会改变其他支链的运动。平台与圆柱载荷共同形成组合刚体，其广义动力学可概括为")
    add_equation(doc, "H(q)q̈ + W_bias(q,q̇) = J_v(q)ᵀF_act + W_ext", "4-5")
    add_body(doc, "其中H(q)为广义质量矩阵，W_bias包含重力、科氏项、离心项及姿态坐标映射项，W_ext为可选外扰。本文在Simscape中通过刚体、关节和重力场自然形成上述耦合，而不在控制器内部重复积分简化动力学。这样既可利用解析模型计算前馈和屏障约束，又可用独立的多体求解器检查控制器是否依赖同一套简化积分过程。")
    add_body(doc, "载荷采用沿动平台x轴横置的刚性圆柱体，其几何尺寸、质量、安装中心和惯量均由场景参数生成。障碍物间隙与碰撞证书在控制器侧采用同一圆柱和有向包围盒参数计算，但当前Simscape主模型不启用接触力。原因在于本文研究目标是通过CBF在接触发生之前保持安全距离，而不是分析碰撞之后的冲击动力学；一旦出现距离越界，该工况即按安全失败处理，不能用接触反力继续“托住”仿真。")

    add_heading(doc, "4.1.4 参数单一来源与模型一致性校核", 2)
    add_body(doc, "为防止轨迹优化、解析动力学、控制器与Simscape分别维护一套参数，本文把几何、质量、惯量、重力和物理边界集中定义在统一模型结构中，再映射为Simscape使用的stewart、payload、ground、disturbances和controller等基础工作区结构体。参数映射关系如图4-4所示。该做法使同一参数修改能够同时进入轨迹约束、计算力矩、CBF和多体对象，减少“控制器认为的机构”和“仿真实际机构”不一致的问题。")
    add_figure(doc, ASSET_DIR / "fig_4_4_mapping.png", "图4-4  统一模型参数到Simscape对象的映射", 6.0, "几何、质量、惯量、约束和重力从统一参数源映射到Simscape，并进行初始腿长和惯量一致性检查")

    add_table_caption(doc, "表4-2  Simscape平台主要几何、动力学与边界参数")
    add_table(
        doc,
        ["参数类别", "参数", "取值与说明"],
        [
            ["平台几何", "固定/动平台半径", "r_A=0.75 m，r_B=0.50 m"],
            ["初始状态", "初始位姿", "q_0=[0,0,1,0,0,0]ᵀ"],
            ["支链连接", "上铰点映射", "legMap=[6,1,2,3,4,5]"],
            ["机构边界", "腿长范围", "1.05～2.00 m"],
            ["机构边界", "腿速/腿加速度", "|L̇_i|≤0.60 m/s，|L̈_i|≤1.20 m/s²"],
            ["执行器边界", "支链轴向力", "-2000 N≤F_i≤2000 N"],
            ["动平台", "质量与等效形状", "50 kg，半径0.50 m的薄圆盘惯量"],
            ["刚性载荷", "圆柱参数", "质量100 kg，半径0.15 m，长度1.20 m，轴向沿动平台x轴"],
            ["组合刚体", "总质量与组合质心", "150 kg，组合质心在动平台坐标系中为[0,0,0.10]ᵀ m"],
            ["重力", "重力加速度", "g=[0,0,-9.81]ᵀ m/s²"],
            ["数值设置", "支链杆件正则质量", "每段10⁻³ kg，仅用于避免退化质量分布"],
            ["归一化", "特征长度与控制周期", "L_c=0.50 m，T_s=0.01 s"],
        ],
        [1700, 2500, 4752],
        [WD_ALIGN_PARAGRAPH.CENTER, WD_ALIGN_PARAGRAPH.LEFT, WD_ALIGN_PARAGRAPH.LEFT],
        font_size=9.4,
    )
    add_note(doc, "注：表中参数为本文主仿真模型的统一设置。支链正则质量不代表真实杆件质量；完整支链惯量、柔性和间隙未进入当前主模型。")
    add_body(doc, "映射完成后首先检查初始腿长。由解析逆运动学获得的L_i(q_0)应与Simscape装配得到的初始长度一致，即")
    add_equation(doc, "max_i |L_i,Simscape(q_0) - L_i,IK(q_0)| < 10⁻¹⁰ m", "4-6")
    add_body(doc, "其次检查动平台与载荷的组合质量、质心和惯量。载荷惯量由其质心转移至组合质心时采用平行轴定理")
    add_equation(doc, "I_O = I_C + m[(dᵀd)I₃ - ddᵀ]", "4-7")
    add_body(doc, "只有当总质量、组合质心、组合惯量和初始腿长的映射误差均不超过设定容差时，平台才允许进入控制器设计与闭环仿真。该校核能够在仿真开始前发现单位、坐标方向、铰点映射和载荷安装位置错误，避免把模型装配问题误判为控制性能问题。")

    add_heading(doc, "4.1.5 参考轨迹到Simscape的连续数据接口", 2)
    add_body(doc, "上游轨迹文件只作为已通过轨迹规划验收的数据源。进入本章时，节点数据至少包含时间t、位姿q、广义速度q̇、优化得到的支链力F_leg、腿长L和初始位姿q_0。Simscape连续求解器和10 ms控制器不能直接使用稀疏节点，因此需在相邻节点[t_k,t_(k+1)]之间构造三次Hermite插值。令τ=(t-t_k)/Δt_k，则位姿参考为")
    add_equation(doc, "q_r(τ)=h₀₀q_k+h₁₀Δt_k q̇_k+h₀₁q_(k+1)+h₁₁Δt_k q̇_(k+1)", "4-8")
    add_body(doc, "Hermite重建同时给出解析的一阶和二阶导数q̇_r、q̈_r，避免由分段线性插值和重复数值差分产生节点速度突变及虚假加速度尖峰。在同一规则时间网格上，再由逆运动学和Jacobian同步计算L_r和L̇_r，并根据当前动力学模型重新计算计算力矩前馈F_CT。原轨迹中的F_leg仍保留用于溯源和对比，但计算力矩-LQI主线使用与当前q_r、q̇_r、q̈_r一致的F_CT。")
    add_figure(doc, ASSET_DIR / "fig_4_5_reference_flow.png", "图4-5  参考轨迹进入Simscape的连续数据流", 6.0, "轨迹MAT文件经Hermite重建、逆运动学和Jacobian计算后形成Simscape From Workspace参考信号")
    add_body(doc, "重建后的信号转换为From Workspace可读取的timeseries，其中references.r=q_r-q_0为相对位姿参考，references.rd和references.rdd分别为绝对广义速度和加速度参考，references.rL=L_r-L_r(0)为相对腿长参考，references.rLd为腿速参考，references.uCT和references.uIHSID分别保存计算力矩力和上游优化力。所有信号使用相同时间戳，控制器按T_s=0.01 s更新并采用零阶保持，Simscape连续求解器在控制步之间积分多体动力学。")
    add_body(doc, "该接口设计将“参考轨迹质量”和“控制器质量”分离：若轨迹重建本身不连续，则计算力矩会被非物理加速度激励；若所有参考量在同一网格上连续一致，闭环误差才主要反映控制器、执行器和模型失配。后续控制方法比较均复用同一组重建参考，不针对某一控制器单独平滑或修改轨迹。")

    add_heading(doc, "4.1.6 执行器Variant与测量反馈链", 2)
    add_body(doc, "为同时服务算法上界研究和工程非理想性分析，支链执行器采用可替换Variant。理想力Variant把F_cmd直接施加到Prismatic Joint，用于检查几何、动力学、控制律和约束构造是否正确；非理想力Variant在同一输入端依次加入输入延迟、死区、力变化率限制、一阶力滞后和幅值饱和，其简化关系可写为")
    add_equation(doc, "τ_f Ḟ_a + F_a = R_Ḟ{D_δ[F_cmd(t-t_d)]}，    F_act = sat(F_a,±F_max)", "4-9")
    add_body(doc, "式中D_δ表示死区算子，R_Ḟ表示力变化率限制，τ_f为力响应时间常数，t_d为输入延迟。本文名义非理想工况采用F_max=2000 N、τ_f=1 ms、t_d=1 μs和零死区；模块接口仍保留死区、速率和时滞参数，以便开展时间常数放大、执行器余量和参数扰动试验。需要指出的是，该非理想力模型用于刻画宏观力通道，并不等同于完整电机-驱动器-丝杠电气模型。")
    add_body(doc, "测量链遵循“控制器只能读取当前可获得信号”的原则。相对运动传感器输出动平台相对初始位姿，绝对运动传感器输出世界坐标系线速度、角速度和绝对位姿；各Prismatic Joint输出腿长、腿速及实际轴向力。对于ZYX欧拉角状态，世界角速度通过姿态速率映射转换为欧拉角速度，而不采用位姿序列数值差分。这样可避免差分噪声和一拍延迟进入CLF、CBF及腿加速度计算。主要信号契约如表4-3所示。")

    add_table_caption(doc, "表4-3  Simscape平台的主要输入输出信号契约")
    add_table(
        doc,
        ["信号", "维数/单位", "来源", "主要用途"],
        [
            ["q_r,q̇_r,q̈_r", "6维；m、rad及其导数", "Hermite参考重建", "计算力矩、跟踪误差、时变CLF"],
            ["L_r,L̇_r", "6维；m、m/s", "IK与Jacobian", "腿长参考、机构边界核对"],
            ["F_CT,F_nom,F_cmd", "6维；N", "计算力矩、LQI与QP", "区分前馈、名义力和最终命令力"],
            ["q,q̇", "6维；m、rad及其导数", "绝对/相对运动传感器", "反馈控制、CLF/CBF状态和动力学评价"],
            ["L,L̇,L̈", "6维；m、m/s、m/s²", "关节测量与规则网格计算", "腿长、腿速和腿加速度验收"],
            ["F_act", "6维；N", "Prismatic Joint力测量", "执行器误差、抗饱和回灌和实际输入评价"],
            ["h,ψ₁,残差与状态标志", "按约束组组织", "严格QP诊断", "可行性、回退、活动约束和安全验收"],
        ],
        [1500, 1900, 2450, 3102],
        [WD_ALIGN_PARAGRAPH.CENTER, WD_ALIGN_PARAGRAPH.CENTER, WD_ALIGN_PARAGRAPH.LEFT, WD_ALIGN_PARAGRAPH.LEFT],
        font_size=9.5,
    )

    add_heading(doc, "4.1.7 仿真时序、日志与逐点验收", 2)
    add_body(doc, "仿真执行不是直接打开.slx文件运行，而是由准备脚本依次完成参数构建、轨迹检查、参考重建、Simscape参数映射、控制器初始化、Variant选择、重力配置和求解器预热。完成上述步骤后再运行完整轨迹，并由评价程序统一解析位姿、支链运动、命令力、实际力和QP诊断。所有结果连同模型设置、轨迹来源和时间戳保存，保证后续表格和图形能够追溯到具体工况。")
    add_body(doc, "控制层以10 ms为离散周期，计算力矩-LQI和QP均在采样时刻读取同一状态快照；执行器命令在两个控制时刻之间保持不变，Simscape求解器连续积分刚体运动。对于严格QP，单步计算顺序为：读取当前位姿和速度，生成F_nom；计算动力学、腿长/腿速/腿加速度、碰撞和奇异性相关量；组装CLF与硬约束；求解并后验检查不等式残差；最后才把通过检查的支链力送入执行器。若求解失败或后验违反量超限，则明确记录回退与“安全未认证”状态，不能把回退轨迹当作满足CBF保证的结果。")
    add_body(doc, "本文不以“仿真正常结束”作为成功判据，而是对每个工况执行分层验收。数值层检查NaN、Inf、求解器错误和信号尺寸；模型层检查参数映射、初始腿长与初始状态；跟踪层检查位姿和腿长误差；机构层检查腿长、腿速、腿加速度和奇异性；执行器层检查命令力、实际力、力幅值和力变化率；安全层检查真实几何距离、屏障状态、硬约束残差、QP可行性及回退次数；实时性层统计QP与完整控制步的均值、95%分位和最大耗时。只有所有与本工况声明相对应的指标同时通过，结果才被记为有效。")
    add_body(doc, "为了检验平台对现实不确定性的承载能力，统一接口还允许在不修改控制器结构的前提下改变载荷质量、载荷质心和执行器时间常数。此类试验用于暴露计算力矩和名义CBF对模型匹配的依赖，但其通过或失败都必须保留。特别地，参数扰动下的单次仿真通过只能作为该工况的工程证据，不能替代有界不确定性条件下的鲁棒安全证明。")

    add_heading(doc, "4.1.8 当前模型的适用范围与后续扩展", 2)
    add_body(doc, "当前平台在控制算法研究所需的主要耦合和约束方面已经形成闭环：多体对象保留空间几何、组合刚体惯量和重力；支链输入保留轴向力、幅值限制与可配置动态；传感器链提供可实现的当前状态；参考与日志采用统一时间基准；安全评价同时读取机构状态和独立几何距离。因此，该平台适合用于比较计算力矩-LQI、CLF-QP和完整CLF-CBF-QP在同一Stewart对象上的跟踪性能、控制分配、约束活动与求解开销。")
    add_body(doc, "同时，模型边界必须在论文中明确列出，见表4-4。对于未建模因素，本文不作隐含的鲁棒性假设，也不以调大安全裕度代替模型说明。后续若开展样机对应研究，可在保持参考、控制和测量接口不变的条件下，逐步加入支链真实质量与柔性、关节摩擦和间隙、传动链电气动态、传感器噪声与偏置、通信时延以及接触模型，并重新进行参数辨识和全部验收。")

    add_table_caption(doc, "表4-4  当前Simscape模型的物理覆盖与结论边界")
    add_table(
        doc,
        ["建模对象", "当前处理", "能够支持的结论", "不能据此声称的内容"],
        [
            ["平台与载荷", "刚体几何、质量、质心、惯量和重力显式建模", "六自由度耦合、重力补偿和负载参数敏感性", "结构柔性与高频模态已被准确复现"],
            ["支链与关节", "理想UCU关节；杆件仅含数值正则质量", "几何闭链、腿长运动和轴向力传递", "真实杆件惯量、关节摩擦、间隙和磨损效应"],
            ["执行器", "理想力及带延迟、死区、速率、滞后、饱和的非理想力Variant", "宏观力通道非理想性和执行器余量分析", "完整电机、电源、PWM、丝杠和热过程均已进入主模型"],
            ["环境安全", "基于圆柱体与有向包围盒的独立距离/屏障计算", "接触前的间隙保持和CBF约束评价", "碰撞接触、冲击和摩擦接触动力学"],
            ["测量", "位姿、速度、腿长和关节力按当前仿真值输出", "因果闭环接口和变量定义一致性", "已覆盖传感器噪声、漂移、量化和丢包"],
            ["安全保证", "连续时间模型条件与10 ms采样仿真分别报告", "给定模型、初值、采样和可行性条件下的证据", "所有参数失配和硬件条件下的无条件安全"],
        ],
        [1550, 2650, 2450, 2302],
        [WD_ALIGN_PARAGRAPH.CENTER, WD_ALIGN_PARAGRAPH.LEFT, WD_ALIGN_PARAGRAPH.LEFT, WD_ALIGN_PARAGRAPH.LEFT],
        font_size=9.3,
    )

    add_heading(doc, "4.1.9 本节小结", 2)
    add_body(doc, "本节从后续控制研究需要回答的问题出发，完成了参考轨迹之后的Simscape平台设计。平台以统一参数源建立固定平台、六支链、动平台、刚性载荷和重力多体对象，通过理想/非理想执行器Variant兼顾算法上界与工程非理想分析；利用Hermite重建、逆运动学和Jacobian形成连续一致的参考接口；通过绝对/相对运动、腿长和实际支链力测量建立因果反馈链；最终以统一日志和逐点验收保证不同控制器比较的公平性与可追溯性。该平台为下一节计算力矩-LQI基线控制的设计和后续CLF-CBF-QP安全滤波提供共同被控对象，同时也限定了仿真结论只能在当前模型、参数和采样条件内成立。")

    settings = doc.settings.element
    update_fields = settings.find(qn("w:updateFields"))
    if update_fields is None:
        update_fields = OxmlElement("w:updateFields")
        settings.append(update_fields)
    update_fields.set(qn("w:val"), "true")

    doc.save(OUT)
    print(OUT)


if __name__ == "__main__":
    build_document()
