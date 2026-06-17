"""将 Stewart 完整仿真流程 Markdown 报告导出为排版规范的 Word 文档。"""

from __future__ import annotations

import re
from io import BytesIO
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "docs" / "controller_workflow" / "stewart_complete_simulation_workflow.md"
OUTPUT = ROOT / "docs" / "controller_workflow" / "stewart_complete_simulation_workflow.docx"

FONT_CN = "Microsoft YaHei"
FONT_LATIN = "Calibri"
FONT_MONO = "Consolas"
BLUE = RGBColor(46, 116, 181)
DARK_BLUE = RGBColor(31, 77, 120)
MUTED = RGBColor(95, 105, 115)
LIGHT_FILL = "F4F6F9"
TABLE_FILL = "E8EEF5"


def set_run_font(run, size=None, bold=None, italic=None, color=None, mono=False):
    name = FONT_MONO if mono else FONT_LATIN
    run.font.name = name
    run._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), name)
    run._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), name)
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), FONT_CN)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    if color is not None:
        run.font.color.rgb = color


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=100, start=120, bottom=100, end=120):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for key, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{key}"))
        if node is None:
            node = OxmlElement(f"w:{key}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = paragraph.add_run("Stewart Simulation Project · 技术报告")
    set_run_font(run, 9, color=MUTED)


def configure_document(doc):
    section = doc.sections[0]
    section.top_margin = Inches(0.82)
    section.bottom_margin = Inches(0.75)
    section.left_margin = Inches(0.88)
    section.right_margin = Inches(0.88)
    section.header_distance = Inches(0.35)
    section.footer_distance = Inches(0.35)

    styles = doc.styles
    normal = styles["Normal"]
    normal.font.name = FONT_LATIN
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), FONT_CN)
    normal.font.size = Pt(10.5)
    normal.paragraph_format.space_after = Pt(7)
    normal.paragraph_format.line_spacing = 1.28
    normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY

    for style_name, size, color, before, after in (
        ("Heading 1", 16, BLUE, 16, 8),
        ("Heading 2", 13, BLUE, 12, 6),
        ("Heading 3", 11.5, DARK_BLUE, 9, 4),
    ):
        style = styles[style_name]
        style.font.name = FONT_LATIN
        style._element.rPr.rFonts.set(qn("w:eastAsia"), FONT_CN)
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = color
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True

    caption = styles["Caption"]
    caption.font.name = FONT_LATIN
    caption._element.rPr.rFonts.set(qn("w:eastAsia"), FONT_CN)
    caption.font.size = Pt(9)
    caption.font.color.rgb = MUTED
    caption.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    caption.paragraph_format.space_before = Pt(2)
    caption.paragraph_format.space_after = Pt(8)
    caption.paragraph_format.line_spacing = 1.15

def add_cover(doc):
    for _ in range(6):
        doc.add_paragraph()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run("Stewart 平台完整仿真流程技术报告")
    set_run_font(run, 25, bold=True, color=DARK_BLUE)
    p.paragraph_format.space_after = Pt(14)

    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run("从 Simscape Multibody 理想基准到 PWM 物理执行器、灰箱 + NARX 与 UKF")
    set_run_font(run, 13, color=BLUE)
    p.paragraph_format.space_after = Pt(28)

    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run("技术报告 · 可复现仿真流程与结果验证")
    set_run_font(run, 10.5, color=MUTED)

    for _ in range(9):
        doc.add_paragraph()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run("MATLAB R2025b · Simulink · Simscape Multibody")
    set_run_font(run, 10, color=MUTED)
    doc.add_page_break()


def collect_main_headings(lines):
    headings = []
    for line in lines:
        match = re.match(r"^## (.+)$", line)
        if match and match.group(1).strip() != "摘要":
            headings.append(match.group(1).strip())
    return headings


def add_static_toc(doc, headings):
    p = doc.add_paragraph("目录", style="Heading 1")
    p.paragraph_format.page_break_before = False
    for idx, text in enumerate(headings, 1):
        p = doc.add_paragraph(style="List Number")
        p.paragraph_format.space_after = Pt(3)
        run = p.add_run(text)
        set_run_font(run, 10.5)
    doc.add_page_break()


def add_inline_runs(paragraph, text, base_size=10.5):
    pattern = re.compile(r"(\*\*.+?\*\*|`.+?`|\*[^*].+?\*)")
    position = 0
    for match in pattern.finditer(text):
        if match.start() > position:
            run = paragraph.add_run(text[position : match.start()])
            set_run_font(run, base_size)
        token = match.group(0)
        if token.startswith("**"):
            run = paragraph.add_run(token[2:-2])
            set_run_font(run, base_size, bold=True)
        elif token.startswith("`"):
            run = paragraph.add_run(token[1:-1])
            set_run_font(run, max(base_size - 0.5, 8.5), mono=True, color=DARK_BLUE)
        else:
            run = paragraph.add_run(token[1:-1])
            set_run_font(run, base_size, italic=True)
        position = match.end()
    if position < len(text):
        run = paragraph.add_run(text[position:])
        set_run_font(run, base_size)


def add_figure(doc, image_path, alt_text):
    path = SOURCE.parent / image_path
    paragraph = doc.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_before = Pt(5)
    paragraph.paragraph_format.space_after = Pt(2)
    paragraph.paragraph_format.keep_with_next = True

    with Image.open(path) as image:
        image = image.convert("RGB")
        image.thumbnail((1800, 1800), Image.Resampling.LANCZOS)
        width_px, height_px = image.size
        image_stream = BytesIO()
        image.save(image_stream, format="JPEG", quality=90, optimize=True)
        image_stream.seek(0)
    max_width, max_height = 6.45, 7.35
    ratio = min(max_width / width_px, max_height / height_px)
    width = width_px * ratio
    run = paragraph.add_run()
    run.add_picture(image_stream, width=Inches(width))
    drawing = run._element.xpath(".//wp:docPr")
    if drawing:
        drawing[0].set("descr", alt_text)


def add_formula(doc, lines):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(7)
    p.paragraph_format.keep_together = True
    p_pr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), LIGHT_FILL)
    p_pr.append(shd)
    for index, line in enumerate(lines):
        if index:
            p.add_run().add_break()
        run = p.add_run(line)
        set_run_font(run, 10, mono=True, color=DARK_BLUE)


def add_code_block(doc, lines):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.18)
    p.paragraph_format.right_indent = Inches(0.18)
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(8)
    p.paragraph_format.line_spacing = 1.1
    p.paragraph_format.keep_together = True
    p_pr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "F2F4F7")
    p_pr.append(shd)
    for index, line in enumerate(lines):
        if index:
            p.add_run().add_break()
        run = p.add_run(line or " ")
        set_run_font(run, 8.5, mono=True)


def add_markdown_table(doc, rows):
    parsed = []
    for row in rows:
        parsed.append([cell.strip() for cell in row.strip().strip("|").split("|")])
    if len(parsed) > 1 and all(re.fullmatch(r":?-{3,}:?", cell) for cell in parsed[1]):
        parsed.pop(1)
    columns = max(len(row) for row in parsed)
    table = doc.add_table(rows=len(parsed), cols=columns)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.style = "Table Grid"
    usable_width = 6.5
    column_width = usable_width / columns

    for row_index, row in enumerate(parsed):
        for col_index in range(columns):
            cell = table.cell(row_index, col_index)
            cell.width = Inches(column_width)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            set_cell_margins(cell)
            if row_index == 0:
                set_cell_shading(cell, TABLE_FILL)
            paragraph = cell.paragraphs[0]
            paragraph.alignment = (
                WD_ALIGN_PARAGRAPH.CENTER if columns > 2 or col_index > 0 else WD_ALIGN_PARAGRAPH.LEFT
            )
            paragraph.paragraph_format.space_after = Pt(0)
            add_inline_runs(paragraph, row[col_index] if col_index < len(row) else "", 8.7)
            if row_index == 0:
                for run in paragraph.runs:
                    run.bold = True
    doc.add_paragraph().paragraph_format.space_after = Pt(1)


def convert_markdown(doc, lines):
    index = 0
    in_code = False
    code_lines = []
    in_formula = False
    formula_lines = []
    first_heading = True

    while index < len(lines):
        raw = lines[index]
        line = raw.rstrip()

        if line.startswith("```"):
            if in_code:
                add_code_block(doc, code_lines)
                code_lines = []
                in_code = False
            else:
                in_code = True
            index += 1
            continue
        if in_code:
            code_lines.append(raw)
            index += 1
            continue

        if line.strip() == r"\[":
            in_formula = True
            formula_lines = []
            index += 1
            continue
        if in_formula:
            if line.strip() == r"\]":
                add_formula(doc, formula_lines)
                in_formula = False
            else:
                formula_lines.append(line)
            index += 1
            continue

        if line.startswith("|") and index + 1 < len(lines) and lines[index + 1].startswith("|"):
            table_lines = [line]
            index += 1
            while index < len(lines) and lines[index].startswith("|"):
                table_lines.append(lines[index])
                index += 1
            add_markdown_table(doc, table_lines)
            continue

        image = re.fullmatch(r"!\[(.*?)\]\((.*?)\)", line.strip())
        if image:
            add_figure(doc, image.group(2), image.group(1))
            index += 1
            continue

        heading = re.match(r"^(#{1,4})\s+(.+)$", line)
        if heading:
            level = len(heading.group(1))
            text = heading.group(2).strip()
            if level == 1 and first_heading:
                first_heading = False
                index += 1
                continue
            style_level = max(1, level - 1)
            p = doc.add_paragraph(style=f"Heading {min(style_level, 3)}")
            if level == 2 and text != "摘要":
                p.paragraph_format.page_break_before = True
            add_inline_runs(p, text, {1: 16, 2: 13, 3: 11.5}.get(style_level, 11))
            index += 1
            continue

        if re.fullmatch(r"-{3,}", line.strip()):
            index += 1
            continue

        if not line.strip():
            index += 1
            continue

        list_match = re.match(r"^\s*[-*]\s+(.+)$", line)
        number_match = re.match(r"^\s*\d+\.\s+(.+)$", line)
        quote_match = re.match(r"^>\s*(.+)$", line)
        if list_match or number_match:
            p = doc.add_paragraph(style="List Bullet" if list_match else "List Number")
            p.paragraph_format.space_after = Pt(4)
            add_inline_runs(p, (list_match or number_match).group(1))
        elif quote_match:
            p = doc.add_paragraph()
            p.paragraph_format.left_indent = Inches(0.3)
            p.paragraph_format.right_indent = Inches(0.25)
            p.paragraph_format.space_before = Pt(5)
            p.paragraph_format.space_after = Pt(8)
            p_pr = p._p.get_or_add_pPr()
            shd = OxmlElement("w:shd")
            shd.set(qn("w:fill"), "EEF4FA")
            p_pr.append(shd)
            add_inline_runs(p, quote_match.group(1), 10.5)
        elif line.startswith("**图"):
            p = doc.add_paragraph(style="Caption")
            add_inline_runs(p, line.replace("**", ""), 9)
        else:
            p = doc.add_paragraph()
            add_inline_runs(p, line)
        index += 1


def build_document():
    lines = SOURCE.read_text(encoding="utf-8").splitlines()
    doc = Document()
    configure_document(doc)
    add_cover(doc)
    add_static_toc(doc, collect_main_headings(lines))
    convert_markdown(doc, lines)
    configure_document(doc)
    doc.core_properties.title = "Stewart 平台完整仿真流程技术报告"
    doc.core_properties.subject = "Simscape、PWM 执行器、灰箱 + NARX 与 UKF 完整仿真流程"
    doc.core_properties.author = "Stewart Simulation Project"
    doc.save(OUTPUT)
    # 保存后重新写入空页眉页脚，消除部分 Word 版本中的默认模板关系残留。
    clean_doc = Document(OUTPUT)
    for section in clean_doc.sections:
        section.header.paragraphs[0].clear()
        section.footer.paragraphs[0].clear()
    clean_output = OUTPUT.with_suffix(".clean.docx")
    clean_doc.save(clean_output)
    clean_output.replace(OUTPUT)
    print(f"Word 报告已导出：{OUTPUT}")


if __name__ == "__main__":
    build_document()
