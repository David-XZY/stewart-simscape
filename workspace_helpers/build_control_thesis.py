from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from docx import Document
from docx.enum.section import WD_ORIENT, WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.enum.text import (
    WD_ALIGN_PARAGRAPH,
    WD_BREAK,
    WD_LINE_SPACING,
    WD_TAB_ALIGNMENT,
    WD_TAB_LEADER,
)
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


WORKSPACE = Path(r"D:\NPU\Matlab_code")
PROJECT = WORKSPACE / "stewart-simscape"
REFERENCE_DOCX = Path(r"D:\NPU\并联机器人\完善\第三章轨迹规划与第四章跟踪控制实现_论文正文稿.docx")
OUTPUT_DIR = Path(r"D:\NPU\并联机器人\完善")
OUTPUT_DOCX = OUTPUT_DIR / "并联机器人计算力矩_LQI与CLF_CBF_QP控制研究.docx"
SNAPSHOT_DOCX = OUTPUT_DIR / "并联机器人计算力矩_LQI与CLF_CBF_QP控制研究_理论初稿快照.docx"
WORK_DIR = WORKSPACE / ".thesis_control_work"
ASSET_DIR = WORK_DIR / "control_initial_assets"
EQUATION_JSON = WORK_DIR / "control_initial_equations.json"
FINALIZER_PS1 = WORK_DIR / "finalize_control_initial_word.ps1"

FRESH_RESULT = PROJECT / "results" / "controller" / "strict_qp_fresh_baseline_20260712.mat"
FRESH_TRACKING = ASSET_DIR / "fresh_baseline_tracking.png"
FRESH_LEG = ASSET_DIR / "fresh_baseline_leg_motion.png"
FRESH_FORCE = ASSET_DIR / "fresh_baseline_force.png"

FIG_ROOT = PROJECT / "docs" / "controller_workflow" / "figures" / "simulation_workflow"
PREVIOUS_ASSETS = OUTPUT_DIR / "_thesis_ch3_ch4_assets" / "figures"

PYTHON_BIN = Path(r"C:\Users\David_X\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe")
MATLAB_BIN = Path(r"D:\MATLAB\R2025b\bin\matlab.exe")

PAGE_WIDTH_DXA = 11906
PAGE_HEIGHT_DXA = 16838
TABLE_WIDTH_DXA = 8832
TABLE_INDENT_DXA = 120


FRESH_METRICS = {
    "length_rms_m": [1.2588e-4, 9.7224e-5, 2.3202e-4, 7.9431e-4, 8.5570e-4, 2.2786e-4],
    "length_peak_m": [2.4839e-4, 2.5818e-4, 6.5614e-4, 1.9e-3, 1.8e-3, 4.3488e-4],
    "pose_rms": [3.5224e-4, 3.1136e-4, 2.6969e-4, 1.9987e-4, 1.4e-3, 2.9994e-4],
    "pose_peak": [5.7035e-4, 7.87747e-4, 6.2498e-4, 3.8664e-4, 2.551079e-3, 6.2873e-4],
    "equivalent_pose_rms_m": 3.64086e-4,
    "equivalent_pose_peak_m": 1.3e-3,
    "max_length_error_m": 1.9e-3,
    "max_translation_error_m": 7.87747e-4,
    "max_rotation_error_rad": 2.551079e-3,
    "max_control_force_n": 1094.58991,
    "max_feedback_force_n": 4.1936,
    "max_feedforward_force_n": 1095.4,
    "max_actual_force_n": 1094.58991,
    "force_tracking_rms_n": 0.1555,
    "force_tracking_peak_n": 1.2509,
    "min_length_m": 1.0784,
    "max_length_m": 1.9039,
    "max_leg_speed_mps": 0.2785,
    "max_leg_accel_mps2": 0.165137,
    "passed": True,
}


EQUATIONS: dict[str, dict[str, str]] = {}


def ensure_dirs() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    ASSET_DIR.mkdir(parents=True, exist_ok=True)


def set_font(run, east_asia: str = "宋体", latin: str = "Times New Roman", size: float = 12,
             bold: bool | None = None, italic: bool | None = None) -> None:
    run.font.name = latin
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.rFonts
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.insert(0, rfonts)
    rfonts.set(qn("w:eastAsia"), east_asia)
    rfonts.set(qn("w:ascii"), latin)
    rfonts.set(qn("w:hAnsi"), latin)
    rfonts.set(qn("w:cs"), latin)
    run.font.size = Pt(size)
    run.font.color.rgb = RGBColor(0, 0, 0)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic


def set_east_asia_language(run) -> None:
    rpr = run._element.get_or_add_rPr()
    lang = rpr.find(qn("w:lang"))
    if lang is None:
        lang = OxmlElement("w:lang")
        rpr.append(lang)
    lang.set(qn("w:eastAsia"), "zh-CN")


def clear_body(doc: Document) -> None:
    body = doc._element.body
    for child in list(body):
        if child.tag != qn("w:sectPr"):
            body.remove(child)


def remove_all_runs(paragraph) -> None:
    for run in list(paragraph.runs):
        paragraph._p.remove(run._r)


def configure_section(section, *, cover: bool = False) -> None:
    section.orientation = WD_ORIENT.PORTRAIT
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.54)
    section.bottom_margin = Cm(2.54)
    section.left_margin = Cm(2.50)
    section.right_margin = Cm(2.50)
    section.header_distance = Cm(1.50)
    section.footer_distance = Cm(1.75)
    section.different_first_page_header_footer = cover


def configure_styles(doc: Document) -> None:
    normal = doc.styles["Normal"]
    normal.font.name = "Times New Roman"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    normal._element.rPr.rFonts.set(qn("w:ascii"), "Times New Roman")
    normal._element.rPr.rFonts.set(qn("w:hAnsi"), "Times New Roman")
    normal.font.size = Pt(12)
    normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    normal.paragraph_format.first_line_indent = Pt(24)
    normal.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(0)
    normal.paragraph_format.widow_control = True

    for name, size, before, after, align in [
        ("Heading 1", 16, 18, 12, WD_ALIGN_PARAGRAPH.CENTER),
        ("Heading 2", 14, 12, 6, WD_ALIGN_PARAGRAPH.LEFT),
        ("Heading 3", 12, 8, 4, WD_ALIGN_PARAGRAPH.LEFT),
    ]:
        style = doc.styles[name]
        style.font.name = "Times New Roman"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "黑体")
        style._element.rPr.rFonts.set(qn("w:ascii"), "Times New Roman")
        style._element.rPr.rFonts.set(qn("w:hAnsi"), "Times New Roman")
        style.font.size = Pt(size)
        style.font.bold = False
        style.font.color.rgb = RGBColor(0, 0, 0)
        pf = style.paragraph_format
        pf.alignment = align
        pf.first_line_indent = Pt(0)
        pf.left_indent = Pt(0)
        pf.space_before = Pt(before)
        pf.space_after = Pt(after)
        pf.line_spacing_rule = WD_LINE_SPACING.SINGLE
        pf.keep_with_next = True
        pf.keep_together = True

    if "Caption Thesis" not in [s.name for s in doc.styles]:
        caption = doc.styles.add_style("Caption Thesis", WD_STYLE_TYPE.PARAGRAPH)
    else:
        caption = doc.styles["Caption Thesis"]
    caption.font.name = "Times New Roman"
    caption._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    caption.font.size = Pt(10.5)
    caption.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    caption.paragraph_format.first_line_indent = Pt(0)
    caption.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    caption.paragraph_format.space_before = Pt(3)
    caption.paragraph_format.space_after = Pt(6)
    caption.paragraph_format.keep_together = True

    if "Reference Thesis" not in [s.name for s in doc.styles]:
        ref_style = doc.styles.add_style("Reference Thesis", WD_STYLE_TYPE.PARAGRAPH)
    else:
        ref_style = doc.styles["Reference Thesis"]
    ref_style.font.name = "Times New Roman"
    ref_style._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    ref_style.font.size = Pt(10.5)
    ref_style.paragraph_format.first_line_indent = Pt(-21)
    ref_style.paragraph_format.left_indent = Pt(21)
    ref_style.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    ref_style.paragraph_format.space_after = Pt(3)

    for name, left_indent in [("TOC 1", 0), ("TOC 2", 12), ("TOC 3", 24)]:
        if name not in [s.name for s in doc.styles]:
            toc_style = doc.styles.add_style(name, WD_STYLE_TYPE.PARAGRAPH)
        else:
            toc_style = doc.styles[name]
        toc_style.font.name = "Times New Roman"
        toc_style._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
        toc_style.font.size = Pt(10.5)
        toc_style.paragraph_format.left_indent = Pt(left_indent)
        toc_style.paragraph_format.first_line_indent = Pt(0)
        toc_style.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        toc_style.paragraph_format.space_before = Pt(0)
        toc_style.paragraph_format.space_after = Pt(0)


def set_update_fields(doc: Document) -> None:
    settings = doc.settings._element
    update = settings.find(qn("w:updateFields"))
    if update is None:
        update = OxmlElement("w:updateFields")
        settings.append(update)
    update.set(qn("w:val"), "true")


def set_page_numbering(section, fmt: str, start: int | None = None) -> None:
    sect_pr = section._sectPr
    pg_num = sect_pr.find(qn("w:pgNumType"))
    if pg_num is None:
        pg_num = OxmlElement("w:pgNumType")
        sect_pr.append(pg_num)
    pg_num.set(qn("w:fmt"), fmt)
    if start is not None:
        pg_num.set(qn("w:start"), str(start))


def add_page_field(paragraph) -> None:
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = " PAGE "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    text = OxmlElement("w:t")
    text.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, text, end])
    set_font(run, "宋体", "Times New Roman", 9)


def add_header_rule(paragraph) -> None:
    ppr = paragraph._p.get_or_add_pPr()
    p_bdr = ppr.find(qn("w:pBdr"))
    if p_bdr is None:
        p_bdr = OxmlElement("w:pBdr")
        ppr.append(p_bdr)
    bottom = p_bdr.find(qn("w:bottom"))
    if bottom is None:
        bottom = OxmlElement("w:bottom")
        p_bdr.append(bottom)
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), "4")
    bottom.set(qn("w:space"), "1")
    bottom.set(qn("w:color"), "000000")


def configure_header_footer(section, *, header_text: str = "", page_number: bool = True) -> None:
    section.header.is_linked_to_previous = False
    section.footer.is_linked_to_previous = False
    header = section.header.paragraphs[0]
    remove_all_runs(header)
    header.alignment = WD_ALIGN_PARAGRAPH.CENTER
    if header_text:
        run = header.add_run(header_text)
        set_font(run, "宋体", "Times New Roman", 9)
        add_header_rule(header)
    footer = section.footer.paragraphs[0]
    remove_all_runs(footer)
    if page_number:
        add_page_field(footer)


def add_run_segments(paragraph, segments: list[tuple[str, dict]]) -> None:
    for text, props in segments:
        run = paragraph.add_run(text)
        set_font(
            run,
            props.get("east_asia", "宋体"),
            props.get("latin", "Times New Roman"),
            props.get("size", 12),
            props.get("bold"),
            props.get("italic"),
        )
        set_east_asia_language(run)


def add_paragraph(doc: Document, text: str, *, first_line: bool = True,
                  align=WD_ALIGN_PARAGRAPH.JUSTIFY, keep_with_next: bool = False,
                  style: str | None = None) -> object:
    p = doc.add_paragraph(style=style)
    p.alignment = align
    p.paragraph_format.first_line_indent = Pt(24) if first_line and style is None else Pt(0)
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE if style is None else p.paragraph_format.line_spacing_rule
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.keep_with_next = keep_with_next
    pieces = re.split(r"(`[^`]+`)", text)
    for piece in pieces:
        if not piece:
            continue
        is_code = piece.startswith("`") and piece.endswith("`")
        content = piece[1:-1] if is_code else piece
        run = p.add_run(content)
        if is_code:
            set_font(run, "黑体", "Consolas", 10.5)
        else:
            set_font(run, "宋体", "Times New Roman", 12)
        set_east_asia_language(run)
    return p


def add_heading(doc: Document, text: str, level: int) -> object:
    p = doc.add_paragraph(style=f"Heading {level}")
    if level == 1:
        p.paragraph_format.page_break_before = True
    run = p.add_run(text)
    set_font(run, "黑体", "Times New Roman", {1: 16, 2: 14, 3: 12}[level], False)
    return p


def add_equation(doc: Document, number: str, linear: str) -> None:
    key = f"E{len(EQUATIONS) + 1:03d}"
    EQUATIONS[key] = {"number": number, "linear": linear}
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.keep_together = True
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    ppr = p._p.get_or_add_pPr()
    outline = ppr.find(qn("w:outlineLvl"))
    if outline is None:
        outline = OxmlElement("w:outlineLvl")
        ppr.append(outline)
    outline.set(qn("w:val"), "9")
    p.paragraph_format.tab_stops.add_tab_stop(Pt(226.8), WD_TAB_ALIGNMENT.CENTER, WD_TAB_LEADER.SPACES)
    p.paragraph_format.tab_stops.add_tab_stop(Pt(453.5), WD_TAB_ALIGNMENT.RIGHT, WD_TAB_LEADER.SPACES)
    lead = p.add_run("\t")
    set_font(lead, "宋体", "Cambria Math", 12)
    marker = p.add_run(f"[[EQ:{key}]]")
    set_font(marker, "宋体", "Cambria Math", 12)
    tail = p.add_run(f"\t{number}")
    set_font(tail, "宋体", "Times New Roman", 12)


def add_caption(doc: Document, text: str) -> object:
    p = doc.add_paragraph(style="Caption Thesis")
    run = p.add_run(text)
    set_font(run, "宋体", "Times New Roman", 10.5)
    return p


def set_image_alt(inline_shape, alt_text: str) -> None:
    doc_pr = inline_shape._inline.docPr
    doc_pr.set("descr", alt_text)
    doc_pr.set("title", alt_text[:100])


def add_figure(doc: Document, path: Path, caption: str, *, width_in: float = 5.8) -> None:
    if not path.exists():
        raise FileNotFoundError(path)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.keep_with_next = True
    shape = p.add_run().add_picture(str(path), width=Inches(width_in))
    set_image_alt(shape, caption)
    add_caption(doc, caption)


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_mar = tc_pr.find(qn("w:tcMar"))
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for side, value in {"top": top, "start": start, "bottom": bottom, "end": end}.items():
        node = tc_mar.find(qn(f"w:{side}"))
        if node is None:
            node = OxmlElement(f"w:{side}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_cell_border(cell, edge: str, val: str = "nil", size: int = 0) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    borders = tc_pr.find(qn("w:tcBorders"))
    if borders is None:
        borders = OxmlElement("w:tcBorders")
        tc_pr.append(borders)
    node = borders.find(qn(f"w:{edge}"))
    if node is None:
        node = OxmlElement(f"w:{edge}")
        borders.append(node)
    node.set(qn("w:val"), val)
    if size:
        node.set(qn("w:sz"), str(size))
        node.set(qn("w:color"), "000000")


def set_repeat_table_header(row) -> None:
    tr_pr = row._tr.get_or_add_trPr()
    marker = OxmlElement("w:tblHeader")
    marker.set(qn("w:val"), "true")
    tr_pr.append(marker)


def set_table_geometry(table, widths_dxa: list[int]) -> None:
    if sum(widths_dxa) != TABLE_WIDTH_DXA:
        raise ValueError(f"table width mismatch: {sum(widths_dxa)}")
    table.autofit = False
    tbl_pr = table._tbl.tblPr
    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(TABLE_WIDTH_DXA))
    tbl_w.set(qn("w:type"), "dxa")
    tbl_ind = tbl_pr.find(qn("w:tblInd"))
    if tbl_ind is None:
        tbl_ind = OxmlElement("w:tblInd")
        tbl_pr.append(tbl_ind)
    tbl_ind.set(qn("w:w"), str(TABLE_INDENT_DXA))
    tbl_ind.set(qn("w:type"), "dxa")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths_dxa:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(width))
        grid.append(col)
    for row in table.rows:
        for index, cell in enumerate(row.cells):
            tc_pr = cell._tc.get_or_add_tcPr()
            tc_w = tc_pr.find(qn("w:tcW"))
            if tc_w is None:
                tc_w = OxmlElement("w:tcW")
                tc_pr.append(tc_w)
            tc_w.set(qn("w:w"), str(widths_dxa[index]))
            tc_w.set(qn("w:type"), "dxa")


def set_cell_text(cell, text: str, *, bold: bool = False,
                  align=WD_ALIGN_PARAGRAPH.CENTER, size: float = 10.5) -> None:
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
    p = cell.paragraphs[0]
    p.alignment = align
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    p.paragraph_format.space_before = Pt(1)
    p.paragraph_format.space_after = Pt(1)
    for run in list(p.runs):
        p._p.remove(run._r)
    run = p.add_run(str(text))
    set_font(run, "宋体", "Times New Roman", size, bold)


def add_three_line_table(doc: Document, caption: str, headers: list[str], rows: list[list[str]],
                         widths_dxa: list[int], *, left_columns: set[int] | None = None,
                         font_size: float = 10.5) -> None:
    left_columns = left_columns or set()
    add_caption(doc, caption)
    table = doc.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, cell in enumerate(table.rows[0].cells):
        set_cell_text(cell, headers[i], bold=True, size=font_size)
    set_repeat_table_header(table.rows[0])
    for row_data in rows:
        row = table.add_row()
        for i, value in enumerate(row_data):
            align = WD_ALIGN_PARAGRAPH.LEFT if i in left_columns else WD_ALIGN_PARAGRAPH.CENTER
            set_cell_text(row.cells[i], value, align=align, size=font_size)
    set_table_geometry(table, widths_dxa)
    for row_index, row in enumerate(table.rows):
        for cell in row.cells:
            set_cell_margins(cell)
            for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
                set_cell_border(cell, edge)
            if row_index == 0:
                set_cell_border(cell, "top", "single", 12)
                set_cell_border(cell, "bottom", "single", 6)
            elif row_index == len(table.rows) - 1:
                set_cell_border(cell, "bottom", "single", 12)
    spacer = doc.add_paragraph()
    spacer.paragraph_format.first_line_indent = Pt(0)
    spacer.paragraph_format.space_after = Pt(3)


def add_bullet_list(doc: Document, items: list[str]) -> None:
    for item in items:
        p = doc.add_paragraph(style="List Bullet")
        p.paragraph_format.left_indent = Pt(24)
        p.paragraph_format.first_line_indent = Pt(-12)
        p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
        p.paragraph_format.space_after = Pt(0)
        run = p.add_run(item)
        set_font(run, "宋体", "Times New Roman", 12)


def add_toc_field(doc: Document) -> None:
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Pt(0)
    run = p.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = ' TOC \\o "1-1" \\h \\z \\u '
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    cached = OxmlElement("w:t")
    cached.text = "目录将在 Word 中自动更新"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, cached, end])
    set_font(run, "宋体", "Times New Roman", 12)


def add_cover(doc: Document) -> None:
    for _ in range(2):
        doc.add_paragraph()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    run = p.add_run("西 北 工 业 大 学")
    set_font(run, "黑体", "Times New Roman", 22, True)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    run = p.add_run("硕 士 学 位 论 文（控制章节独立稿）")
    set_font(run, "黑体", "Times New Roman", 18, False)
    for _ in range(3):
        doc.add_paragraph()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.line_spacing = Pt(32)
    run = p.add_run("面向载荷挂装的并联机器人\n计算力矩-LQI与约束QP控制研究")
    set_font(run, "黑体", "Times New Roman", 22, True)
    for _ in range(5):
        doc.add_paragraph()
    for label, value in [("作    者", "谢振宇"), ("培养单位", "西北工业大学"), ("研究方向", "并联机器人轨迹跟踪与约束控制")]:
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.first_line_indent = Pt(0)
        p.paragraph_format.space_after = Pt(8)
        add_run_segments(p, [(f"{label}：", {"east_asia": "宋体", "size": 14}),
                             (value, {"east_asia": "宋体", "size": 14})])
    for _ in range(3):
        doc.add_paragraph()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    run = p.add_run("二〇二六年七月")
    set_font(run, "宋体", "Times New Roman", 14)


def load_font(size: int, bold: bool = False):
    candidates = [
        Path(r"C:\Windows\Fonts\simhei.ttf") if bold else Path(r"C:\Windows\Fonts\simsun.ttc"),
        Path(r"C:\Windows\Fonts\msyhbd.ttc") if bold else Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\arial.ttf"),
    ]
    for path in candidates:
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def draw_centered_multiline(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], text: str,
                            font, fill="black", spacing=8) -> None:
    x0, y0, x1, y1 = box
    bbox = draw.multiline_textbbox((0, 0), text, font=font, spacing=spacing, align="center")
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.multiline_text(((x0 + x1 - width) / 2, (y0 + y1 - height) / 2), text,
                        font=font, fill=fill, spacing=spacing, align="center")


def arrow(draw: ImageDraw.ImageDraw, start: tuple[int, int], end: tuple[int, int], width=4) -> None:
    draw.line([start, end], fill="black", width=width)
    x2, y2 = end
    x1, y1 = start
    angle = math.atan2(y2 - y1, x2 - x1)
    length = 15
    wing = 0.55
    p1 = (x2 - length * math.cos(angle - wing), y2 - length * math.sin(angle - wing))
    p2 = (x2 - length * math.cos(angle + wing), y2 - length * math.sin(angle + wing))
    draw.polygon([end, p1, p2], fill="black")


def create_qp_schematics() -> tuple[Path, Path]:
    flow = ASSET_DIR / "preliminary_qp_flow.png"
    hierarchy = ASSET_DIR / "qp_objective_constraints.png"
    title_font = load_font(32, True)
    box_font = load_font(27, False)
    note_font = load_font(22, False)

    img = Image.new("RGB", (1800, 760), "white")
    d = ImageDraw.Draw(img)
    boxes = [
        (60, 245, 330, 435, "参考轨迹\nqᵣ, q̇ᵣ, q̈ᵣ"),
        (420, 210, 760, 470, "计算力矩 + LQI\n基线控制器\nF_nom"),
        (850, 165, 1230, 515, "在线约束 QP\n跟踪目标\n力/变化率/支链约束"),
        (1320, 245, 1590, 435, "非理想执行器\nF_act"),
        (1630, 245, 1770, 435, "平台"),
    ]
    d.text((900, 55), "约束 QP 作为基线总力指令的在线校正层", font=title_font, fill="black", anchor="mm")
    for x0, y0, x1, y1, text in boxes:
        d.rounded_rectangle((x0, y0, x1, y1), radius=18, outline="black", width=4, fill=(245, 245, 245))
        draw_centered_multiline(d, (x0, y0, x1, y1), text, box_font)
    for i in range(len(boxes) - 1):
        arrow(d, (boxes[i][2], (boxes[i][1] + boxes[i][3]) // 2),
              (boxes[i + 1][0], (boxes[i + 1][1] + boxes[i + 1][3]) // 2))
    d.rounded_rectangle((540, 600, 1210, 710), radius=15, outline="black", width=3, fill=(255, 255, 255))
    draw_centered_multiline(d, (540, 600, 1210, 710), "位姿、速度、腿长与执行器状态反馈", note_font)
    arrow(d, (1635, 435), (1210, 655), width=3)
    arrow(d, (850, 655), (760, 470), width=3)
    img.save(flow, dpi=(240, 240))

    img = Image.new("RGB", (1700, 920), "white")
    d = ImageDraw.Draw(img)
    d.text((850, 55), "单步约束 QP 的信息层次", font=title_font, fill="black", anchor="mm")
    center = (605, 300, 1095, 525)
    d.rounded_rectangle(center, radius=22, outline="black", width=5, fill=(235, 235, 235))
    draw_centered_multiline(d, center, "决策变量\n六维支链力 F\n及必要松弛量", title_font)
    satellites = [
        (70, 155, 470, 335, "跟踪层\n接近 F_nom\n限制控制偏离"),
        (1230, 155, 1630, 335, "执行器层\n力上下限\n力变化率"),
        (70, 590, 470, 790, "机构运动层\n腿长 / 腿速\n腿加速度"),
        (1230, 590, 1630, 790, "风险监测层\n奇异性指标\n碰撞距离"),
    ]
    for box in satellites:
        d.rounded_rectangle(box[:4], radius=18, outline="black", width=4, fill=(250, 250, 250))
        draw_centered_multiline(d, box[:4], box[4], box_font)
    arrow(d, (470, 245), (605, 330))
    arrow(d, (1230, 245), (1095, 330))
    arrow(d, (470, 690), (605, 490))
    arrow(d, (1230, 690), (1095, 490))
    d.text((850, 860), "初步实现中，风险监测量不等同于严格的前向安全约束。", font=note_font, fill="black", anchor="mm")
    img.save(hierarchy, dpi=(240, 240))
    return flow, hierarchy


def validate_reference() -> None:
    if not REFERENCE_DOCX.exists():
        raise FileNotFoundError(REFERENCE_DOCX)
    import hashlib
    digest = hashlib.sha256(REFERENCE_DOCX.read_bytes()).hexdigest().upper()
    expected = "89B22A5B7774D6033CBCF1288EAA82787F98E3E0B200E22E64E627FE27A71A0D"
    if digest != expected:
        raise RuntimeError(f"reference hash changed: {digest}")


def document_text(doc: Document) -> str:
    parts = [p.text for p in doc.paragraphs]
    for table in doc.tables:
        for row in table.rows:
            for cell in row.cells:
                parts.append(cell.text)
    for section in doc.sections:
        parts.extend(p.text for p in section.header.paragraphs)
        parts.extend(p.text for p in section.footer.paragraphs)
    return "\n".join(parts)


def add_front_title(doc: Document, text: str, *, english: bool = False,
                    page_break_before: bool = False) -> None:
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.space_before = Pt(12)
    p.paragraph_format.space_after = Pt(18)
    p.paragraph_format.page_break_before = page_break_before
    run = p.add_run(text)
    set_font(run, "黑体" if not english else "宋体", "Times New Roman", 16, True)


def add_keywords(doc: Document, label: str, words: str, *, english: bool = False) -> None:
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.space_before = Pt(12)
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
    r1 = p.add_run(label)
    set_font(r1, "黑体" if not english else "宋体", "Times New Roman", 12, True)
    r2 = p.add_run(words)
    set_font(r2, "宋体", "Times New Roman", 12)


def add_chapter_break(doc: Document) -> None:
    # Chapter headings carry page_break_before; a separate break paragraph can
    # be pushed onto a new page and create a blank sheet when the prior page is full.
    return


def build_initial_document() -> Document:
    validate_reference()
    create_qp_schematics()
    doc = Document(str(REFERENCE_DOCX))
    clear_body(doc)
    configure_styles(doc)
    set_update_fields(doc)
    for section in doc.sections:
        configure_section(section)

    cover_section = doc.sections[0]
    configure_section(cover_section, cover=True)
    configure_header_footer(cover_section, page_number=False)
    add_cover(doc)

    front_section = doc.add_section(WD_SECTION.NEW_PAGE)
    configure_section(front_section)
    configure_header_footer(front_section, page_number=True)
    set_page_numbering(front_section, "lowerRoman", 1)

    add_front_title(doc, "摘  要")
    add_paragraph(doc, "Stewart并联机器人具有承载能力强、结构刚度高和运动精度高等特点，适用于重载定位、装配对接与载荷挂装等任务。该类任务不仅要求动平台沿给定六维位姿轨迹稳定运行，而且要求六条支链驱动力、长度、速度与加速度始终处于可执行范围。由于平台位姿与支链运动之间存在显著耦合，单纯提高跟踪增益并不能同时解决模型误差、执行器非理想动态和多类物理边界之间的协调问题。针对上述特点，本文以已通过工程后验检查的两阶段参考轨迹为输入，研究计算力矩前馈、线性二次型积分反馈以及在线约束二次规划相结合的控制方法。")
    add_paragraph(doc, "首先，在统一坐标系和符号约定下建立Stewart平台逆运动学、速度映射与动力学模型，严格区分平台扭量雅可比和欧拉角广义坐标雅可比，从功率一致性出发给出支链轴向力到平台广义力的映射。其次，构造以逆动力学计算力矩为前馈、以LQI输出反馈为误差补偿的基线控制器。LQI部分采用增广积分状态、状态观测器、积分泄漏与执行器抗饱和回灌，使前馈项主要承担重力和参考运动载荷，反馈项主要修正模型失配与执行器跟踪误差。")
    add_paragraph(doc, "在基线基础上，本文进一步给出约束QP的设计动机和初步标准形式，将基线总力指令作为名义输入，在每个采样时刻协调驱动力幅值、力变化率及支链运动约束。初步方案中的碰撞距离和奇异性指标仅作为风险监测量，不被表述为已经建立严格前向不变性保证；其性能也不采用历史轻量级滚动数据进行论证，而留待精确输入仿射模型、严格约束导数和同一Simscape被控对象完成验证后补充。")
    add_paragraph(doc, "采用2026年7月12日重新运行并保存的Simscape非理想力执行器结果对基线方法进行验证。结果表明：仿真完整结束且综合验收通过，最大平移误差为0.788 mm，最大姿态误差为0.146°，等效位姿均方根误差为0.364 mm；支链长度、速度、加速度与驱动力均处于中期设定边界内，其中最大腿加速度为0.165 m/s²，显著低于1.2 m/s²。该结果说明基线在当前标称轨迹上具有良好跟踪能力，但固定控制律本身仍不能对扰动工况和边界邻域给出显式安全保证，这正是引入严格约束优化层的主要原因。")
    add_keywords(doc, "关键词：", "Stewart并联机器人；计算力矩；线性二次型积分控制；约束二次规划；Simscape")

    add_front_title(doc, "ABSTRACT", english=True, page_break_before=True)
    add_paragraph(doc, "The Stewart parallel robot is attractive for heavy-load positioning, docking, and payload installation because of its high stiffness, load capacity, and motion accuracy. These tasks require more than small pose-tracking errors. The axial forces, lengths, velocities, and accelerations of all six legs must remain executable throughout the motion. Owing to the strong coupling between the platform pose and leg coordinates, increasing a fixed feedback gain alone cannot coordinate model uncertainty, actuator dynamics, tracking quality, and multiple physical limits.", first_line=False)
    add_paragraph(doc, "This study first establishes a consistent kinematic and dynamic description of the Stewart platform. The twist Jacobian and the generalized-coordinate Jacobian associated with ZYX Euler angles are explicitly distinguished, and the force mapping is derived from power consistency. A baseline controller is then developed by combining inverse-dynamics computed-torque feedforward with linear-quadratic-integral output feedback. An observer, leaky integral action, and actuator anti-windup back-calculation are included so that the feedforward term compensates gravity and reference motion loads, while the feedback term rejects modeling and actuation errors.", first_line=False)
    add_paragraph(doc, "A constrained quadratic-programming layer is subsequently formulated as an online correction of the complete baseline force command. Its purpose is to reconcile force magnitude, force-rate, and leg-motion constraints while remaining close to the nominal computed-torque-plus-LQI command. At the initial-manuscript stage, collision clearance and Jacobian singularity are treated only as monitored risk indicators; no strict forward-invariance claim or unverified QP comparison is made. A rigorous extension will require exact input-affine dynamics, exact constraint derivatives, and validation on the same controlled plant.", first_line=False)
    add_paragraph(doc, "A fresh Simscape simulation with a nonideal force actuator, saved on July 12, 2026, is used as the baseline evidence. The full simulation passed its acceptance checks. The maximum translational and rotational errors were 0.788 mm and 0.146 degrees, respectively, and the equivalent pose RMSE was 0.364 mm. The maximum leg acceleration was 0.165 m/s²; the corresponding midterm limit was 1.2 m/s². The results demonstrate strong nominal tracking, while also motivating an explicit constraint mechanism for disturbances and operation near physical boundaries.", first_line=False)
    add_keywords(doc, "KEY WORDS: ", "Stewart parallel robot; computed torque; LQI control; constrained quadratic programming; Simscape", english=True)

    add_front_title(doc, "目  录", page_break_before=True)
    add_toc_field(doc)

    main_section = doc.add_section(WD_SECTION.NEW_PAGE)
    configure_section(main_section)
    configure_header_footer(main_section, header_text="并联机器人计算力矩-LQI与约束QP控制研究", page_number=True)
    set_page_numbering(main_section, "decimal", 1)

    # Chapter 1
    add_heading(doc, "第1章 绪论", 1)
    add_heading(doc, "1.1 研究背景与意义", 2)
    add_paragraph(doc, "并联机器人通过多条支链共同约束动平台，其驱动单元通常布置在支链方向上，具有闭环承力、误差不累积和较高结构刚度等特点。Stewart平台是六自由度并联机构的典型形式，自Stewart提出六自由度运动平台以来，已在飞行模拟、精密加工、天线指向、车辆试验和重载装配等领域得到广泛研究[1-4]。在载荷挂装任务中，平台需要携带大质量圆柱体沿受限空间完成接近和插入，几何避障、运动学可达性、驱动力能力与最终装配精度相互耦合，因而控制器必须同时面向轨迹跟踪和工程约束。")
    add_paragraph(doc, "传统工业控制往往将轨迹规划和闭环控制分开处理：规划器给出位姿或腿长参考，控制器再通过固定增益反馈降低误差。当参考轨迹远离机构边界、执行器裕度充分且负载参数准确时，这种分层方式具有结构清晰、实现成本低等优点；但当轨迹靠近腿长边界、支链速度或加速度上限时，固定控制律可能在减小位姿误差的同时产生不可执行的支链力。尤其在高惯量负载、执行器滞后和力饱和并存时，跟踪性能与物理边界不再能够分别设计。")
    add_paragraph(doc, "计算力矩控制利用模型逆动力学直接补偿参考运动所需广义力，是机器人非线性控制中的经典方法[5-6]。LQI在状态反馈中引入积分状态，能够改善恒值扰动和稳态偏差，并可通过观测器构成输出反馈实现[7-8]。二者结合后，计算力矩承担主要前馈负载，LQI负责误差修正，能够形成具有明确物理分工的基线。然而，计算力矩与LQI本质上仍是离线确定的固定控制律，力限幅、力变化率和机构状态边界通常只能在控制器之后被动截断，无法在统一决策中体现多约束优先级。")
    add_paragraph(doc, "二次规划能够在每个采样周期求解带线性等式或不等式约束的凸优化问题，其目标函数和约束矩阵具有明确解释，并可利用热启动满足较高更新频率[10,16]。若把计算力矩与LQI产生的总力指令作为名义解，则QP可以被理解为一个最小改动的约束校正器：当名义力满足所有约束时基本保持原控制行为；当某类约束趋于活动时，通过重新分配六条支链力避免简单逐通道饱和造成的耦合失真。这一思路为后续引入具有稳定性和安全性条件的严格CLF-CBF约束提供了统一接口[11,14]。")

    add_heading(doc, "1.2 国内外相关研究概述", 2)
    add_heading(doc, "1.2.1 并联机器人动力学与计算力矩控制", 3)
    add_paragraph(doc, "Stewart平台的控制研究以运动学映射、动力学建模和驱动力分配为基础。Fichter对平台几何与控制问题进行了系统分析[3]，Dasgupta等对Stewart平台的结构、运动学和动力学研究进行了综述[4]。由于六条支链共同作用于动平台，平台空间的期望合力和合力矩必须通过雅可比转置映射为轴向力。若忽略欧拉角速度与角速度之间的变换，容易在动力学方程中混用扭量雅可比和广义坐标雅可比，导致公式在功率意义上不一致。因此，规范的符号体系是后续控制推导可信的前提。")
    add_paragraph(doc, "计算力矩控制通常根据参考位姿、速度和加速度求取逆动力学广义力，再经雅可比转置反解支链驱动力。模型准确时，该方法可显著降低反馈控制器所需增益，并改善快速轨迹的相位滞后；模型失配时，负载质量、质心和摩擦参数误差会直接进入前馈项。工程实现中常采用伪逆或阻尼最小二乘处理雅可比条件数恶化，并在反解后加入驱动力限幅。上述处理提高了数值鲁棒性，但单纯限幅并不能保证限幅后的力仍满足原定闭环动态。")
    add_heading(doc, "1.2.2 积分最优控制与抗饱和", 3)
    add_paragraph(doc, "线性二次型控制通过最小化状态偏差和控制输入的二次型性能指标获得反馈增益，LQI进一步将输出误差积分并入增广状态，可消除常值扰动引起的稳态偏差[7-8]。对于不能直接测得全部线性化状态的对象，通常使用Luenberger观测器或Kalman滤波器重构状态。分离原理保证在理想线性条件下，状态反馈极点与观测误差极点可以分别设计；但当输入进入饱和，控制器内部模型和实际执行量出现差异，积分状态可能持续累积，形成积分饱和。")
    add_paragraph(doc, "抗饱和回灌利用实际执行量与未饱和命令之差修正积分状态，是工程中常用的积分恢复方法[9]。对于本研究的六腿力执行器，输入还包含延迟、死区、速率限制和一阶滞后，因而回灌量不能只依据静态饱和器输出，而应使用执行器实际力或其可信估计。抗饱和能够改善饱和后的恢复过程，却仍不能把腿长、腿速、碰撞距离和奇异性裕度转化为控制器可证明满足的约束。")
    add_heading(doc, "1.2.3 约束优化与安全关键控制", 3)
    add_paragraph(doc, "约束优化控制将控制输入作为在线决策变量，在跟踪目标之外显式考虑执行器和状态边界。模型预测控制通过有限时域预测协调未来行为，适合处理较长时间尺度的约束，但在线计算量随预测长度和非线性模型复杂度增加。单步QP安全滤波器则保留原有名义控制器，只对当前控制量进行最小修改，因而便于在已有控制架构上增量部署。")
    add_paragraph(doc, "控制Lyapunov函数可用于把收敛要求写为关于控制输入的不等式，控制障碍函数则通过集合前向不变性描述安全边界[11-12]。当安全函数相对系统输入具有高相对阶时，需要递归构造高阶约束[14]。需要强调的是，只有在输入仿射模型、Lie导数、初始可行性和QP持续可行等条件成立时，才能给出严格安全结论。仅把当前状态裕度附加到QP中，或使用与控制输入无关的零系数约束，只能充当风险监测或可行性筛查，不能替代严格障碍函数。")

    add_heading(doc, "1.3 研究内容与技术路线", 2)
    add_paragraph(doc, "本文围绕“可复现的基线控制—基线能力边界—在线约束改进”展开。首先，读取已经通过上游工程检查的两阶段位姿轨迹，利用分段三次Hermite方法在0.01 s规则网格上重建位姿、速度和加速度参考；其次，基于Stewart平台精确几何和动力学构造计算力矩前馈，以线性化力到位姿对象设计LQI输出反馈，并在非理想力执行器下闭环验证；最后，在基线总力之后设置QP接口，统一定义目标函数、约束类别、诊断量和验证指标。")
    add_three_line_table(
        doc,
        "表1-1  本文研究任务与证据层级",
        ["研究任务", "主要内容", "本稿证据状态"],
        [
            ["模型统一", "几何、Jacobian、动力学与单位约定", "公式与当前代码一致"],
            ["基线控制", "计算力矩前馈、LQI、观测器与抗饱和", "同一Simscape模型已验证"],
            ["约束QP", "以基线总力为名义输入的在线约束协调", "给出初步原理，不采用历史失真数据"],
            ["严格扩展", "精确导数、硬安全约束和同模型对比", "完成代码与仿真后追加"],
        ],
        [1800, 4300, 2732],
        left_columns={0, 1, 2},
    )
    add_paragraph(doc, "论文中的结论严格按照证据层级表述。已重新运行的Simscape基线可以用于报告真实跟踪和约束指标；尚未在同一被控对象上完成的QP仅用于方法定义，不能报告人为构造的误差衰减、缩放后的鲁棒性指标或与真实基线不具可比性的对比数据。该原则保证后续补充严格方法时，所有图表均能够回溯到明确的结果文件。")

    add_heading(doc, "1.4 论文结构", 2)
    add_paragraph(doc, "第1章说明研究背景、相关方法及本文证据边界。第2章建立Stewart平台运动学、Jacobian与动力学模型，并统一符号和单位。第3章推导计算力矩与LQI基线控制器，说明参考重建、状态观测和抗饱和实现。第4章从固定增益、输入饱和与多约束协调角度分析基线的结构性问题，提出QP改进接口。第5章给出初步约束QP的标准形式、实现边界和最新基线Simscape仿真结果。严格CLF-CBF-QP的精确推导、代码实现与同模型对比将在验证完成后附于同一文档后部。")

    add_chapter_break(doc)
    # Chapter 2
    add_heading(doc, "第2章 Stewart平台建模与控制问题描述", 1)
    add_heading(doc, "2.1 坐标系、广义坐标与符号约定", 2)
    add_paragraph(doc, "建立固定于基座的惯性坐标系{B}和固定于动平台的坐标系{P}。动平台中心在{B}中的位置记为p，姿态采用ZYX欧拉角，依次记为绕x、y、z轴的滚转角α、俯仰角β和偏航角γ。为避免正文与代码之间出现同一符号多种含义，本文统一用粗体小写表示向量、粗体大写表示矩阵或六维支链量，用ℓ_i表示第i条支链长度，用L表示六维腿长向量，用F表示六维支链轴向力。")
    add_equation(doc, "(2-1)", "q=[p^T,θ^T]^T,  θ=[α,β,γ]^T")
    add_paragraph(doc, "姿态旋转矩阵按ZYX顺序构造。该顺序与中期轨迹文件和当前`sgpIK`、`sgpJacobian`函数一致。欧拉角速度并不等于平台角速度，因此后续分别使用广义速度q̇和平台扭量ν，二者通过姿态相关变换矩阵T(q)联系。")
    add_equation(doc, "(2-2)", "R(θ)=R_z(γ)R_y(β)R_x(α)")
    add_figure(doc, FIG_ROOT / "02_stewart_geometry.png", "图2-1  Stewart平台坐标系、铰点与六条支链", width_in=5.85)
    add_paragraph(doc, "设基座第i个铰点在{B}中的位置为a_i，动平台对应铰点在{P}中的位置为b_i。动平台运动后，支链矢量、长度和单位方向分别为")
    add_equation(doc, "(2-3)", "r_i=p+R(θ)b_i-a_i,  ℓ_i=‖r_i‖_2,  s_i=r_i/ℓ_i")
    add_equation(doc, "(2-4)", "L=[ℓ_1,ℓ_2,ℓ_3,ℓ_4,ℓ_5,ℓ_6]^T")

    add_heading(doc, "2.2 速度映射与Jacobian矩阵", 2)
    add_paragraph(doc, "平台扭量由平移速度v和角速度ω组成。对式(2-3)求导并投影到支链单位方向，可得支链长度变化率与平台扭量之间的线性关系。若使用欧拉角广义速度，则需先通过T(q)把q̇转换为扭量。本文将这两个映射分别记为J_v和J_q，以保证动力学输入映射与代码接口前后一致。")
    add_equation(doc, "(2-5)", "ν=[v^T,ω^T]^T=T(q)q̇")
    add_equation(doc, "(2-6)", "L̇=J_v(q)ν=J_q(q)q̇,  J_q(q)=J_v(q)T(q)")
    add_paragraph(doc, "J_v的第i行由支链方向和动平台铰点相对平台中心的力臂构成。叉乘项的次序由虚功关系确定，必须与支链轴向力正方向保持一致。")
    add_equation(doc, "(2-7)", "J_(v,i)=[s_i^T,(Rb_i×s_i)^T]")
    add_paragraph(doc, "由瞬时功率守恒，支链功率F^T L̇应分别等于平台扭量形式和广义坐标形式下的输入功率。因此，平台空间力旋量W与欧拉角共轭广义力τ_q的虚功映射分别为J_v^TF和J_q^TF。当前动力学与自动微分代码以空间力旋量作为输入，故状态加速度方程统一使用J_v^TF；J_q用于腿空间速度和加速度映射。")
    add_equation(doc, "(2-8)", "F^T L̇=W^Tν=τ_q^Tq̇,  W=J_v^TF,  τ_q=J_q^TF")
    add_figure(doc, FIG_ROOT / "05_kinematics_dynamics_flow.png", "图2-2  运动学、Jacobian与动力学计算链", width_in=5.9)

    add_heading(doc, "2.3 平台动力学模型", 2)
    add_paragraph(doc, "在平台扭量坐标中，将动平台、载荷及等效支链惯性合并后，可把动力学写为质量矩阵、速度相关项和重力项之和。当前计算力矩函数首先根据参考状态求取所需六维合力/合力矩，再求解J_v^TF=W_r得到支链力，这与下式保持一致。")
    add_equation(doc, "(2-9)", "M(q)ν̇+C(q,ν)ν+g(q)=J_v(q)^TF")
    add_paragraph(doc, "按照当前`stateDynamics`及后续自动微分模型的统一接口，将质量矩阵、科氏/离心项、重力项以及姿态坐标变换项分别归入H(q)与h(q,q̇)，并保持输入端为J_v^TF，得到")
    add_equation(doc, "(2-10)", "H(q)q̈+h(q,q̇)=J_v(q)^TF")
    add_paragraph(doc, "在H(q)正定且J_v可计算的工作空间内，式(2-10)可整理成关于支链力的输入仿射加速度形式。该形式是后续把支链加速度、Lyapunov导数和安全约束写成QP线性不等式的基础。")
    add_equation(doc, "(2-11)", "q̈=A_F(q)F+b_F(q,q̇),  A_F=H^(-1)J_v^T,  b_F=-H^(-1)h")
    add_paragraph(doc, "对式(2-6)再次求导，可得腿加速度与支链力之间的仿射关系。这里的J̇_q q̇项不能在快速姿态运动或接近奇异位形时忽略，否则腿加速度约束会产生系统性偏差。")
    add_equation(doc, "(2-12)", "L̈=J_q q̈+J̇_q q̇=A_L(q)F+b_L(q,q̇)")
    add_equation(doc, "(2-13)", "A_L=J_qA_F,  b_L=J_qb_F+J̇_q q̇")

    add_heading(doc, "2.4 量纲归一化与奇异性指标", 2)
    add_paragraph(doc, "J_v的前三列对应平移速度，后三列对应角速度，两部分量纲不同，直接计算奇异值会受到单位选择影响。沿用中期设置，引入特征长度L_c=0.50 m对旋转分量归一化，形成无量纲Jacobian。后续所有奇异值阈值都应基于同一归一化矩阵定义。")
    add_equation(doc, "(2-14)", "J̄=J_v diag(I_3,L_c^(-1)I_3),  L_c=0.50 m")
    add_paragraph(doc, "同样，为把平移误差和姿态误差合并为单一评价指标，定义等效位姿误差e_eq。该指标把姿态误差乘以特征长度后转换为等效线位移，既避免直接混合米和弧度，也与当前评价函数中的`equivalentPoseError`定义一致。")
    add_equation(doc, "(2-15)", "e_(eq)=[e_p^T,L_c e_θ^T]^T")

    add_heading(doc, "2.5 参考轨迹与Simscape被控对象", 2)
    add_paragraph(doc, "控制仿真读取上游两阶段轨迹文件，其中包含节点时间、六维位姿、广义速度、支链力和腿长。控制准备函数首先验证轨迹已通过求解器和工程后验检查，再将节点数据重建到0.01 s规则网格。重建后的q_r、q̇_r、q̈_r用于计算力矩前馈，L_r和L̇_r用于结果检查，原轨迹驱动力保留为对照但不直接替代当前计算力矩。")
    add_figure(doc, FIG_ROOT / "11_reference_reconstruction.png", "图2-3  节点轨迹的Hermite重建及速度连续性", width_in=5.85)
    add_paragraph(doc, "被控对象采用Simscape Multibody搭建，包含Stewart平台刚体、六条可伸缩支链、载荷、重力和运动传感器。控制输入为六条支链的轴向力，输出包括相对位姿、绝对位姿、腿长、腿轴向力和控制器内部信号。该模型提供了比简化数值积分模型更完整的刚体耦合与执行器接口，是本文基线结果和后续QP对比必须共用的被控对象。")
    add_figure(doc, FIG_ROOT / "07_simscape_top_level.png", "图2-4  Simscape Stewart平台顶层被控对象", width_in=5.75)

    add_heading(doc, "2.6 控制约束与单位约定", 2)
    add_paragraph(doc, "为与中期工作保持一致，本文统一采用支链力±2000 N、腿长1.05～2.00 m、腿速0.60 m/s、腿加速度1.20 m/s²、特征长度0.50 m和采样周期0.01 s。除非在鲁棒性试验中明确改变负载参数，所有控制方法均使用相同参考轨迹、初始状态、重力设置和物理边界。")
    add_three_line_table(
        doc,
        "表2-1  中期约束与本文统一单位",
        ["物理量", "符号", "边界或取值", "单位"],
        [
            ["支链轴向力", "F_i", "-2000～2000", "N"],
            ["支链长度", "ℓ_i", "1.05～2.00", "m"],
            ["支链速度", "ℓ̇_i", "|ℓ̇_i|≤0.60", "m/s"],
            ["支链加速度", "ℓ̈_i", "|ℓ̈_i|≤1.20", "m/s²"],
            ["特征长度", "L_c", "0.50", "m"],
            ["控制采样周期", "T_s", "0.01", "s"],
        ],
        [2550, 1500, 3100, 1682],
        left_columns={0},
    )
    add_three_line_table(
        doc,
        "表2-2  主要符号及其代码对应关系",
        ["论文符号", "含义", "代码/结果字段"],
        [
            ["q, q̇, q̈", "平台广义位姿、速度、加速度", "refs.q, refs.qd, refs.qdd"],
            ["L, L̇, L̈", "六条支链长度、速度、加速度", "refs.L, report.legSpeed, report.legAcceleration"],
            ["J_v, J_q", "扭量与广义坐标Jacobian", "sgpJacobian.Jv, sgpJacobian.Jq"],
            ["F_CT", "计算力矩前馈支链力", "refs.Fcomputed / report.feedforwardForce"],
            ["F_LQI", "LQI反馈修正力", "report.feedbackForce"],
            ["F_nom", "基线名义总力", "report.controlForce进入约束层前的命令"],
            ["F_act", "执行器实际轴向力", "report.actualForce"],
        ],
        [1500, 3300, 4032],
        left_columns={1, 2},
        font_size=9.5,
    )
    add_paragraph(doc, "本章通过功率一致性统一了运动学和动力学符号，明确了J_v与J_q的适用方程，并给出了支链加速度关于驱动力的仿射表达。上述模型既服务于计算力矩基线，也为后续在线QP构造线性约束提供了统一接口。")

    add_chapter_break(doc)
    # Chapter 3
    add_heading(doc, "第3章 计算力矩与LQI基线控制方法", 1)
    add_heading(doc, "3.1 基线控制总体结构", 2)
    add_paragraph(doc, "基线控制器采用“模型前馈+动态误差反馈”的并联结构。参考轨迹经Hermite重建后，计算力矩模块根据q_r、q̇_r和q̈_r求取参考运动所需的支链轴向力F_CT；LQI模块根据实际位姿、观测状态和积分状态生成反馈修正力F_LQI；二者相加形成名义总力F_nom，再经非理想力执行器作用于Simscape平台。执行器实际力与原始命令之差回灌至积分器，避免执行器受限时积分状态继续累积。")
    add_figure(doc, PREVIOUS_ASSETS / "ch4_ct_lqi_nonideal_workflow.png", "图3-1  计算力矩、LQI与非理想力执行器基线结构", width_in=6.0)
    add_paragraph(doc, "该结构的核心是保持前馈与反馈职责可解释。F_CT只依据参考状态和模型参数计算，不把比例、微分误差人为混入逆动力学；F_LQI只对偏离参考轨迹的动态误差进行补偿。这样既便于单独检查模型前馈是否正确，也便于分析反馈力是否因模型失配、外扰或执行器滞后而显著增大。后续QP改进以二者相加后的F_nom为名义输入，而不是只处理F_CT。")

    add_heading(doc, "3.2 参考轨迹连续重建", 2)
    add_paragraph(doc, "上游轨迹由有限节点组成，而控制器和Simscape求解器需要连续参考。若直接对位姿节点进行分段线性插值，节点处速度不连续，进一步差分会产生不真实的加速度尖峰，计算力矩因此受到高频激励。本文在相邻节点[t_k,t_(k+1)]之间使用三次Hermite多项式，以节点位姿和速度作为端点条件。令τ=(t-t_k)/Δt_k，则")
    add_equation(doc, "(3-1)", "q_r(t)=h_(00)(τ)q_k+h_(10)(τ)Δt_k q̇_k+h_(01)(τ)q_(k+1)+h_(11)(τ)Δt_k q̇_(k+1)")
    add_equation(doc, "(3-2)", "h_(00)=2τ^3-3τ^2+1,  h_(10)=τ^3-2τ^2+τ")
    add_equation(doc, "(3-3)", "h_(01)=-2τ^3+3τ^2,  h_(11)=τ^3-τ^2")
    add_paragraph(doc, "对式(3-1)解析求导得到q̇_r和q̈_r，并统一采样到T_s=0.01 s的规则时间网格。腿长和腿速不通过独立插值获得，而是在每个规则采样点调用逆运动学和Jacobian重新计算，以避免位姿参考与腿空间参考之间因插值方式不同产生不一致。")

    add_heading(doc, "3.3 计算力矩前馈", 2)
    add_paragraph(doc, "在参考位姿处计算M、C、g和J_v，根据参考扭量及其导数构造平台所需合力/合力矩W_r。考虑到当前实现以平台空间Wrench作为中间变量，计算力矩方程采用第2章的扭量动力学形式。")
    add_equation(doc, "(3-4)", "W_r=M(q_r)ν̇_r+C(q_r,ν_r)ν_r+g(q_r)")
    add_paragraph(doc, "六条支链力通过J_v^T映射为W_r。对于非奇异的6×6矩阵，可直接求解线性方程；当矩阵互易条件数低于数值阈值时，代码改用Moore-Penrose伪逆。该分支防止直接求逆放大舍入误差，但并不意味着可以忽略接近奇异位形时的控制风险。")
    add_equation(doc, "(3-5)", "J_v(q_r)^T F_(CT)=W_r")
    add_equation(doc, "(3-6)", "F_(CT)={J_v(q_r)^(-T)W_r,  J_v^T可逆;  (J_v(q_r)^T)^†W_r,  其他}")
    add_paragraph(doc, "计算结果按执行器力上下限进行保护。由于限幅会破坏式(3-5)的精确等式，限幅状态必须被记录并由反馈和约束层感知。在最新标称仿真中，F_CT峰值约1095.4 N，低于±2000 N边界，说明当前参考轨迹具有较充分的名义力裕度。")

    add_heading(doc, "3.4 LQI增广控制器设计", 2)
    add_paragraph(doc, "LQI设计对象由Simscape力到位姿通道在工作点附近线性化得到。设线性化状态为x，LQI修正力为u=F_LQI，输出为位姿偏差y=Cx，且直接通道D=0，则局部对象写为")
    add_equation(doc, "(3-7)", "ẋ=Ax+Bu,  y=Cx")
    add_paragraph(doc, "定义位姿误差e_q=q-q_r，并引入六维积分状态x_I。当前实现中，增广积分状态等价于对输出误差取负积分，因而理想未饱和条件下ẋ_I=-e_q。将x和x_I组合为增广状态x_a，可得")
    add_equation(doc, "(3-8)", "x_a=[x^T,x_I^T]^T,  ẋ_I=-e_q")
    add_equation(doc, "(3-9)", "ẋ_a=[A,0;-C,0]x_a+[B;0]u")
    add_paragraph(doc, "LQI增益通过最小化无限时域二次性能指标获得。Q_a分别惩罚平移误差、姿态误差、动态状态和积分状态，R惩罚六条支链反馈力。姿态相关权重使用特征长度换算，以保持平移和旋转通道量纲一致。")
    add_equation(doc, "(3-10)", "J_(LQI)=∫_0^∞(x_a^TQ_a x_a+u^TRu)dt")
    add_equation(doc, "(3-11)", "F_(LQI)=-K_x x̂-K_I x_I")
    add_paragraph(doc, "代码中`lqi`函数返回的全状态增益按平台状态和积分状态分块为K_x与K_I。控制权重对应力尺度，位姿权重分别按平移、旋转和积分尺度构造。为使同一套线性化控制器适应当前非理想执行器，最终反馈通道还设置了平移增益缩放0.45、旋转增益缩放1.4和120 N反馈力保护。最新仿真中反馈力峰值仅4.19 N，说明标称轨迹主要由模型前馈承担。")

    add_heading(doc, "3.5 状态观测器、积分泄漏与抗饱和", 2)
    add_paragraph(doc, "Simscape可直接输出平台位姿，但LQI线性化状态不一定全部可测，因此采用对偶LQR形式设计观测器。设观测器增益为L_o，则其连续形式可写为")
    add_equation(doc, "(3-12)", "x̂̇=Ax̂+BF_(LQI)+L_o(e_q-Cx̂)")
    add_paragraph(doc, "在理想线性条件下，若A-BK_x与A-L_oC均为Hurwitz矩阵，则状态反馈与观测误差可以分别设计。当前代码还显式检查包含观测器和积分器的闭环极点，最新设计的最大极点实部为-1.00001，线性稳定性检查通过。需要注意，该结论只适用于线性化模型和未触发非线性约束的局部条件。")
    add_paragraph(doc, "为了抑制积分状态在低频测量偏差下无界漂移，引入泄漏率λ；为了处理执行器饱和、死区和速率限制，引入实际力F_act与原始命令F_raw之差的回灌项。积分器最终形式为")
    add_equation(doc, "(3-13)", "ẋ_I=-e_q-λx_I+K_(aw)(F_(act)-F_(raw))")
    add_paragraph(doc, "K_aw由积分增益的伪逆和抗饱和时间常数构造，其符号与ẋ_I=-e_q的积分约定一致。当执行器能够准确跟踪命令时，回灌项接近零；当命令被限幅或受速率限制时，回灌项推动积分状态向实际可执行控制方向恢复，从而减小解除饱和后的过冲和恢复滞后[9]。")

    add_heading(doc, "3.6 名义总力与非理想执行器", 2)
    add_paragraph(doc, "计算力矩与LQI修正相加形成基线名义总力。本文后续所有QP目标均以该总力为中心，确保QP改进继承基线的前馈精度和积分消差能力。")
    add_equation(doc, "(3-14)", "F_(nom)=F_(CT)+F_(LQI)")
    add_paragraph(doc, "非理想力执行器按延迟、死区、力变化率限制、一阶滞后和饱和的顺序处理F_nom。若把延迟与死区后的速率受限目标记为F_lim，则一阶滞后可写为")
    add_equation(doc, "(3-15)", "τ_F Ḟ_(act)=F_(lim)-F_(act),  F_(min)≤F_(act)≤F_(max)")
    add_paragraph(doc, "在轨迹开始前，执行器内部状态由第一拍计算力矩初始化，以建立抵消重力所需的静载力，避免从0 N起步导致平台初始下坠。该处理与实际系统在执行任务前先建立支撑压力或预载的工程过程相对应。")

    add_heading(doc, "3.7 闭环稳定性讨论", 2)
    add_paragraph(doc, "若计算力矩模型准确、J_v非奇异、执行器理想且LQI在线性化有效域内工作，则F_CT抵消参考动力学，剩余误差由稳定的LQI增广系统调节。对理想线性闭环状态x_cl，可由Lyapunov方程构造正定矩阵P_cl，使")
    add_equation(doc, "(3-16)", "A_(cl)^TP_(cl)+P_(cl)A_(cl)=-Q_(cl),  Q_(cl)>0")
    add_equation(doc, "(3-17)", "V=x_(cl)^TP_(cl)x_(cl),  V̇=-x_(cl)^TQ_(cl)x_(cl)<0")
    add_paragraph(doc, "上述推导说明基线在既定局部条件下具有渐近稳定性，但不能直接推出腿长、腿速、腿加速度、碰撞距离或奇异性集合的前向不变性。当输入被限幅、模型失配或状态接近约束边界时，闭环系统已不再等同于式(3-17)对应的理想线性系统。因此，基线稳定性与安全约束满足必须分别验证。")
    add_figure(doc, FIG_ROOT / "14_run02_controller_design.png", "图3-2  力到位姿对象线性化与LQI设计流程", width_in=5.85)
    add_three_line_table(
        doc,
        "表3-1  最新基线仿真控制配置",
        ["配置项", "取值", "说明"],
        [
            ["控制律", "computed-torque", "纯逆动力学前馈"],
            ["反馈律", "lqi-output", "位姿输出反馈与观测器"],
            ["执行器", "nonideal-force", "延迟、死区、速率、滞后和饱和"],
            ["带宽参数", "15 Hz", "线性化与控制设计参数"],
            ["平移增益缩放", "0.45", "平移反馈通道"],
            ["旋转增益缩放", "1.40", "姿态反馈通道"],
            ["采样周期", "0.01 s", "参考重建与后续QP周期"],
            ["重力", "开启", "由计算力矩承担主要补偿"],
        ],
        [2400, 2100, 4332],
        left_columns={0, 2},
    )
    add_paragraph(doc, "本章建立了计算力矩前馈与LQI输出反馈基线，给出了参考重建、逆动力学力分配、积分增广、状态观测、积分泄漏和抗饱和回灌的统一公式。该控制器在标称轨迹上能够提供高精度跟踪，但其控制律并未把多类安全边界纳入同一在线决策，下一章将针对这一结构性不足提出QP改进思路。")

    add_chapter_break(doc)
    # Chapter 4
    add_heading(doc, "第4章 基线控制的能力边界与QP改进思路", 1)
    add_heading(doc, "4.1 跟踪性能与约束满足的区别", 2)
    add_paragraph(doc, "控制器的跟踪误差小并不等价于物理约束得到保证。计算力矩和LQI的设计目标主要是使位姿误差收敛，其输出首先由模型和反馈增益确定，随后才由执行器限幅。设状态相关的可执行力集合为U(q,q̇)，则固定控制律产生的F_nom并不必然属于该集合。")
    add_equation(doc, "(4-1)", "U(q,q̇)={F∈R^6 | A_u(q,q̇)F≤b_u(q,q̇)}")
    add_equation(doc, "(4-2)", "F_(nom)=κ_(CT+LQI)(q,q̇,t)  ⇏  F_(nom)∈U(q,q̇)")
    add_paragraph(doc, "在最新标称仿真中，F_nom恰好具有充足裕度，因此各项边界均满足；但这种满足来自当前轨迹、负载和初始状态组合，而不是固定控制律对集合U的前向不变性证明。若负载质量增加、质心偏移、执行器时间常数增大或初始误差改变，名义力和支链运动可能更接近边界，单纯依赖仿真后验检查无法在控制执行前阻止违反。")

    add_heading(doc, "4.2 固定模型与固定增益的局限", 2)
    add_paragraph(doc, "计算力矩对质量、惯量、质心和重力方向等模型参数敏感。当模型参数与真实对象不一致时，前馈误差转化为LQI需要消除的扰动；若反馈力保护或执行器能力不足，误差会增加。LQI增益由单一线性化对象和预设权重得到，其闭环极点在工作点附近具有意义，但平台Jacobian、姿态变换和惯量映射均随位姿变化，固定增益不能自动反映不同位形下的控制分配效率。")
    add_paragraph(doc, "提高LQI权重或反馈增益可以在部分工况下降低误差，却可能增加驱动力峰值和力变化率，并放大测量噪声。降低增益则有利于执行器平滑，但可能减弱扰动抑制和积分恢复速度。因此，继续把调参作为主要改进手段，难以从方法层面解决多个约束之间的冲突。")

    add_heading(doc, "4.3 饱和与逐通道截断的问题", 2)
    add_paragraph(doc, "若计算力矩和LQI相加后超过执行器能力，最直接的处理是逐元素饱和。该操作只保证每条支链力不越过上下限，却没有利用六条支链共同产生平台Wrench的冗余协调关系。在接近奇异位形或某条支链变化率受限时，逐通道截断可能显著改变J_v^TF对应的合力和合力矩方向，使闭环行为偏离原设计。")
    add_equation(doc, "(4-3)", "F_(sat)=min(max(F_(nom),F_(min)),F_(max))")
    add_paragraph(doc, "抗饱和回灌能够修正积分器内部状态，但它发生在执行器命令已经被改变之后，且主要针对输入饱和，不会主动考虑腿长、腿速、腿加速度、碰撞和奇异性等状态约束。因而抗饱和与在线约束优化是互补关系，不能相互替代。")

    add_heading(doc, "4.4 多类约束的统一描述", 2)
    add_paragraph(doc, "对腿长、腿速、奇异性和碰撞距离分别定义安全裕度。腿长和腿速裕度可以直接由当前运动学状态计算；奇异性裕度基于无量纲Jacobian最小奇异值；碰撞裕度由载荷与障碍物几何距离减去安全阈值得到。")
    add_equation(doc, "(4-4)", "h_L=[L-L_(min);L_(max)-L],  h_(L̇)=[L̇_(max)-L̇;L̇-L̇_(min)]")
    add_equation(doc, "(4-5)", "h_σ=σ_(min)(J̄)-σ_(safe),  h_c=d_(collision)-d_(safe)")
    add_paragraph(doc, "仅要求当前时刻h≥0只能说明状态尚未越界，并不能保证控制输入会把下一时刻状态保持在安全集合内。要使约束对F产生实际作用，需要利用系统动力学构造h的导数或离散预测，使约束系数显式包含A_F或A_L。对于相对阶大于1的状态量，还需采用高阶递归条件。这也是初步QP与后续严格方法之间的关键区别。")

    add_heading(doc, "4.5 约束QP的设计定位", 2)
    add_paragraph(doc, "本文将QP定位为基线总力之后、执行器之前的最小修改层。输入包括当前位姿和速度、参考状态、基线名义力F_nom、上一时刻命令F_(k-1)、模型与场景参数；输出为满足当前QP约束的F_cmd以及求解状态、活动约束、松弛量和求解时间。")
    add_equation(doc, "(4-6)", "F_(cmd)=argmin_(F∈U_k) ‖F-F_(nom)‖_W^2")
    add_figure(doc, ASSET_DIR / "preliminary_qp_flow.png", "图4-1  约束QP作为基线总力在线校正层", width_in=6.05)
    add_paragraph(doc, "这种接口具有三个优点。第一，名义控制器保持独立，可单独验证计算力矩和LQI；第二，约束层不需要重新承担全部跟踪任务，只在必要时偏离名义力；第三，QP诊断量能够说明是哪一类约束导致控制分配改变，为调参与工程验收提供可追溯证据。")

    add_heading(doc, "4.6 初步QP的实现边界", 2)
    add_paragraph(doc, "工作区中的早期QP原型已经包含名义力跟踪、力变化率、CLF样式误差约束以及多类裕度字段，但其轻量级滚动过程不是与Simscape被控对象耦合的闭环动力学仿真；部分状态边界以与F无关的零系数行加入，只能检查当前裕度；碰撞距离由工作空间近似量代替真实几何距离；奇异性和碰撞约束默认关闭。因此，该原型不能被表述为严格安全控制器，也不能用其历史导出数据与最新Simscape基线进行性能对比。")
    add_paragraph(doc, "本初稿保留早期方法的合理设计动机和标准QP框架，同时明确撤回不具同模型证据的数据。后续改进必须完成三项工作：使用精确输入仿射动力学；使腿长、碰撞和奇异性条件通过精确导数显式依赖F；在理想力与非理想执行器两类Simscape模型中分别验证理论保证和工程鲁棒性。")
    add_three_line_table(
        doc,
        "表4-1  基线问题、QP作用与本稿结论边界",
        ["问题", "QP拟解决的机制", "当前可下结论"],
        [
            ["固定模型与固定增益", "以F_nom为中心在线重分配", "标称性能已验证，扰动保证待验证"],
            ["逐通道力饱和", "六维力联合优化与变化率约束", "原理成立，需同模型对比"],
            ["腿长/腿速/腿加速度", "输入仿射运动约束", "基线本次满足；QP保证待实现"],
            ["奇异性与碰撞", "精确高相对阶安全条件", "初步原型仅监测，不作严格声明"],
        ],
        [2350, 3300, 3182],
        left_columns={0, 1, 2},
        font_size=9.5,
    )
    add_paragraph(doc, "本章从控制结构而非单次仿真失败出发说明了引入QP的必要性。最新基线结果证明固定律在当前标称任务上表现良好；QP的价值在于为边界工况和扰动条件提供显式的多约束协调接口，而不是人为制造基线失败。")

    add_chapter_break(doc)
    # Chapter 5
    add_heading(doc, "第5章 初步约束QP原理与基线仿真验证", 1)
    add_heading(doc, "5.1 单步QP决策变量与目标函数", 2)
    add_paragraph(doc, "在第k个采样时刻，以六维支链力F为主要决策变量。为防止跟踪要求和安全约束冲突导致问题无解，初步形式可对收敛约束设置非负松弛量δ_V；对于尚未建立严格前向安全条件的状态裕度，不在本稿中使用“硬安全保证”表述。决策向量写为")
    add_equation(doc, "(5-1)", "z=[F^T,δ_V,δ_s^T]^T,  δ_V≥0,  δ_s≥0")
    add_paragraph(doc, "目标函数同时惩罚对基线总力的偏离、相邻控制周期的力变化以及松弛量。W_F决定QP保持名义控制行为的程度，W_Δ抑制驱动力跳变，ρ_V和ρ_s分别对应跟踪与初步状态约束松弛的代价。")
    add_equation(doc, "(5-2)", "min_z  1/2‖F-F_(nom)‖_(W_F)^2+1/2‖F-F_(k-1)‖_(W_Δ)^2+ρ_V δ_V^2/2+ρ_s‖δ_s‖_2^2/2")
    add_paragraph(doc, "展开二次项后，单步问题可写成标准凸QP形式。只要G为半正定矩阵、约束关于z为仿射，便可使用`quadprog`等成熟求解器。为了获得唯一且数值稳定的力解，通常令W_F+W_Δ正定。")
    add_equation(doc, "(5-3)", "min_z  1/2 z^TGz+g^Tz,  s.t.  Az≤b,  z_(min)≤z≤z_(max)")
    add_figure(doc, ASSET_DIR / "qp_objective_constraints.png", "图5-1  初步约束QP的目标与约束层次", width_in=5.95)

    add_heading(doc, "5.2 执行器硬约束", 2)
    add_paragraph(doc, "支链力上下限直接构成变量边界。为与中期设置一致，六条支链均采用±2000 N边界。力变化率根据上一时刻命令和采样周期写成线性不等式，可避免QP在相邻采样周期产生执行器无法跟随的跳变。")
    add_equation(doc, "(5-4)", "F_(min)≤F≤F_(max)")
    add_equation(doc, "(5-5)", "-Ḟ_(max)T_s≤F-F_(k-1)≤Ḟ_(max)T_s")
    add_paragraph(doc, "当驱动力边界和力变化率边界同时存在时，当前周期可行上下限应取二者交集。若交集为空，说明上一时刻命令、变化率或静态力边界配置不一致，应将该状态记录为不可行，而不能通过隐式放宽硬边界掩盖。")

    add_heading(doc, "5.3 支链运动约束", 2)
    add_paragraph(doc, "利用第2章式(2-12)，腿加速度约束可直接写为关于F的仿射不等式。该约束不需要有限差分估计未来腿加速度，能够在模型准确条件下直接限制当前控制输入。")
    add_equation(doc, "(5-6)", "-L̈_(max)≤A_L(q)F+b_L(q,q̇)≤L̈_(max)")
    add_paragraph(doc, "腿速和腿长对F的相对阶更高。作为离散初步实现，可以用常加速度一步预测形成线性约束，其中L̈由式(2-12)给出。预测仅覆盖一个采样周期，因此它是数值近似约束，不等价于连续时间前向不变性证明。")
    add_equation(doc, "(5-7)", "L̇_(k+1)=L̇_k+T_s(A_LF+b_L)")
    add_equation(doc, "(5-8)", "L_(k+1)=L_k+T_sL̇_k+T_s^2(A_LF+b_L)/2")
    add_equation(doc, "(5-9)", "L_(min)≤L_(k+1)≤L_(max),  -L̇_(max)≤L̇_(k+1)≤L̇_(max)")
    add_paragraph(doc, "初步QP若只把当前腿长或腿速裕度写入b而令对应A行全零，则控制输入无法改变该不等式，只能在状态已越界时触发不可行。因此，后续严格实现必须使用式(5-7)至式(5-9)或连续时间高阶约束，不能把零系数状态检查称为控制屏障。")

    add_heading(doc, "5.4 跟踪收敛约束", 2)
    add_paragraph(doc, "定义位姿误差e=q-q_r、速度误差ė=q̇-q̇_r，并组合为误差状态ζ。选择正定矩阵P构造二次Lyapunov候选函数V。")
    add_equation(doc, "(5-10)", "ζ=[e^T,ė^T]^T,  V(ζ)=ζ^TPζ,  P=P^T>0")
    add_paragraph(doc, "由输入仿射动力学可将V̇写成关于F的仿射函数。为保持QP在安全约束活动时可行，收敛约束允许唯一松弛量δ_V。")
    add_equation(doc, "(5-11)", "L_fV+L_gV F+∂V/∂t≤-c_VV+δ_V")
    add_paragraph(doc, "早期原型采用ė^T(q̈-q̈_des)形式近似构造收敛不等式，其中q̈_des=q̈_r-K_dė-K_pe。该形式能够表达期望二阶误差动态，但若质量矩阵采用对角等效近似，则其约束只能视为工程近似。后续严格版本应使用与第2章精确动力学一致的L_fV和L_gV。")
    add_equation(doc, "(5-12)", "q̈_(des)=q̈_r-K_dė-K_pe")

    add_heading(doc, "5.5 奇异性与碰撞风险监测", 2)
    add_paragraph(doc, "初步实现计算归一化Jacobian最小奇异值和场景距离裕度，用于标记接近奇异位形或障碍物的风险。监测量可参与日志、图表和离线调参，但若没有计算它们沿系统动力学的精确导数，就不能形成关于F的有效安全约束。")
    add_equation(doc, "(5-13)", "m_σ=σ_(min)(J̄)-σ_(safe),  m_c=d_(geom)-d_(safe)")
    add_paragraph(doc, "因此，本稿不报告初步QP对奇异性和碰撞的“保证”，也不使用以工作空间腿长裕度替代真实几何距离的结果。严格方法需要对顶板和两侧裙板分别建立可验证的几何分离证书，并对奇异性指标构造具有正确相对阶的约束。")

    add_heading(doc, "5.6 求解、回退与诊断", 2)
    add_paragraph(doc, "QP求解器应以上一时刻最优解热启动，并记录退出标志、约束残差、活动约束、求解时间和松弛量。若求解器不可用或问题不可行，工程系统不能直接输出NaN或中断仿真，可采用限幅和变化率投影形成回退命令。")
    add_equation(doc, "(5-14)", "F_(fb)=Π_[F_min,F_max]∩[F_(k-1)±Ḟ_maxT_s](ηF_(nom)+(1-η)F_(k-1))")
    add_paragraph(doc, "回退策略只保证投影中显式包含的力幅值和力变化率，不自动保证腿长、碰撞和奇异性安全。论文必须把回退次数单独报告；若主要试验中发生回退，就不能声称QP持续可行或硬安全条件全程成立。")
    add_three_line_table(
        doc,
        "表5-1  初步QP各类约束的证据属性",
        ["约束类别", "关于F的形式", "本稿属性"],
        [
            ["支链力", "变量上下界", "硬约束，可直接验证"],
            ["力变化率", "基于上一命令的仿射约束", "硬约束，可直接验证"],
            ["腿加速度", "A_LF+b_L边界", "精确模型下为硬仿射约束"],
            ["一步腿速/腿长", "离散预测仿射约束", "近似约束，需采样敏感性检查"],
            ["跟踪收敛", "Lyapunov导数+单一松弛", "软约束"],
            ["奇异性/碰撞", "当前裕度监测", "初步阶段不作严格安全声明"],
        ],
        [2300, 3400, 3132],
        left_columns={0, 1, 2},
        font_size=9.5,
    )

    add_heading(doc, "5.7 最新基线Simscape仿真设置", 2)
    add_paragraph(doc, "为避免沿用历史摘要中已经被后续实现修正的结果，本文使用2026年7月12日重新运行并保存的`strict_qp_fresh_baseline_20260712.mat`作为基线证据。该结果由当前工作区的计算力矩、LQI输出反馈和非理想力执行器在完整两阶段参考轨迹上生成，包含完整的位姿、腿长、速度、加速度、指令力和实际力时间序列。")
    add_paragraph(doc, "仿真采用重力开启、15 Hz带宽参数、0.45平移增益缩放、1.4旋转增益缩放和0.01 s参考重建周期。评价函数分别检查信号有限性、线性设计稳定性、腿长、驱动力、腿速、腿加速度和跟踪误差。尽管非理想执行器配置中的通用加速度验收阈值为10 m/s²，本文仍额外按中期1.2 m/s²边界复核真实时间序列。")
    add_three_line_table(
        doc,
        "表5-2  最新基线仿真条件与结果来源",
        ["项目", "设置或来源"],
        [
            ["被控对象", "Simscape Multibody Stewart平台+载荷+非理想力执行器"],
            ["参考轨迹", "ihsid_40x20_limited_memory/simscape_references.mat"],
            ["控制器", "计算力矩前馈+LQI输出反馈+抗饱和"],
            ["结果文件", "results/controller/strict_qp_fresh_baseline_20260712.mat"],
            ["主要边界", "±2000 N；1.05～2.00 m；0.60 m/s；1.20 m/s²"],
            ["综合验收", "passed=1，所有记录子项通过"],
        ],
        [2500, 6332],
        left_columns={0, 1},
        font_size=9.5,
    )

    add_heading(doc, "5.8 基线跟踪与约束结果", 2)
    add_paragraph(doc, "图5-2给出了六维位姿和六腿长度的真实跟踪误差。平移误差全程小于0.8 mm，最大值出现在y方向；姿态误差最大分量为β，其峰值约0.146°；腿长误差峰值约1.9 mm。误差曲线在轨迹阶段转换处保持连续，没有出现旧结果中的明显初始下坠或大幅腿长瞬态。")
    add_figure(doc, FRESH_TRACKING, "图5-2  最新计算力矩-LQI基线位姿与腿长跟踪误差", width_in=4.50)
    add_three_line_table(
        doc,
        "表5-3  最新基线总体跟踪指标",
        ["指标", "数值", "单位"],
        [
            ["等效位姿RMSE", f"{FRESH_METRICS['equivalent_pose_rms_m']*1000:.3f}", "mm"],
            ["等效位姿峰值", f"{FRESH_METRICS['equivalent_pose_peak_m']*1000:.3f}", "mm"],
            ["最大平移误差", f"{FRESH_METRICS['max_translation_error_m']*1000:.3f}", "mm"],
            ["最大姿态误差", f"{math.degrees(FRESH_METRICS['max_rotation_error_rad']):.3f}", "°"],
            ["最大腿长误差", f"{FRESH_METRICS['max_length_error_m']*1000:.3f}", "mm"],
            ["最大反馈修正力", f"{FRESH_METRICS['max_feedback_force_n']:.3f}", "N"],
        ],
        [4000, 2600, 2232],
        left_columns={0},
    )
    add_paragraph(doc, "为说明六个通道的误差分布，表5-4分别列出平移和姿态分量的均方根与峰值。姿态误差由弧度转换为角度后报告，表中不混用两种单位。")
    add_three_line_table(
        doc,
        "表5-4  六维位姿分量误差",
        ["通道", "RMSE", "峰值", "单位"],
        [
            ["x", f"{FRESH_METRICS['pose_rms'][0]*1000:.3f}", f"{FRESH_METRICS['pose_peak'][0]*1000:.3f}", "mm"],
            ["y", f"{FRESH_METRICS['pose_rms'][1]*1000:.3f}", f"{FRESH_METRICS['pose_peak'][1]*1000:.3f}", "mm"],
            ["z", f"{FRESH_METRICS['pose_rms'][2]*1000:.3f}", f"{FRESH_METRICS['pose_peak'][2]*1000:.3f}", "mm"],
            ["α", f"{math.degrees(FRESH_METRICS['pose_rms'][3]):.3f}", f"{math.degrees(FRESH_METRICS['pose_peak'][3]):.3f}", "°"],
            ["β", f"{math.degrees(FRESH_METRICS['pose_rms'][4]):.3f}", f"{math.degrees(FRESH_METRICS['pose_peak'][4]):.3f}", "°"],
            ["γ", f"{math.degrees(FRESH_METRICS['pose_rms'][5]):.3f}", f"{math.degrees(FRESH_METRICS['pose_peak'][5]):.3f}", "°"],
        ],
        [1800, 2500, 2500, 2032],
        left_columns={0},
    )
    add_paragraph(doc, "图5-3给出了六条支链速度与加速度。最大腿速为0.279 m/s，低于0.60 m/s；最大腿加速度为0.165 m/s²，低于中期1.20 m/s²边界。由此可见，当前标称基线并未发生腿加速度超限，QP改进的动机应表述为在扰动和边界工况下建立显式保证，而不是修正一个本次并不存在的违反。")
    add_figure(doc, FRESH_LEG, "图5-3  最新基线六支链速度、加速度与中期边界", width_in=5.85)
    add_paragraph(doc, "驱动力结果显示，六条支链总驱动力峰值为1094.59 N，低于±2000 N；指令力与实际力的RMS差为0.156 N，峰值为1.251 N，说明当前非理想执行器设置能够较好跟随平滑的前馈力。")
    add_figure(doc, FRESH_FORCE, "图5-4  最新基线六支链驱动力与执行器力跟踪误差", width_in=5.85)
    add_three_line_table(
        doc,
        "表5-5  最新基线物理约束复核",
        ["约束量", "实测极值", "中期边界", "结论"],
        [
            ["绝对腿长", f"{FRESH_METRICS['min_length_m']:.4f}～{FRESH_METRICS['max_length_m']:.4f} m", "1.05～2.00 m", "满足"],
            ["腿速度", f"{FRESH_METRICS['max_leg_speed_mps']:.4f} m/s", "≤0.60 m/s", "满足"],
            ["腿加速度", f"{FRESH_METRICS['max_leg_accel_mps2']:.4f} m/s²", "≤1.20 m/s²", "满足"],
            ["支链总驱动力", f"{FRESH_METRICS['max_control_force_n']:.2f} N", "≤2000 N", "满足"],
            ["信号有限性", "全部有限", "无NaN/Inf", "满足"],
            ["综合验收", "passed=1", "全部子项通过", "通过"],
        ],
        [2600, 2450, 2300, 1482],
        left_columns={0},
        font_size=9.5,
    )

    add_heading(doc, "5.9 结果讨论与后续严格验证要求", 2)
    add_paragraph(doc, "最新结果表明，计算力矩能够准确承担重力和轨迹运动所需载荷，LQI只需较小反馈修正即可将平移、姿态和腿长误差控制在较小范围，且所有中期物理边界均满足。这一结果为后续QP提供了质量较高的名义控制器：在约束不活动时，QP应尽量复现当前基线性能，而不应为了显示改进而无依据地改变误差数据。")
    add_paragraph(doc, "另一方面，单次标称仿真不能证明负载质量变化、质心偏移、执行器滞后增大或接近几何边界时仍满足约束。QP验证必须与基线使用同一Simscape模型、参考轨迹和初始状态，并至少包含理想力安全验证、非理想执行器工程验证、收敛约束消融、完整安全约束消融和参数扰动工况。所有指标应从结果文件自动读取。")
    add_paragraph(doc, "严格方法的验收标准应包括：QP持续可行且无隐藏回退；支链力、腿长、腿速和腿加速度全程满足边界；奇异值和真实几何间隙满足预设阈值；CLF松弛量、活动约束和求解时间完整记录；主要试验的95%单步求解时间不超过采样周期。若任一条件未满足，应如实报告局限，不允许对仿真误差进行人工缩放或预设衰减。")
    add_equation(doc, "(5-15)", "RMSE_p=√((1/N)Σ_(k=1)^N ‖e_p(k)‖_2^2)")
    add_equation(doc, "(5-16)", "RMSE_(eq)=√((1/(6N))Σ_(k=1)^N ‖e_(eq)(k)‖_2^2)")
    add_equation(doc, "(5-17)", "E_F=∫_0^T F(t)^TF(t)dt")
    add_paragraph(doc, "本章给出了初步约束QP的决策变量、目标函数、执行器约束、支链运动约束、跟踪收敛条件和回退诊断，并基于最新真实Simscape结果完成计算力矩-LQI基线验证。现阶段不报告未经同模型验证的QP性能数据；严格方法完成后，应在同一文档后续部分补充精确推导、代码实现、消融和鲁棒性结果。")

    add_chapter_break(doc)
    add_heading(doc, "结论与本阶段工作界限", 1)
    add_paragraph(doc, "本文围绕Stewart并联机器人载荷挂装轨迹的闭环执行问题，建立了从六维位姿参考到六腿轴向力的统一控制描述。通过严格区分扭量雅可比J_v与广义坐标雅可比J_q，给出了运动学速度映射、功率一致的力映射和支链加速度仿射模型，保证论文公式、代码变量和单位约定前后一致。")
    add_paragraph(doc, "构建的基线控制器以逆动力学计算力矩作为前馈，以LQI输出反馈作为动态误差补偿，并包括状态观测器、积分泄漏和执行器抗饱和回灌。2026年7月12日重新运行的Simscape结果显示，完整仿真综合验收通过，最大平移误差0.788 mm、最大姿态误差0.146°、等效位姿RMSE 0.364 mm；最大腿加速度0.165 m/s²，驱动力、腿长和腿速也均满足中期约束。")
    add_paragraph(doc, "针对固定控制律缺少显式多约束协调和前向安全保证的问题，本文定义了以F_nom=F_CT+F_LQI为名义输入的单步QP框架。初步方案能够规范表达力幅值、力变化率和支链运动约束，并提供活动约束、松弛量、求解时间和回退等诊断接口；但碰撞和奇异性在本阶段只作风险监测，历史轻量级滚动数据不作为论文性能证据。")
    add_paragraph(doc, "后续补充工作应在精确输入仿射模型上构造严格的收敛和安全条件，完成同一Simscape被控对象上的理想力、非理想执行器、消融与扰动仿真。只有当数学一致性、持续可行性、硬约束满足和实时性均通过验收后，方可在本独立论文后部追加严格方法的性能结论。")

    add_chapter_break(doc)
    add_heading(doc, "参考文献", 1)
    references = [
        "[1] STEWART D. A platform with six degrees of freedom[J]. Proceedings of the Institution of Mechanical Engineers, 1965, 180(1): 371-386.",
        "[2] MERLET J P. Parallel Robots[M]. 2nd ed. Dordrecht: Springer, 2006.",
        "[3] FICHTER E F. A Stewart platform-based manipulator: general theory and practical construction[J]. The International Journal of Robotics Research, 1986, 5(2): 157-182.",
        "[4] DASGUPTA B, MRUTHYUNJAYA T S. The Stewart platform manipulator: a review[J]. Mechanism and Machine Theory, 2000, 35(1): 15-40.",
        "[5] SPONG M W, HUTCHINSON S, VIDYASAGAR M. Robot Modeling and Control[M]. Hoboken: John Wiley & Sons, 2006.",
        "[6] SLOTINE J J E, LI W. Applied Nonlinear Control[M]. Englewood Cliffs: Prentice Hall, 1991.",
        "[7] LEWIS F L, VRABIE D L, SYRMOS V L. Optimal Control[M]. 3rd ed. Hoboken: John Wiley & Sons, 2012.",
        "[8] ANDERSON B D O, MOORE J B. Linear Optimal Control[M]. Englewood Cliffs: Prentice Hall, 1971.",
        "[9] ÅSTRÖM K J, RUNDQWIST L. Integrator windup and how to avoid it[C]//Proceedings of the 1989 American Control Conference. Pittsburgh: IEEE, 1989: 1693-1698.",
        "[10] BOYD S, VANDENBERGHE L. Convex Optimization[M]. Cambridge: Cambridge University Press, 2004.",
        "[11] AMES A D, XU X, GRIZZLE J W, et al. Control barrier function based quadratic programs for safety critical systems[J]. IEEE Transactions on Automatic Control, 2017, 62(8): 3861-3876.",
        "[12] KHALIL H K. Nonlinear Systems[M]. 3rd ed. Upper Saddle River: Prentice Hall, 2002.",
        "[13] SICILIANO B, SCIAVICCO L, VILLANI L, et al. Robotics: Modelling, Planning and Control[M]. London: Springer, 2009.",
        "[14] NGUYEN Q, SREENATH K. Exponential control barrier functions for enforcing high relative-degree safety-critical constraints[C]//Proceedings of the 2016 American Control Conference. Boston: IEEE, 2016: 322-328.",
        "[15] RAWLINGS J B, MAYNE D Q, DIEHL M. Model Predictive Control: Theory, Computation, and Design[M]. 2nd ed. Madison: Nob Hill Publishing, 2017.",
        "[16] NOCEDAL J, WRIGHT S J. Numerical Optimization[M]. 2nd ed. New York: Springer, 2006.",
        "[17] THE MATHWORKS, INC. Simscape Multibody User's Guide[EB/OL]. Natick: The MathWorks, Inc., 2026[2026-07-12]. https://www.mathworks.com/help/sm/.",
    ]
    for reference in references:
        add_paragraph(doc, reference, first_line=False, align=WD_ALIGN_PARAGRAPH.JUSTIFY, style="Reference Thesis")

    doc.core_properties.title = "并联机器人计算力矩-LQI与CLF-CBF-QP控制研究"
    doc.core_properties.subject = "独立控制论文理论初稿"
    doc.core_properties.author = "谢振宇"
    doc.core_properties.keywords = "Stewart; computed torque; LQI; constrained QP; Simscape"
    doc.core_properties.comments = "基线数据来自2026-07-12重新运行的Simscape结果；初步QP不引用历史失真数据。"
    return doc


def _remove_paragraph(paragraph) -> None:
    parent = paragraph._p.getparent()
    if parent is not None:
        parent.remove(paragraph._p)


def _remove_captioned_figure(doc: Document, caption_text: str) -> None:
    for paragraph in list(doc.paragraphs):
        if paragraph.text.strip() == caption_text:
            previous = paragraph._p.getprevious()
            if previous is not None and previous.tag == qn("w:p"):
                previous.getparent().remove(previous)
            _remove_paragraph(paragraph)
            return
    raise RuntimeError(f"figure caption not found: {caption_text}")


def _remove_captioned_table(doc: Document, caption_text: str) -> None:
    for paragraph in list(doc.paragraphs):
        if paragraph.text.strip() == caption_text:
            next_node = paragraph._p.getnext()
            if next_node is not None and next_node.tag == qn("w:tbl"):
                parent = next_node.getparent()
                after = next_node.getnext()
                parent.remove(next_node)
                if after is not None and after.tag == qn("w:p") and not "".join(after.itertext()).strip():
                    parent.remove(after)
            _remove_paragraph(paragraph)
            return
    raise RuntimeError(f"table caption not found: {caption_text}")


def compact_initial_document(doc: Document) -> None:
    """Keep the initial manuscript within the 18-22 page budget reserved by the user."""
    for caption in [
        "图2-1  Stewart平台坐标系、铰点与六条支链",
        "图2-2  运动学、Jacobian与动力学计算链",
        "图2-3  节点轨迹的Hermite重建及速度连续性",
        "图2-4  Simscape Stewart平台顶层被控对象",
        "图3-2  力到位姿对象线性化与LQI设计流程",
        "图4-1  约束QP作为基线总力在线校正层",
        "图5-1  初步约束QP的目标与约束层次",
        "图5-4  最新基线六支链驱动力与执行器力跟踪误差",
    ]:
        _remove_captioned_figure(doc, caption)
    for caption in [
        "表1-1  本文研究任务与证据层级",
        "表2-1  中期约束与本文统一单位",
        "表2-2  主要符号及其代码对应关系",
        "表3-1  最新基线仿真控制配置",
        "表4-1  基线问题、QP作用与本稿结论边界",
        "表5-1  初步QP各类约束的证据属性",
        "表5-2  最新基线仿真条件与结果来源",
        "表5-4  六维位姿分量误差",
    ]:
        _remove_captioned_table(doc, caption)

    remove_exact = {
        "1.2.1 并联机器人动力学与计算力矩控制",
        "1.2.2 积分最优控制与抗饱和",
        "1.2.3 约束优化与安全关键控制",
        "1.4 论文结构",
    }
    remove_prefixes = [
        "传统工业控制往往将轨迹规划和闭环控制分开处理",
        "This study first establishes a consistent kinematic and dynamic description",
        "Stewart平台的控制研究以运动学映射",
        "J_v的第i行由支链方向和动平台铰点相对平台中心的力臂构成",
        "在平台扭量坐标中，将动平台、载荷及等效支链惯性合并后",
        "控制仿真读取上游两阶段轨迹文件",
        "约束优化控制将控制输入作为在线决策变量",
        "论文中的结论严格按照证据层级表述",
        "第1章说明研究背景、相关方法及本文证据边界",
        "在H(q)正定且J_v可计算的工作空间内",
        "同样，为把平移误差和姿态误差合并为单一评价指标",
        "本章通过功率一致性统一了运动学和动力学符号",
        "对式(3-1)解析求导得到q̇_r和q̈_r",
        "计算结果按执行器力上下限进行保护",
        "代码中`lqi`函数返回的全状态增益",
        "K_aw由积分增益的伪逆和抗饱和时间常数构造",
        "在轨迹开始前，执行器内部状态由第一拍计算力矩初始化",
        "本章建立了计算力矩前馈与LQI输出反馈基线",
        "上述推导说明基线在既定局部条件下具有渐近稳定性",
        "提高LQI权重或反馈增益可以在部分工况下降低误差",
        "抗饱和回灌能够修正积分器内部状态",
        "这种接口具有三个优点",
        "本章从控制结构而非单次仿真失败出发",
        "本初稿保留早期方法的合理设计动机和标准QP框架",
        "展开二次项后，单步问题可写成标准凸QP形式",
        "当驱动力边界和力变化率边界同时存在时",
        "为说明六个通道的误差分布",
        "本章给出了初步约束QP的决策变量",
        "本文围绕Stewart并联机器人载荷挂装轨迹的闭环执行问题",
    ]
    for paragraph in list(doc.paragraphs):
        text = paragraph.text.strip()
        if text in remove_exact or any(text.startswith(prefix) for prefix in remove_prefixes):
            _remove_paragraph(paragraph)

    max_width = Inches(5.20)
    for shape in doc.inline_shapes:
        if shape.width > max_width:
            ratio = max_width / shape.width
            shape.width = max_width
            shape.height = int(shape.height * ratio)


def write_word_finalizer() -> None:
    script = r'''param(
    [Parameter(Mandatory=$true)][string]$DocxPath,
    [Parameter(Mandatory=$true)][string]$EquationJson,
    [string]$PdfPath = ""
)
$ErrorActionPreference = "Stop"
$equations = Get-Content -Raw -Encoding UTF8 -LiteralPath $EquationJson | ConvertFrom-Json
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$word.Options.UpdateFieldsAtPrint = $true
$doc = $null
$equationDoc = $null
try {
    $doc = $word.Documents.Open($DocxPath, $false, $false)
    $equationDoc = $word.Documents.Add()
    $properties = @($equations.PSObject.Properties)
    for ($propertyIndex = $properties.Count - 1; $propertyIndex -ge 0; $propertyIndex--) {
        $property = $properties[$propertyIndex]
        $id = [string]$property.Name
        $entry = $property.Value
        $marker = "[[EQ:$id]]"
        $equationRange = $doc.Content.Duplicate
        $find = $equationRange.Find
        $find.ClearFormatting()
        $find.Text = $marker
        $find.Forward = $true
        $find.Wrap = 0
        $find.MatchCase = $true
        $find.MatchWildcards = $false
        $found = $find.Execute()
        if (-not $found) { throw "Equation marker not found: $id" }
        $linear = [string]$entry.linear
        $equationDoc.Content.Text = ""
        $equationParagraph = $equationDoc.Paragraphs.Item(1)
        $sourceRange = $equationParagraph.Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End = $sourceRange.End - 1 }
        $sourceRange.Text = $linear
        $equationParagraph = $equationDoc.Paragraphs.Item(1)
        $sourceRange = $equationParagraph.Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End = $sourceRange.End - 1 }
        $sourceRange.Font.Name = "Cambria Math"
        $sourceRange.Font.NameFarEast = "宋体"
        $sourceRange.Font.Size = 12
        $addedRange = $equationDoc.OMaths.Add($sourceRange)
        $addedRange.OMaths.Item(1).BuildUp()
        $equationParagraph = $equationDoc.Paragraphs.Item(1)
        $sourceRange = $equationParagraph.Range.Duplicate
        if ($sourceRange.End -gt $sourceRange.Start) { $sourceRange.End = $sourceRange.End - 1 }
        $equationRange.FormattedText = $sourceRange.FormattedText
    }
    $equationDoc.Close($false)
    $equationDoc = $null
    [void]$doc.Fields.Update()
    for ($i = 1; $i -le $doc.TablesOfContents.Count; $i++) {
        $doc.TablesOfContents.Item($i).Update()
    }
    $doc.Repaginate()
    $doc.Save()
    if ($PdfPath -ne "") {
        $pdfDir = Split-Path -Parent $PdfPath
        if (-not (Test-Path -LiteralPath $pdfDir)) { New-Item -ItemType Directory -Force -Path $pdfDir | Out-Null }
        $doc.ExportAsFixedFormat($PdfPath, 17)
    }
    $doc.Close($false)
    $doc = $null
}
finally {
    if ($null -ne $equationDoc) {
        try { $equationDoc.Close($false) } catch { }
    }
    if ($null -ne $doc) {
        try { $doc.Close($false) } catch { }
    }
    $word.Quit()
    [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($word)
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
'''
    FINALIZER_PS1.write_text(script, encoding="utf-8-sig")


def finalize_with_word(docx_path: Path, pdf_path: Path | None = None) -> None:
    EQUATION_JSON.write_text(json.dumps(EQUATIONS, ensure_ascii=False, indent=2), encoding="utf-8-sig")
    write_word_finalizer()
    command = [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", str(FINALIZER_PS1),
        "-DocxPath", str(docx_path),
        "-EquationJson", str(EQUATION_JSON),
    ]
    if pdf_path is not None:
        command.extend(["-PdfPath", str(pdf_path)])
    subprocess.run(command, cwd=WORKSPACE, check=True)


def package_text(path: Path) -> str:
    chunks: list[str] = []
    with zipfile.ZipFile(path) as archive:
        for name in archive.namelist():
            if name.endswith((".xml", ".rels")):
                chunks.append(archive.read(name).decode("utf-8", errors="ignore"))
    return "\n".join(chunks)


def structural_qa(path: Path) -> dict[str, int | bool]:
    final = Document(str(path))
    text = document_text(final)
    xml = package_text(path)
    with zipfile.ZipFile(path) as archive:
        document_xml = archive.read("word/document.xml").decode("utf-8", errors="ignore")
    forbidden = "M" + "CP"
    if forbidden.lower() in (text + "\n" + xml).lower():
        raise RuntimeError("forbidden acronym found in final document")
    for marker in ("[[EQ:", "待补充"):
        if marker.lower() in (text + "\n" + xml).lower():
            raise RuntimeError(f"unresolved marker found: {marker}")
    for marker in ("placeholder", "TODO"):
        if marker.lower() in text.lower():
            raise RuntimeError(f"unresolved visible marker found: {marker}")
    if re.search(r"E\d{3}\]\]", text + "\n" + document_xml):
        raise RuntimeError("partial equation marker remains visible")
    omath_count = len(re.findall(r"<m:oMath(?:\s|>)", document_xml))
    equation_paragraph_count = sum(1 for paragraph in final.paragraphs if paragraph._p.xpath(".//m:oMath"))
    heading_count = sum(1 for p in final.paragraphs if p.style.name.startswith("Heading"))
    figure_count = len(final.inline_shapes)
    table_count = len(final.tables)
    if equation_paragraph_count != len(EQUATIONS):
        raise RuntimeError(
            f"native equation paragraph count {equation_paragraph_count} != expected {len(EQUATIONS)}"
        )
    if figure_count < 3:
        raise RuntimeError(f"figure count too small: {figure_count}")
    if table_count < 2:
        raise RuntimeError(f"table count too small: {table_count}")
    return {
        "equations_expected": len(EQUATIONS),
        "omath_count": omath_count,
        "equation_paragraph_count": equation_paragraph_count,
        "heading_count": heading_count,
        "figure_count": figure_count,
        "table_count": table_count,
        "forbidden_absent": True,
        "markers_absent": True,
    }


def build(*, export_pdf: bool = True) -> dict[str, object]:
    ensure_dirs()
    required_assets = [FRESH_TRACKING, FRESH_LEG, FRESH_FORCE]
    missing_assets = [str(path) for path in required_assets if not path.exists()]
    if missing_assets:
        raise FileNotFoundError("Fresh baseline figures are missing: " + "; ".join(missing_assets))
    EQUATIONS.clear()
    doc = build_initial_document()
    compact_initial_document(doc)
    draft_text = document_text(doc)
    forbidden = "M" + "CP"
    if forbidden.lower() in draft_text.lower():
        raise RuntimeError("forbidden acronym found before save")
    doc.save(OUTPUT_DOCX)
    qa_pdf = WORK_DIR / "control_initial_qa" / "并联机器人控制研究_理论初稿.pdf" if export_pdf else None
    finalize_with_word(OUTPUT_DOCX, qa_pdf)
    shutil.copy2(OUTPUT_DOCX, SNAPSHOT_DOCX)
    qa = structural_qa(OUTPUT_DOCX)
    qa.update({
        "docx": str(OUTPUT_DOCX),
        "snapshot": str(SNAPSHOT_DOCX),
        "pdf": str(qa_pdf) if qa_pdf else "",
        "fresh_result": str(FRESH_RESULT),
    })
    qa_path = WORK_DIR / "control_initial_qa" / "structural_qa.json"
    qa_path.parent.mkdir(parents=True, exist_ok=True)
    qa_path.write_text(json.dumps(qa, ensure_ascii=False, indent=2), encoding="utf-8")
    return qa


def main() -> None:
    parser = argparse.ArgumentParser(description="生成并联机器人控制研究独立Word理论初稿")
    parser.add_argument("--no-pdf", action="store_true", help="不使用Word COM导出QA PDF")
    args = parser.parse_args()
    qa = build(export_pdf=not args.no_pdf)
    print(json.dumps(qa, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
