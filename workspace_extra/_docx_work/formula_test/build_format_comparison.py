from pathlib import Path
import sys

from PIL import Image, ImageDraw, ImageFont
from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_ROW_HEIGHT_RULE, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.style import WD_STYLE_TYPE
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor

WORK = Path(r"D:\NPU\Matlab_code\_docx_work\formula_test")
OUT = Path(r"D:\NPU\Matlab_code\output\Word_同一公式_不同表达格式测试.docx")
IMG = WORK / "quadratic_formula_raster.png"

sys.path.insert(0, str(WORK))
from build_formula_test import (  # noqa: E402
    mr, frac, radical, sup, add_math, set_east_asia_font,
    set_cell_margins, set_cell_shading, set_table_geometry, add_page_field,
)

BLUE = "2E74B5"
DARK = "1F4D78"
INK = "0B2545"
MUTED = "667085"
LIGHT = "E8EEF5"
PALE = "F4F6F9"
GOLD = "9A6700"
GREEN = "1F6B4F"


def create_formula_image():
    text = "x = (−b ± √(b² − 4ac)) / 2a"
    font = ImageFont.truetype(r"C:\Windows\Fonts\cambria.ttc", 54)
    canvas = Image.new("RGBA", (1600, 180), (255, 255, 255, 0))
    draw = ImageDraw.Draw(canvas)
    box = draw.textbbox((0, 0), text, font=font)
    x = (canvas.width - (box[2] - box[0])) // 2
    y = (canvas.height - (box[3] - box[1])) // 2 - box[1]
    draw.text((x, y), text, font=font, fill=(0, 0, 0, 255))
    bbox = canvas.getbbox()
    canvas.crop(bbox).save(IMG, dpi=(300, 300))


def configure_styles(doc):
    normal = doc.styles["Normal"]
    normal.font.name = "Calibri"
    normal.font.size = Pt(11)
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.25

    title = doc.styles["Title"]
    title.font.name = "Calibri"
    title.font.size = Pt(23)
    title.font.bold = True
    title.font.color.rgb = RGBColor.from_string(INK)
    title._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    title.paragraph_format.space_before = Pt(0)
    title.paragraph_format.space_after = Pt(5)

    subtitle = doc.styles["Subtitle"]
    subtitle.font.name = "Calibri"
    subtitle.font.size = Pt(11.5)
    subtitle.font.color.rgb = RGBColor.from_string(MUTED)
    subtitle._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    subtitle.paragraph_format.space_after = Pt(7)

    for name, size, before, after, color in (
        ("Heading 1", 16, 18, 10, BLUE),
        ("Heading 2", 13, 14, 7, BLUE),
        ("Heading 3", 12, 10, 5, DARK),
    ):
        st = doc.styles[name]
        st.font.name = "Calibri"
        st.font.size = Pt(size)
        st.font.bold = True
        st.font.color.rgb = RGBColor.from_string(color)
        st._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        st.paragraph_format.space_before = Pt(before)
        st.paragraph_format.space_after = Pt(after)
        st.paragraph_format.keep_with_next = True

    if "Code Small" not in doc.styles:
        code = doc.styles.add_style("Code Small", WD_STYLE_TYPE.PARAGRAPH)
    else:
        code = doc.styles["Code Small"]
    code.font.name = "Consolas"
    code.font.size = Pt(8.5)
    code._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    code.paragraph_format.space_before = Pt(0)
    code.paragraph_format.space_after = Pt(0)
    code.paragraph_format.line_spacing = 1.0


def configure_page(doc):
    sec = doc.sections[0]
    sec.page_width = Inches(8.5)
    sec.page_height = Inches(11)
    sec.top_margin = Inches(0.65)
    sec.bottom_margin = Inches(0.65)
    sec.left_margin = Inches(1)
    sec.right_margin = Inches(1)
    sec.header_distance = Inches(0.492)
    sec.footer_distance = Inches(0.492)

    hp = sec.header.paragraphs[0]
    hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    r = hp.add_run("WORD FORMULA FORMAT MAP")
    set_east_asia_font(r)
    r.font.size = Pt(8.5)
    r.font.color.rgb = RGBColor.from_string(MUTED)

    fp = sec.footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = fp.add_run("同一公式 · 不同保存方式  |  ")
    set_east_asia_font(r)
    r.font.size = Pt(9)
    r.font.color.rgb = RGBColor.from_string(MUTED)
    add_page_field(fp)


def add_professional_formula(paragraph):
    items = [
        mr("x"), mr("="),
        frac(
            [mr("−"), mr("b"), mr("±"), radical([sup([mr("b")], [mr("2")]), mr("−4ac")])],
            [mr("2a")],
        ),
    ]
    add_math(paragraph, items, display=True)


def add_linear_omml(paragraph):
    add_math(paragraph, [mr("x=(-b±√(b^2-4ac))/(2a)", plain=True)], display=False)


def add_eq_field(paragraph):
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = r" EQ x = \F(-b ± \R(,b\S(2)-4ac),2a) "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    result = OxmlElement("w:t")
    result.text = "x = (−b ± √(b² − 4ac)) / 2a"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, result, end])
    set_east_asia_font(run, latin="Cambria Math")
    run.font.size = Pt(13)


def add_cell_text(cell, text, font="Calibri", size=9.5, color=None, bold=False, align=WD_ALIGN_PARAGRAPH.LEFT):
    p = cell.paragraphs[0]
    p.alignment = align
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.line_spacing = 1.05
    r = p.add_run(text)
    set_east_asia_font(r, latin=font)
    r.font.size = Pt(size)
    r.bold = bold
    if color:
        r.font.color.rgb = RGBColor.from_string(color)
    return p


def add_status(cell, editability, storage, color):
    p = cell.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.LEFT
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run(editability)
    set_east_asia_font(r)
    r.bold = True
    r.font.size = Pt(9)
    r.font.color.rgb = RGBColor.from_string(color)
    r2 = p.add_run("\n" + storage)
    set_east_asia_font(r2)
    r2.font.size = Pt(8.3)
    r2.font.color.rgb = RGBColor.from_string(MUTED)


def format_row(row):
    row.height_rule = WD_ROW_HEIGHT_RULE.AT_LEAST
    for cell in row.cells:
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        set_cell_margins(cell, top=45, bottom=45, start=120, end=120)


def build():
    WORK.mkdir(parents=True, exist_ok=True)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    create_formula_image()

    doc = Document()
    configure_styles(doc)
    configure_page(doc)

    kicker = doc.add_paragraph()
    kicker.paragraph_format.space_after = Pt(2)
    r = kicker.add_run("FORMULA FORMAT MAP")
    set_east_asia_font(r)
    r.bold = True
    r.font.size = Pt(9.5)
    r.font.color.rgb = RGBColor.from_string(BLUE)

    doc.add_paragraph("同一个公式的不同 Word 表达格式", style="Title")
    doc.add_paragraph("只比较保存方式与可编辑性，不再堆叠不同数学公式。测试公式：一元二次方程求根公式。", style="Subtitle")

    table = doc.add_table(rows=1, cols=3)
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    widths = [2200, 4760, 2400]
    hdr = table.rows[0]
    format_row(hdr)
    for i, text in enumerate(("表达方式 / 对象类型", "同一公式的实际表示", "可编辑性 / Word 内部保存")):
        set_cell_shading(hdr.cells[i], LIGHT)
        add_cell_text(hdr.cells[i], text, size=9.3, color=DARK, bold=True, align=WD_ALIGN_PARAGRAPH.CENTER)
    trpr = hdr._tr.get_or_add_trPr()
    repeat = OxmlElement("w:tblHeader")
    repeat.set(qn("w:val"), "true")
    trpr.append(repeat)

    # 1 Professional native Office Math.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "1  Word 原生专业公式\nOMML · Professional", bold=True, color=DARK)
    p = row.cells[1].paragraphs[0]; p.alignment = WD_ALIGN_PARAGRAPH.CENTER; add_professional_formula(p)
    add_status(row.cells[2], "完全可编辑", "原生 m:oMath 二维结构", GREEN)

    # 2 Linear equation object.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "2  Word 原生线性公式\nOMML · Linear", bold=True, color=DARK)
    p = row.cells[1].paragraphs[0]; p.alignment = WD_ALIGN_PARAGRAPH.CENTER; add_linear_omml(p)
    add_status(row.cells[2], "可编辑", "公式对象，线性显示", GREEN)

    # 3 UnicodeMath source.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "3  UnicodeMath 输入串\nWord 线性语法", bold=True, color=DARK)
    add_cell_text(row.cells[1], "x=(-b±√(b^2-4ac))/(2a)", font="Cambria Math", size=11.5, align=WD_ALIGN_PARAGRAPH.CENTER)
    add_status(row.cells[2], "可转换", "普通文本；粘入公式框后转换", GOLD)

    # 4 LaTeX source.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "4  LaTeX 输入串\nWord LaTeX 模式", bold=True, color=DARK)
    add_cell_text(row.cells[1], r"x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}", font="Consolas", size=8.8, align=WD_ALIGN_PARAGRAPH.CENTER)
    add_status(row.cells[2], "可转换", "普通文本；需 LaTeX 输入模式", GOLD)

    # 5 Unicode plain text.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "5  Unicode 普通文本\n字符本身", bold=True, color=DARK)
    add_cell_text(row.cells[1], "x = (−b ± √(b² − 4ac)) / 2a", font="Cambria Math", size=11.5, align=WD_ALIGN_PARAGRAPH.CENTER)
    add_status(row.cells[2], "文本可编辑", "w:t 普通字符，不是公式对象", GOLD)

    # 6 Manual run formatting.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "6  手工上下标文本\n运行级格式", bold=True, color=DARK)
    p = row.cells[1].paragraphs[0]; p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for text, sup_flag in (("x = (−b ± √(b", False), ("2", True), (" − 4ac)) / 2a", False)):
        r = p.add_run(text); set_east_asia_font(r, latin="Cambria Math"); r.font.size = Pt(11.5); r.font.superscript = sup_flag
    add_status(row.cells[2], "文本可编辑", "多个 w:r，2 设置为上标", GOLD)

    # 7 Legacy EQ field.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "7  旧式 EQ 域\nLegacy Field", bold=True, color=DARK)
    p = row.cells[1].paragraphs[0]; p.alignment = WD_ALIGN_PARAGRAPH.CENTER; add_eq_field(p)
    add_status(row.cells[2], "域代码可编辑", "w:fldChar + EQ 指令；可按 F9", GOLD)

    # 8 MathML source.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "8  MathML 源码\nXML 文本", bold=True, color=DARK)
    mathml = '<math><mi>x</mi><mo>=</mo><mfrac><mrow><mo>−</mo><mi>b</mi><mo>±</mo><msqrt><msup><mi>b</mi><mn>2</mn></msup><mo>−</mo><mn>4</mn><mi>a</mi><mi>c</mi></msqrt></mrow><mrow><mn>2</mn><mi>a</mi></mrow></mfrac></math>'
    add_cell_text(row.cells[1], mathml, font="Consolas", size=7.2)
    add_status(row.cells[2], "源码可编辑", "普通 XML 文本；需导入器转换", GOLD)

    # 9 Raster image.
    row = table.add_row(); format_row(row)
    add_cell_text(row.cells[0], "9  PNG 图片公式\nRaster Image", bold=True, color=DARK)
    p = row.cells[1].paragraphs[0]; p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.add_run().add_picture(str(IMG), width=Inches(2.75))
    add_status(row.cells[2], "公式不可编辑", "嵌入 PNG 图片；只能缩放/替换", "9B1C1C")

    set_table_geometry(table, widths, indent=120)

    doc.add_heading("还能有哪些？", level=2)
    note = doc.add_paragraph()
    note.paragraph_format.space_after = Pt(4)
    r = note.add_run("依赖本机组件的格式：")
    set_east_asia_font(r); r.bold = True; r.font.color.rgb = RGBColor.from_string(DARK)
    r2 = note.add_run("MathType OLE、Microsoft Equation 3.0 OLE、外部链接公式对象，以及 EMF/SVG 矢量图片。它们并非所有 Word 环境都能创建或编辑，因此本文件不伪造这些对象。")
    set_east_asia_font(r2)

    hint = doc.add_paragraph()
    hint.paragraph_format.space_before = Pt(2)
    hint.paragraph_format.space_after = Pt(0)
    ppr = hint._p.get_or_add_pPr()
    shd = OxmlElement("w:shd"); shd.set(qn("w:fill"), PALE); ppr.append(shd)
    r = hint.add_run("建议测试动作：双击、切换“专业/线性”、Alt+= 后粘贴源码、F9 更新 EQ 域、复制到 WPS/LibreOffice、另存 PDF。")
    set_east_asia_font(r); r.font.size = Pt(9.3); r.font.color.rgb = RGBColor.from_string(MUTED)

    props = doc.core_properties
    props.title = "同一个公式的不同 Word 表达格式"
    props.subject = "Word 公式对象与源码格式兼容性测试"
    props.author = "Codex"
    props.keywords = "Word, OMML, UnicodeMath, LaTeX, EQ field, MathML, formula"
    settings = doc.settings._element
    update = settings.find(qn("w:updateFields"))
    if update is None:
        update = OxmlElement("w:updateFields"); settings.append(update)
    update.set(qn("w:val"), "true")

    doc.save(OUT)
    print(OUT)


if __name__ == "__main__":
    build()
