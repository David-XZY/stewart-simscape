from pathlib import Path
from copy import deepcopy

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement, parse_xml
from docx.oxml.ns import nsdecls, qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path(r"D:\NPU\Matlab_code")
WORK = ROOT / "_docx_work" / "formula_test"
OUT = ROOT / "output" / "Word_公式格式测试集.docx"

BLUE = "2E74B5"
DARK_BLUE = "1F4D78"
INK = "0B2545"
MUTED = "667085"
LIGHT = "E8EEF5"
PALE = "F4F6F9"
GRID = "C9D2DF"
GOLD = "9A6700"
GREEN = "1F6B4F"

M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
NS = {"m": M_NS, "w": W_NS}


def set_east_asia_font(run, latin="Calibri", east="Microsoft YaHei"):
    run.font.name = latin
    rpr = run._element.get_or_add_rPr()
    fonts = rpr.rFonts
    if fonts is None:
        fonts = OxmlElement("w:rFonts")
        rpr.insert(0, fonts)
    fonts.set(qn("w:ascii"), latin)
    fonts.set(qn("w:hAnsi"), latin)
    fonts.set(qn("w:eastAsia"), east)
    fonts.set(qn("w:cs"), latin)


def set_cell_shading(cell, fill):
    tcpr = cell._tc.get_or_add_tcPr()
    shd = tcpr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tcpr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120):
    tc = cell._tc
    tcpr = tc.get_or_add_tcPr()
    tcmar = tcpr.first_child_found_in("w:tcMar")
    if tcmar is None:
        tcmar = OxmlElement("w:tcMar")
        tcpr.append(tcmar)
    for name, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tcmar.find(qn(f"w:{name}"))
        if node is None:
            node = OxmlElement(f"w:{name}")
            tcmar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_table_geometry(table, widths, indent=120):
    table.autofit = False
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    tblpr = table._tbl.tblPr
    tblw = tblpr.find(qn("w:tblW"))
    if tblw is None:
        tblw = OxmlElement("w:tblW")
        tblpr.append(tblw)
    tblw.set(qn("w:w"), str(sum(widths)))
    tblw.set(qn("w:type"), "dxa")
    tblind = tblpr.find(qn("w:tblInd"))
    if tblind is None:
        tblind = OxmlElement("w:tblInd")
        tblpr.append(tblind)
    tblind.set(qn("w:w"), str(indent))
    tblind.set(qn("w:type"), "dxa")
    layout = tblpr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        tblpr.append(layout)
    layout.set(qn("w:type"), "fixed")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(width))
        grid.append(col)
    for row in table.rows:
        for idx, cell in enumerate(row.cells):
            tcpr = cell._tc.get_or_add_tcPr()
            tcw = tcpr.find(qn("w:tcW"))
            if tcw is None:
                tcw = OxmlElement("w:tcW")
                tcpr.append(tcw)
            tcw.set(qn("w:w"), str(widths[idx]))
            tcw.set(qn("w:type"), "dxa")
            cell.width = Inches(widths[idx] / 1440)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            set_cell_margins(cell)


def set_paragraph_shading(paragraph, fill=PALE, border=GRID):
    ppr = paragraph._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), fill)
    ppr.append(shd)
    pbdr = OxmlElement("w:pBdr")
    for side in ("top", "left", "bottom", "right"):
        edge = OxmlElement(f"w:{side}")
        edge.set(qn("w:val"), "single")
        edge.set(qn("w:sz"), "4")
        edge.set(qn("w:space"), "6")
        edge.set(qn("w:color"), border)
        pbdr.append(edge)
    ppr.append(pbdr)


def add_page_field(paragraph, field="PAGE"):
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = f" {field} "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    result = OxmlElement("w:t")
    result.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, result, end])
    set_east_asia_font(run)
    run.font.size = Pt(9)
    run.font.color.rgb = RGBColor.from_string(MUTED)


def el(tag, **attrs):
    node = OxmlElement(f"m:{tag}")
    for key, value in attrs.items():
        node.set(qn(f"m:{key}"), str(value))
    return node


def mr(text, plain=False, bold=False):
    node = el("r")
    if plain or bold:
        rpr = el("rPr")
        sty = el("sty", val="b" if bold else "p")
        rpr.append(sty)
        node.append(rpr)
    t = el("t")
    t.text = str(text)
    node.append(t)
    return node


def seq(*items):
    out = []
    for item in items:
        if item is None:
            continue
        if isinstance(item, str):
            out.append(mr(item))
        elif isinstance(item, (list, tuple)):
            out.extend(seq(*item))
        else:
            out.append(item)
    return out


def container(tag, children, **attrs):
    node = el(tag, **attrs)
    for child in seq(*children if isinstance(children, (list, tuple)) else [children]):
        node.append(child)
    return node


def frac(num, den, ftype=None):
    node = el("f")
    if ftype:
        fpr = el("fPr")
        fpr.append(el("type", val=ftype))
        node.append(fpr)
    node.append(container("num", num))
    node.append(container("den", den))
    return node


def radical(body, degree=None):
    node = el("rad")
    rpr = el("radPr")
    rpr.append(el("degHide", val="1" if degree is None else "0"))
    node.append(rpr)
    node.append(container("deg", [] if degree is None else degree))
    node.append(container("e", body))
    return node


def script(kind, base, lower=None, upper=None):
    node = el(kind)
    node.append(el(f"{kind}Pr"))
    node.append(container("e", base))
    if lower is not None:
        node.append(container("sub", lower))
    if upper is not None:
        node.append(container("sup", upper))
    return node


def sub(base, lower):
    return script("sSub", base, lower=lower)


def sup(base, upper):
    return script("sSup", base, upper=upper)


def subsup(base, lower, upper):
    return script("sSubSup", base, lower=lower, upper=upper)


def nary(char, body, lower=None, upper=None):
    node = el("nary")
    npr = el("naryPr")
    npr.append(el("chr", val=char))
    npr.append(el("limLoc", val="undOvr"))
    npr.append(el("subHide", val="1" if lower is None else "0"))
    npr.append(el("supHide", val="1" if upper is None else "0"))
    node.append(npr)
    node.append(container("sub", [] if lower is None else lower))
    node.append(container("sup", [] if upper is None else upper))
    node.append(container("e", body))
    return node


def delim(body, begin="(", end=")", sep="|"):
    node = el("d")
    dpr = el("dPr")
    dpr.append(el("begChr", val=begin))
    dpr.append(el("endChr", val=end))
    dpr.append(el("sepChr", val=sep))
    node.append(dpr)
    bodies = body if isinstance(body, list) and body and isinstance(body[0], list) else [body]
    for part in bodies:
        node.append(container("e", part))
    return node


def matrix(rows):
    node = el("m")
    mpr = el("mPr")
    mcs = el("mcs")
    for _ in range(len(rows[0])):
        mc = el("mc")
        mcpr = el("mcPr")
        mcpr.append(el("mcJc", val="center"))
        mc.append(mcpr)
        mcs.append(mc)
    mpr.append(mcs)
    node.append(mpr)
    for row in rows:
        mrnode = el("mr")
        for cell in row:
            mrnode.append(container("e", cell))
        node.append(mrnode)
    return node


def eq_array(rows):
    node = el("eqArr")
    pr = el("eqArrPr")
    pr.append(el("maxDist", val="1"))
    node.append(pr)
    for row in rows:
        node.append(container("e", row))
    return node


def limit_low(base, limit):
    node = el("limLow")
    node.append(el("limLowPr"))
    node.append(container("e", base))
    node.append(container("lim", limit))
    return node


def accent(body, char="ˆ"):
    node = el("acc")
    pr = el("accPr")
    pr.append(el("chr", val=char))
    node.append(pr)
    node.append(container("e", body))
    return node


def bar(body, pos="top"):
    node = el("bar")
    pr = el("barPr")
    pr.append(el("pos", val=pos))
    node.append(pr)
    node.append(container("e", body))
    return node


def func(name, arg):
    node = el("func")
    node.append(container("fName", [mr(name, plain=True)]))
    node.append(container("e", arg))
    return node


def group_char(body, char="⏟", pos="bot", vert="bot"):
    node = el("groupChr")
    pr = el("groupChrPr")
    pr.append(el("chr", val=char))
    pr.append(el("pos", val=pos))
    pr.append(el("vertJc", val=vert))
    node.append(pr)
    node.append(container("e", body))
    return node


def add_math(paragraph, items, display=True):
    if display:
        omp = el("oMathPara")
        ompp = el("oMathParaPr")
        ompp.append(el("jc", val="center"))
        omp.append(ompp)
        om = el("oMath")
        for item in seq(*items):
            om.append(item)
        omp.append(om)
        paragraph._p.append(omp)
    else:
        om = el("oMath")
        for item in seq(*items):
            om.append(item)
        paragraph._p.append(om)


def add_formula_block(doc, number, title, items, note, badge="原生 OMML"):
    head = doc.add_paragraph(style="Formula Label")
    head.paragraph_format.keep_with_next = True
    r = head.add_run(f"{number:02d}  {title}")
    set_east_asia_font(r)
    r.bold = True
    r.font.color.rgb = RGBColor.from_string(DARK_BLUE)
    badge_run = head.add_run(f"   {badge}")
    set_east_asia_font(badge_run)
    badge_run.font.size = Pt(8.5)
    badge_run.font.color.rgb = RGBColor.from_string(GREEN if badge.startswith("原生") else GOLD)
    p = doc.add_paragraph(style="Formula Display")
    p.paragraph_format.keep_with_next = True
    add_math(p, items, display=True)
    desc = doc.add_paragraph(style="Formula Note")
    rr = desc.add_run(note)
    set_east_asia_font(rr)


def add_text_formula_block(doc, number, title, builder, note, badge):
    head = doc.add_paragraph(style="Formula Label")
    head.paragraph_format.keep_with_next = True
    r = head.add_run(f"{number:02d}  {title}")
    set_east_asia_font(r)
    r.bold = True
    r.font.color.rgb = RGBColor.from_string(DARK_BLUE)
    br = head.add_run(f"   {badge}")
    set_east_asia_font(br)
    br.font.size = Pt(8.5)
    br.font.color.rgb = RGBColor.from_string(GOLD)
    p = doc.add_paragraph(style="Text Formula Display")
    p.paragraph_format.keep_with_next = True
    builder(p)
    desc = doc.add_paragraph(style="Formula Note")
    rr = desc.add_run(note)
    set_east_asia_font(rr)


def add_legacy_eq_field(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = r" EQ \F(1,2) + \R(3,x) "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    result = OxmlElement("w:t")
    result.text = "1/2 + ∛x"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, result, end])
    set_east_asia_font(run, latin="Cambria Math")
    run.font.size = Pt(16)


def add_code_paragraph(doc, text):
    p = doc.add_paragraph(style="Code Block")
    r = p.add_run(text)
    set_east_asia_font(r, latin="Consolas", east="Microsoft YaHei")
    r.font.size = Pt(9.5)
    set_paragraph_shading(p)
    return p


def configure_styles(doc):
    styles = doc.styles
    normal = styles["Normal"]
    normal.font.name = "Calibri"
    normal.font.size = Pt(11)
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.25

    title = styles["Title"]
    title.font.name = "Calibri"
    title.font.size = Pt(30)
    title.font.bold = True
    title.font.color.rgb = RGBColor.from_string(INK)
    title._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    title.paragraph_format.space_after = Pt(8)

    subtitle = styles["Subtitle"]
    subtitle.font.name = "Calibri"
    subtitle.font.size = Pt(13)
    subtitle.font.color.rgb = RGBColor.from_string(MUTED)
    subtitle._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    subtitle.paragraph_format.space_after = Pt(18)

    for name, size, before, after, color in (
        ("Heading 1", 16, 18, 10, BLUE),
        ("Heading 2", 13, 14, 7, BLUE),
        ("Heading 3", 12, 10, 5, DARK_BLUE),
    ):
        style = styles[name]
        style.font.name = "Calibri"
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = RGBColor.from_string(color)
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True

    custom = [
        ("Kicker", 10, True, BLUE, 0, 4, 1.0),
        ("Formula Label", 10, True, DARK_BLUE, 8, 2, 1.0),
        ("Formula Display", 16, False, "000000", 0, 2, 1.0),
        ("Text Formula Display", 15, False, "000000", 0, 3, 1.0),
        ("Formula Note", 9, False, MUTED, 0, 5, 1.05),
        ("Code Block", 9.5, False, "202124", 2, 6, 1.0),
        ("Small Note", 9, False, MUTED, 0, 4, 1.1),
    ]
    for name, size, bold, color, before, after, line in custom:
        if name in styles:
            style = styles[name]
        else:
            style = styles.add_style(name, WD_STYLE_TYPE.PARAGRAPH)
        style.font.name = "Calibri"
        style.font.size = Pt(size)
        style.font.bold = bold
        style.font.color.rgb = RGBColor.from_string(color)
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.line_spacing = line

    styles["Formula Display"].paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    styles["Text Formula Display"].paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    styles["Formula Note"].paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER


def configure_page(doc):
    for section in doc.sections:
        section.page_width = Inches(8.5)
        section.page_height = Inches(11)
        section.top_margin = Inches(1)
        section.bottom_margin = Inches(1)
        section.left_margin = Inches(1)
        section.right_margin = Inches(1)
        section.header_distance = Inches(0.492)
        section.footer_distance = Inches(0.492)


def add_header_footer(section):
    header = section.header
    p = header.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run("WORD 公式格式测试集  ·  FORMULA LAB")
    set_east_asia_font(r)
    r.font.size = Pt(8.5)
    r.font.color.rgb = RGBColor.from_string(MUTED)

    footer = section.footer
    fp = footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    fp.paragraph_format.space_before = Pt(0)
    rr = fp.add_run("公式兼容性测试  |  ")
    set_east_asia_font(rr)
    rr.font.size = Pt(9)
    rr.font.color.rgb = RGBColor.from_string(MUTED)
    add_page_field(fp)


def page_break(doc):
    p = doc.add_paragraph()
    p.add_run().add_break(WD_BREAK.PAGE)


def build():
    doc = Document()
    configure_styles(doc)
    configure_page(doc)
    section = doc.sections[0]
    add_header_footer(section)

    # Cover: editorial-cover pattern with compact reference-guide styling.
    spacer = doc.add_paragraph()
    spacer.paragraph_format.space_after = Pt(66)
    kicker = doc.add_paragraph(style="Kicker")
    kicker.alignment = WD_ALIGN_PARAGRAPH.CENTER
    kr = kicker.add_run("FORMULA LAB · WORD COMPATIBILITY TEST")
    set_east_asia_font(kr)
    title = doc.add_paragraph("Word 公式格式测试集", style="Title")
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    subtitle = doc.add_paragraph("原生可编辑公式、文本公式、旧式域代码与源码表示的集中测试文档", style="Subtitle")
    subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER

    lead = doc.add_paragraph()
    lead.alignment = WD_ALIGN_PARAGRAPH.CENTER
    lead.paragraph_format.space_after = Pt(18)
    lr = lead.add_run("共 30 个示例 · 重点验证编辑、复制、显示、打印与跨软件兼容性")
    set_east_asia_font(lr)
    lr.bold = True
    lr.font.color.rgb = RGBColor.from_string(DARK_BLUE)

    doc.add_heading("格式地图", level=1)
    table = doc.add_table(rows=1, cols=3)
    table.style = "Table Grid"
    widths = [2100, 1440, 5820]
    set_table_geometry(table, widths)
    headers = ["表示方式", "可编辑性", "本文件中的测试范围"]
    for idx, text in enumerate(headers):
        cell = table.rows[0].cells[idx]
        set_cell_shading(cell, LIGHT)
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(text)
        set_east_asia_font(r)
        r.bold = True
        r.font.color.rgb = RGBColor.from_string(DARK_BLUE)
    rows = [
        ("Word 原生公式（OMML）", "高", "分式、根式、上下标、积分、求和、极限、矩阵、分段函数、重音、括号等"),
        ("Word 普通文本", "中", "Unicode 数学符号、手工上下标、数学字体、化学式"),
        ("旧式 EQ 域", "依版本", "传统域代码分式与根式；可用 F9 尝试更新"),
        ("线性源码", "源码", "LaTeX 与 MathML 字符串；便于导入或转换测试"),
    ]
    for row in rows:
        cells = table.add_row().cells
        for idx, text in enumerate(row):
            p = cells[idx].paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if idx == 1 else WD_ALIGN_PARAGRAPH.LEFT
            r = p.add_run(text)
            set_east_asia_font(r)
            r.font.size = Pt(9.5)
    set_table_geometry(table, widths)

    note = doc.add_paragraph(style="Small Note")
    note.paragraph_format.space_before = Pt(10)
    nr = note.add_run("建议测试：双击原生公式进入编辑状态；复制到新文档；另存为 PDF；用 WPS、LibreOffice 或网页预览器打开并比较。")
    set_east_asia_font(nr)
    set_paragraph_shading(note, fill="FFF8E8", border="E5C07B")

    page_break(doc)
    doc.add_heading("1  基础结构与代数", level=1)
    intro = doc.add_paragraph("以下示例均为 Word 原生 Office Math 结构，可在 Word 中逐层选择并编辑。")
    intro.runs[0] if intro.runs else None
    for r in intro.runs:
        set_east_asia_font(r)

    add_formula_block(doc, 1, "行内公式", [mr("能量关系：", plain=True), mr("E"), mr("="), mr("m"), sup([mr("c")], [mr("2")])], "公式与正文处于同一段落中，用于检查基线对齐与中英文混排。")
    add_formula_block(doc, 2, "一元二次方程求根公式", [mr("x"), mr("="), frac([mr("−"), mr("b"), mr("±"), radical([sup([mr("b")], [mr("2")]), mr("−4ac")])], [mr("2a")])], "组合测试：分式、正负号、平方与根式。")
    add_formula_block(doc, 3, "嵌套分式（连分式）", [mr("x"), mr("="), frac([mr("1")], [mr("a"), mr("+"), frac([mr("1")], [mr("b"), mr("+"), frac([mr("1")], [mr("c")])])])], "测试多层分式的高度、行距与编辑层级。")
    add_formula_block(doc, 4, "n 次根与指数", [radical([mr("x"), mr("+"), sup([mr("y")], [mr("2")])], degree=[mr("3")]), mr("="), sup([delim([mr("x"), mr("+"), sup([mr("y")], [mr("2")])])], [frac([mr("1")], [mr("3")])])], "测试带根指数的根式，以及分数指数。")
    add_formula_block(doc, 5, "上下标与多重索引", [subsup([mr("T")], [mr("ij")], [mr("(k)")]), mr("="), sub([mr("A")], [mr("i1")]), sub([mr("B")], [mr("1j")]), mr("+⋯+"), sub([mr("A")], [mr("in")]), sub([mr("B")], [mr("nj")])], "测试下标、上标、上下标同时存在，以及省略号。")
    add_formula_block(doc, 6, "二项式系数", [delim([frac([mr("n!")], [mr("k!(n−k)!")])], begin="[", end="]"), mr("="), frac([mr("n!")], [mr("k!(n−k)!")])], "左侧使用方括号定界，右侧使用结构化分式。")

    page_break(doc)
    doc.add_heading("2  微积分、极限与大型运算符", level=1)
    add_formula_block(doc, 7, "有限求和", [nary("∑", [frac([mr("1")], [sup([mr("i")], [mr("2")])])], lower=[mr("i=1")], upper=[mr("n")]), mr("→"), frac([sup([mr("π")], [mr("2")])], [mr("6")])], "测试求和符号的上下限、分式和趋近箭头。")
    add_formula_block(doc, 8, "有限乘积", [nary("∏", [delim([mr("1"), mr("+"), frac([mr("x")], [mr("k")])])], lower=[mr("k=1")], upper=[mr("n")])], "测试乘积符号、上下限和自动伸缩括号。")
    add_formula_block(doc, 9, "定积分与无穷上限", [nary("∫", [sup([mr("e")], [mr("−"), sup([mr("x")], [mr("2")])]), mr("d"), mr("x")], lower=[mr("0")], upper=[mr("∞")]), mr("="), frac([radical([mr("π")])], [mr("2")])], "测试积分、无穷符号、指数与根式结果。")
    add_formula_block(doc, 10, "二重积分", [nary("∫", [nary("∫", [func("f", [delim([mr("x,y")])]), mr("dA")])], lower=[mr("D")]), mr("="), nary("∫", [nary("∫", [mr("f(x,y)"), mr("dy")], lower=[mr("c")], upper=[mr("d")]), mr("dx")], lower=[mr("a")], upper=[mr("b")])], "测试嵌套大型运算符与函数名。")
    add_formula_block(doc, 11, "极限", [limit_low([mr("lim", plain=True)], [mr("x→0")]), frac([func("sin", [mr("x")])], [mr("x")]), mr("=1")], "测试下置极限、函数名与分式。")
    add_formula_block(doc, 12, "导数与偏导数", [frac([sup([mr("d")], [mr("2")]), mr("y")], [mr("d"), sup([mr("x")], [mr("2")])]), mr("+"), frac([sup([mr("∂")], [mr("2")]), mr("u")], [mr("∂x∂y")]), mr("=0")], "测试普通导数、偏导数与二阶符号。")
    add_formula_block(doc, 13, "微分方程", [frac([mr("dy")], [mr("dx")]), mr("+"), mr("P(x)y"), mr("="), mr("Q(x)")], "测试线性常微分方程的典型排版。")

    doc.add_heading("3  线性代数、矩阵与方程组", level=1)
    add_formula_block(doc, 14, "圆括号矩阵", [mr("A="), delim([matrix([[[mr("1")], [mr("2")], [mr("3")]], [[mr("0")], [mr("1")], [mr("4")]], [[mr("5")], [mr("6")], [mr("0")]]])])], "测试 3×3 矩阵、列对齐与自动伸缩圆括号。")
    add_formula_block(doc, 15, "行列式", [delim([matrix([[[mr("a")], [mr("b")]], [[mr("c")], [mr("d")]]])], begin="|", end="|"), mr("=ad−bc")], "测试竖线定界符与 2×2 行列式。")
    add_formula_block(doc, 16, "向量与转置", [accent([mr("v")], char="⃗"), mr("="), delim([matrix([[[mr("x")]], [[mr("y")]], [[mr("z")]]])], begin="[", end="]"), mr(","), sup([mr("A")], [mr("T")]), mr("A=I")], "测试向量箭头、列向量、方括号和转置上标。")
    add_formula_block(doc, 17, "分段函数", [mr("f(x)="), delim([matrix([[[sup([mr("x")], [mr("2")])], [mr(",  x≥0")]], [[mr("−x")], [mr(",  x<0")]]])], begin="{", end="")], "测试左大括号、两行公式阵列与条件文本。")
    add_formula_block(doc, 18, "联立方程", [delim([matrix([[[mr("2x+3y=7")]], [[mr("x−y=1")]]])], begin="{", end="")], "测试方程组的纵向对齐与大括号。")
    add_formula_block(doc, 19, "特征值问题", [mr("A"), accent([mr("v")], char="⃗"), mr("="), mr("λ"), accent([mr("v")], char="⃗"), mr(",  det"), delim([mr("A−λI")]), mr("=0")], "测试希腊字母、向量重音、函数文本和行列式记号。")
    add_formula_block(doc, 20, "范数与内积", [delim([mr("x")], begin="‖", end="‖"), sub([mr("")], [mr("2")]), mr("="), radical([nary("∑", [sup([sub([mr("x")], [mr("i")])], [mr("2")])], lower=[mr("i=1")], upper=[mr("n")])]), mr(",  ⟨u,v⟩="), sup([mr("u")], [mr("T")]), mr("v")], "测试双竖线、下标范数、内积尖括号和求和根式。")

    doc.add_heading("4  高级记号与应用公式", level=1)
    add_formula_block(doc, 21, "平均值、方差与期望", [mr("μ=E"), delim([mr("X")], begin="[", end="]"), mr(",  "), sup([mr("σ")], [mr("2")]), mr("=E"), delim([sup([delim([mr("X−μ")])], [mr("2")])], begin="[", end="]")], "测试数学黑板/统计记号、方括号与复合上标。")
    add_formula_block(doc, 22, "概率密度函数", [mr("p(x)="), frac([mr("1")], [radical([mr("2π"), sup([mr("σ")], [mr("2")])])]), sup([mr("e")], [mr("−"), frac([sup([delim([mr("x−μ")])], [mr("2")])], [mr("2"), sup([mr("σ")], [mr("2")])])])], "测试正态分布中的嵌套根式、分式和复合指数。")
    add_formula_block(doc, 23, "傅里叶变换", [accent([mr("f")], char="ˆ"), delim([mr("ω")]), mr("="), nary("∫", [mr("f(t)"), sup([mr("e")], [mr("−iωt")]), mr("dt")], lower=[mr("−∞")], upper=[mr("∞")])], "测试帽重音、无穷积分和复指数。")
    add_formula_block(doc, 24, "复数共轭与欧拉公式", [bar([mr("z")]), mr("=a−bi,  "), sup([mr("e")], [mr("iθ")]), mr("=cosθ+i sinθ")], "测试顶线共轭、复指数和三角函数文本。")
    add_formula_block(doc, 25, "集合与逻辑", [mr("A∩B="), delim([mr("x∈Ω ∣ x∈A ∧ x∈B")], begin="{", end="}"), mr(",  ∀x∃y: x<y")], "测试集合括号、属于、交集、逻辑量词与条件竖线。")
    add_formula_block(doc, 26, "Maxwell 方程组", [delim([matrix([[[mr("∇·E=ρ/ε₀")]], [[mr("∇·B=0")]], [[mr("∇×E=−∂B/∂t")]], [[mr("∇×B=μ₀J+μ₀ε₀∂E/∂t")]]])], begin="{", end="")], "测试四行方程阵列、矢量微分算子与希腊下标。")

    doc.add_heading("5  非原生与源码格式对照", level=1)
    p = doc.add_paragraph("用于比较非原生格式在复制、搜索、字体替换和跨软件显示上的差异。")
    for r in p.runs:
        set_east_asia_font(r)

    def plain_unicode(p):
        r = p.add_run("∫₀^∞ e⁻ˣ² dx = √π / 2    |    Σᵢ₌₁ⁿ 1/i² = π²/6")
        set_east_asia_font(r, latin="Cambria Math")
        r.font.size = Pt(15)

    add_text_formula_block(doc, 27, "Unicode 线性文本", plain_unicode, "全部内容都是普通字符，可搜索和复制，但不会自动堆叠为专业二维公式。", "普通文本")

    def manual_scripts(p):
        r1 = p.add_run("H")
        r2 = p.add_run("2")
        r2.font.subscript = True
        r3 = p.add_run("O   +   x")
        r4 = p.add_run("2")
        r4.font.superscript = True
        r5 = p.add_run("   +   a")
        r6 = p.add_run("ij")
        r6.font.subscript = True
        for r in (r1, r2, r3, r4, r5, r6):
            set_east_asia_font(r, latin="Cambria Math")
            r.font.size = Pt(15)

    add_text_formula_block(doc, 28, "手工上下标文本", manual_scripts, "使用普通字符运行级格式实现下标/上标，适合化学式和短表达式。", "运行格式")

    head = doc.add_paragraph(style="Formula Label")
    hr = head.add_run("29  旧式 Word EQ 域   传统域代码")
    set_east_asia_font(hr)
    hr.bold = True
    hr.font.color.rgb = RGBColor.from_string(DARK_BLUE)
    eqp = doc.add_paragraph(style="Text Formula Display")
    add_legacy_eq_field(eqp)
    desc = doc.add_paragraph("这是旧式 EQ 域（代码包含分式和三次根）。在 Word 中选中后按 F9，可测试当前版本是否仍能更新。", style="Formula Note")
    for r in desc.runs:
        set_east_asia_font(r)

    label = doc.add_paragraph(style="Formula Label")
    rr = label.add_run("30  LaTeX / MathML 源码   纯文本源码")
    set_east_asia_font(rr)
    rr.bold = True
    rr.font.color.rgb = RGBColor.from_string(DARK_BLUE)
    add_code_paragraph(doc, r"LaTeX:  \int_0^\infty e^{-x^2}\,dx = \frac{\sqrt{\pi}}{2}")
    add_code_paragraph(doc, '<math><mfrac><msqrt><mi>π</mi></msqrt><mn>2</mn></mfrac></math>')

    doc.add_heading("快速验收清单", level=2)
    checklist = [
        "原生公式可编辑、可逐层选择；复制到新 Word 文档后仍保持二维结构。",
        "PDF、WPS 与 LibreOffice 中的根号、积分、矩阵括号、字体和行距正常。",
        "旧式 EQ 域可用 F9 更新；Unicode 与源码段落可全文搜索。",
    ]
    for idx, item in enumerate(checklist, 1):
        p = doc.add_paragraph()
        p.paragraph_format.left_indent = Inches(0.375)
        p.paragraph_format.first_line_indent = Inches(-0.188)
        p.paragraph_format.space_after = Pt(4)
        r = p.add_run(f"{idx}.  {item}")
        set_east_asia_font(r)

    # Apply font to any residual plain runs.
    for paragraph in doc.paragraphs:
        for run in paragraph.runs:
            if run._element.rPr is None or run._element.rPr.rFonts is None:
                set_east_asia_font(run)

    # Document metadata and compatibility hints.
    props = doc.core_properties
    props.title = "Word 公式格式测试集"
    props.subject = "Word 原生公式、普通文本公式、EQ 域与源码兼容性测试"
    props.author = "Codex"
    props.keywords = "Word, OMML, 公式, 数学, 兼容性"
    settings = doc.settings._element
    compat = settings.find(qn("w:compat"))
    if compat is None:
        compat = OxmlElement("w:compat")
        settings.append(compat)
    mode = OxmlElement("w:compatSetting")
    mode.set(qn("w:name"), "compatibilityMode")
    mode.set(qn("w:uri"), "http://schemas.microsoft.com/office/word")
    mode.set(qn("w:val"), "15")
    compat.append(mode)
    update = settings.find(qn("w:updateFields"))
    if update is None:
        update = OxmlElement("w:updateFields")
        settings.append(update)
    update.set(qn("w:val"), "true")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    doc.save(OUT)
    print(OUT)


if __name__ == "__main__":
    build()
