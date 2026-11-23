from __future__ import annotations

from pathlib import Path
from zipfile import ZipFile

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt
from lxml import etree


OUT = Path(r"D:\NPU\Matlab_code\output\documents\第四章_4.3_Stewart平台严格CLF-CBF-QP在线控制.docx")
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"


def m(tag: str, **attrs):
    node = OxmlElement(f"m:{tag}")
    for key, value in attrs.items():
        node.set(qn(f"m:{key}"), str(value))
    return node


def box(tag: str, items: list[dict]):
    node = m(tag)
    for item in items:
        node.append(make_node(item))
    return node


def run(text: str, *, plain: bool = False, bold: bool = False) -> dict:
    return {"type": "run", "text": text, "plain": plain, "bold": bold}


def sub(base: list[dict], below: list[dict]) -> dict:
    return {"type": "sub", "base": base, "sub": below}


def sup(base: list[dict], above: list[dict]) -> dict:
    return {"type": "sup", "base": base, "sup": above}


def subsup(base: list[dict], below: list[dict], above: list[dict]) -> dict:
    return {"type": "subsup", "base": base, "sub": below, "sup": above}


def accent(base: list[dict], char: str = "˙") -> dict:
    return {"type": "accent", "base": base, "char": char}


def delim(body: list[dict], left: str = "(", right: str = ")") -> dict:
    return {"type": "delim", "body": body, "left": left, "right": right}


def frac(num: list[dict], den: list[dict]) -> dict:
    return {"type": "frac", "num": num, "den": den}


def make_node(spec: dict):
    kind = spec["type"]
    if kind == "run":
        node = m("r")
        if spec.get("plain") or spec.get("bold"):
            props = m("rPr")
            style = "p"
            if spec.get("bold"):
                style = "b" if spec.get("plain") else "bi"
            props.append(m("sty", val=style))
            node.append(props)
        text = m("t")
        text.text = str(spec["text"])
        node.append(text)
        return node
    if kind in {"sub", "sup", "subsup"}:
        tag = {"sub": "sSub", "sup": "sSup", "subsup": "sSubSup"}[kind]
        node = m(tag)
        node.append(m(f"{tag}Pr"))
        node.append(box("e", spec["base"]))
        if kind in {"sub", "subsup"}:
            node.append(box("sub", spec["sub"]))
        if kind in {"sup", "subsup"}:
            node.append(box("sup", spec["sup"]))
        return node
    if kind == "accent":
        node = m("acc")
        props = m("accPr")
        props.append(m("chr", val=spec.get("char", "˙")))
        node.append(props)
        node.append(box("e", spec["base"]))
        return node
    if kind == "frac":
        node = m("f")
        node.append(box("num", spec["num"]))
        node.append(box("den", spec["den"]))
        return node
    if kind == "delim":
        node = m("d")
        props = m("dPr")
        props.append(m("begChr", val=spec.get("left", "(")))
        props.append(m("endChr", val=spec.get("right", ")")))
        node.append(props)
        node.append(box("e", spec["body"]))
        return node
    raise ValueError(f"Unsupported node type: {kind}")


def append_equation(document: Document, items: list[dict]) -> None:
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_before = Pt(3)
    paragraph.paragraph_format.space_after = Pt(3)
    math_para = m("oMathPara")
    props = m("oMathParaPr")
    props.append(m("jc", val="center"))
    math_para.append(props)
    equation = m("oMath")
    for item in items:
        equation.append(make_node(item))
    math_para.append(equation)
    paragraph._p.append(math_para)


def set_run_font(run_obj, name: str, size: Pt, bold: bool = False):
    run_obj.font.name = name
    run_obj._element.rPr.rFonts.set(qn("w:eastAsia"), name)
    run_obj._element.rPr.rFonts.set(qn("w:ascii"), "Times New Roman")
    run_obj._element.rPr.rFonts.set(qn("w:hAnsi"), "Times New Roman")
    run_obj.font.size = size
    run_obj.bold = bold


def add_body(document: Document, text: str):
    paragraph = document.add_paragraph(style="QP Body")
    paragraph.add_run(text)
    return paragraph


def add_page_field(paragraph):
    field_char = OxmlElement("w:fldChar")
    field_char.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = " PAGE "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    text = OxmlElement("w:t")
    text.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    for child in (field_char, instr, separate, text, end):
        word_run = OxmlElement("w:r")
        word_run.append(child)
        paragraph._p.append(word_run)


def configure_document(document: Document):
    section = document.sections[0]
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.54)
    section.bottom_margin = Cm(2.54)
    section.left_margin = Cm(2.50)
    section.right_margin = Cm(2.50)
    section.header_distance = Cm(1.50)
    section.footer_distance = Cm(1.75)

    normal = document.styles["Normal"]
    normal.font.name = "宋体"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    normal.font.size = Pt(12)
    normal.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
    normal.paragraph_format.first_line_indent = Pt(24)
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(0)

    body = document.styles.add_style("QP Body", WD_STYLE_TYPE.PARAGRAPH)
    body.base_style = normal
    body.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    body.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
    body.paragraph_format.first_line_indent = Pt(24)

    heading2 = document.styles["Heading 2"]
    heading2.font.name = "黑体"
    heading2._element.rPr.rFonts.set(qn("w:eastAsia"), "黑体")
    heading2.font.size = Pt(14)
    heading2.font.bold = False
    heading2.paragraph_format.space_before = Pt(12)
    heading2.paragraph_format.space_after = Pt(6)
    heading2.paragraph_format.keep_with_next = True

    heading3 = document.styles["Heading 3"]
    heading3.font.name = "黑体"
    heading3._element.rPr.rFonts.set(qn("w:eastAsia"), "黑体")
    heading3.font.size = Pt(12)
    heading3.font.bold = False
    heading3.paragraph_format.space_before = Pt(8)
    heading3.paragraph_format.space_after = Pt(4)
    heading3.paragraph_format.keep_with_next = True

    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    footer.paragraph_format.first_line_indent = Pt(0)
    add_page_field(footer)


def build_document():
    document = Document()
    configure_document(document)

    heading = document.add_paragraph("4.3 Stewart平台QP在线控制", style="Heading 2")
    heading.paragraph_format.first_line_indent = Pt(0)

    add_body(document, "上一节的计算力矩-LQI控制器以参考动力学补偿和固定反馈律形成名义支链力，能够在标称工作点附近抑制位姿误差。但该控制力经过限幅后，无法主动协调力幅值、力变化率、支腿运动边界、机构奇异性和碰撞间隙等约束。为此，在名义力之外设置严格CLF-CBF-QP在线约束层：该层不重新估计外部扰动，而是在每个控制周期根据测得状态和模型约束对支链力进行最小修正，从而把轨迹收敛要求与安全边界统一到同一个优化问题中。控制框图见另交付的图4.4。")

    document.add_paragraph("4.3.1 控制器总体结构", style="Heading 3")
    add_body(document, "QP控制器并不替代上一节的计算力矩-LQI结构，而是将其输出作为名义输入。参考位姿、速度和加速度用于生成参考动力学补偿；位姿与速度的实际值则用于计算跟踪误差、约束余量以及约束对支链力的瞬时影响。优化器输出的支链力作为最终指令输入执行器，并将上一周期指令保留以限制力变化率。")
    append_equation(document, [
        sub([run("F", bold=True)], [run("nom", plain=True)]), run(" = "),
        sub([run("F", bold=True)], [run("CT", plain=True)]), run(" + "),
        sub([run("F", bold=True)], [run("LQI", plain=True)]),
    ])
    add_body(document, "式中，名义支链力由计算力矩前馈力和LQI反馈修正力组成；QP只在满足约束的前提下选择与该名义力最接近的可行控制力。因此，前馈补偿仍承担标称动力学载荷，在线优化负责处理当前状态下的约束协调。")

    document.add_paragraph("4.3.2 CLF跟踪约束", style="Heading 3")
    add_body(document, "在当前采样时刻，将平台的六维广义位姿、广义速度和广义加速度写成输入仿射形式。该形式显式给出支链力对加速度的作用方向，是将跟踪条件和安全条件写成关于决策变量线性不等式的基础。")
    append_equation(document, [
        run("a", bold=True), run(" = "),
        run("f", bold=True), delim([run("q", bold=True), run(", "), run("v", bold=True)]),
        run(" + "), run("G", bold=True), delim([run("q", bold=True)]), run("F", bold=True),
    ])
    add_body(document, "定义实际位姿与参考位姿之差为位姿误差，并以广义速度误差构成十二维误差状态。这里的误差变量只用于描述参考轨迹邻域内的跟踪偏差，不表示对全工作空间进行线性化。")
    append_equation(document, [
        run("e", bold=True), run(" = "), run("q", bold=True), run(" − "),
        sub([run("q", bold=True)], [run("r", plain=True)]), run(",     "),
        sub([run("e", bold=True)], [run("v", plain=True)]), run(" = "), run("v", bold=True), run(" − "),
        sub([run("v", bold=True)], [run("r", plain=True)]), run(",     "),
        run("z", bold=True), run(" = "), sup([delim([
            run("e", bold=True), run(", "), sub([run("e", bold=True)], [run("v", plain=True)])
        ], "[", "]")], [run("T", plain=True)])
    ])
    add_body(document, "选取正定矩阵构成二次型控制Lyapunov函数，并要求其沿闭环轨迹以给定速率下降。由于跟踪性能在极端接近约束边界时可以作适度让步，CLF条件引入非负松弛变量；该变量只放松收敛速度，不改变任何安全约束。")
    append_equation(document, [
        run("V", plain=True), delim([run("z", bold=True)]), run(" = "),
        sup([run("z", bold=True)], [run("T", plain=True)]), run("P", bold=True), run("z", bold=True),
        run(",     "), run("P", bold=True), run(" = "), sup([run("P", bold=True)], [run("T", plain=True)]), run(" ≻ 0"),
    ])
    append_equation(document, [
        sub([run("L", plain=True)], [run("f", plain=True)]), run("V", plain=True), run(" + "),
        sub([run("L", plain=True)], [run("g", plain=True)]), run("V", plain=True), run("F", bold=True), run(" ≤ −"),
        sub([run("c", plain=True)], [run("V", plain=True)]), run("V", plain=True), delim([run("z", bold=True)]), run(" + "),
        sub([run("δ", plain=True)], [run("clf", plain=True)]), run(",     "),
        sub([run("δ", plain=True)], [run("clf", plain=True)]), run(" ≥ 0"),
    ])

    document.add_paragraph("4.3.3 HOCBF安全约束", style="Heading 3")
    add_body(document, "对于腿长、碰撞距离和Jacobian最小奇异值等相对阶次为二的状态边界，直接限制边界函数本身不足以约束控制输入。采用高阶控制屏障函数递推，使最终不等式显式依赖支链力；只要优化问题可行并且初始递推条件满足，该约束形式便持续限制系统向不可行区域演化。")
    append_equation(document, [
        sub([run("ψ", plain=True)], [run("0", plain=True)]), run(" = "), run("h", plain=True), delim([run("q", bold=True)]), run(",     "),
        sub([run("ψ", plain=True)], [run("1", plain=True)]), run(" = "),
        sub([run("L", plain=True)], [run("f", plain=True)]), run("h", plain=True), run(" + "),
        sub([run("α", plain=True)], [run("1", plain=True)]), sub([run("ψ", plain=True)], [run("0", plain=True)]), run(",     "),
        sub([run("L", plain=True)], [run("f", plain=True)]), sub([run("ψ", plain=True)], [run("1", plain=True)]), run(" + "),
        sub([run("L", plain=True)], [run("g", plain=True)]), sub([run("ψ", plain=True)], [run("1", plain=True)]), run("F", bold=True), run(" + "),
        sub([run("α", plain=True)], [run("2", plain=True)]), sub([run("ψ", plain=True)], [run("1", plain=True)]), run(" ≥ 0"),
    ])
    add_body(document, "在本文控制器中，支链力的上、下限以及相邻采样周期的力变化率直接作为执行器硬约束；腿长、腿速、腿加速度、奇异性和碰撞间隙分别由相应的屏障函数转换为线性不等式。每类约束均作用于六维支链力，不对CBF引入松弛变量。")
    append_equation(document, [
        sub([run("F", bold=True)], [run("min", plain=True)]), run(" ≤ "), run("F", bold=True), run(" ≤ "), sub([run("F", bold=True)], [run("max", plain=True)]), run(",     −"),
        sub([run("Δ", plain=True), run("F", bold=True)], [run("max", plain=True)]), run(" ≤ "),
        sub([run("F", bold=True)], [run("k", plain=True)]), run(" − "), sub([run("F", bold=True)], [run("k−1", plain=True)]), run(" ≤ "),
        sub([run("Δ", plain=True), run("F", bold=True)], [run("max", plain=True)]),
    ])

    document.add_paragraph("4.3.4 QP优化与在线求解", style="Heading 3")
    add_body(document, "在每个采样周期内，以六个支链力和一个CLF松弛变量作为决策变量。目标函数第一项限制最终力偏离名义力，第二项抑制相邻周期的力突变，第三项对跟踪松弛施加较大惩罚；约束集合同时包含CLF条件、由HOCBF得到的安全不等式和执行器边界。CBF矩阵及其右端项由当前测量状态、参考轨迹和非线性模型实时计算。")
    append_equation(document, [
        sub([run("min", plain=True)], [delim([run("F", bold=True), run(", "), sub([run("δ", plain=True)], [run("clf", plain=True)]), run(" ≥ 0")])]), run("  "),
        frac([run("1", plain=True)], [run("2", plain=True)]),
        sub([run("‖"), run("F", bold=True), run(" − "), sub([run("F", bold=True)], [run("nom", plain=True)]), run("‖")], [run("W", bold=True), run("F", plain=True)]),
        sup([], [run("2", plain=True)]), run(" + "),
        frac([run("1", plain=True)], [run("2", plain=True)]),
        sub([run("‖"), run("F", bold=True), run(" − "), sub([run("F", bold=True)], [run("k−1", plain=True)]), run("‖")], [run("W", bold=True), run("ΔF", plain=True)]),
        sup([], [run("2", plain=True)]), run(" + "), sub([run("ρ", plain=True)], [run("clf", plain=True)]),
        sup([sub([run("δ", plain=True)], [run("clf", plain=True)])], [run("2", plain=True)]),
        run(";  "), run("s.t.", plain=True), run("  "),
        sub([run("A", bold=True)], [run("cbf", plain=True)]), run("F", bold=True), run(" ≤ "),
        sub([run("b", bold=True)], [run("cbf", plain=True)]),
    ])
    add_body(document, "控制周期取0.01 s。求解器首先检查力幅值和力变化率给出的可行区间，再求解二次规划并对硬约束残差进行后验检查。若优化器不可用、未收敛或后验检查失败，只能输出经幅值与变化率限制的有界回退力，同时记录求解状态；该回退路径不具备严格安全认证，不能与QP可行时的约束结论等同。")

    document.add_paragraph("4.3.5 原理小结与适用边界", style="Heading 3")
    add_body(document, "严格CLF-CBF-QP控制将计算力矩-LQI产生的性能导向名义力投影到由执行器、机构运动学和环境安全共同限定的可行集合中。其核心价值在于在线协调多个彼此耦合的约束，而非直接估计或消除模型误差和外部扰动。若要讨论严格前向不变性，还需同时满足模型与执行器匹配、初始状态满足屏障递推条件、QP持续可行、采样与零阶保持误差被计入裕量等前提；在存在参数失配、执行器滞后或求解失败时，应将结果限定为工程鲁棒性评价，而不能外推为无条件安全结论。")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    document.save(OUT)


def audit_document(path: Path):
    with ZipFile(path) as archive:
        all_names = archive.namelist()
        document_xml = archive.read("word/document.xml")
    root = etree.fromstring(document_xml)
    ns = {"m": M_NS, "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main"}
    return {
        "output": str(path),
        "oMath": int(root.xpath("count(.//m:oMath)", namespaces=ns)),
        "fractions": int(root.xpath("count(.//m:f)", namespaces=ns)),
        "subscripts": int(root.xpath("count(.//m:sSub)", namespaces=ns)),
        "superscripts": int(root.xpath("count(.//m:sSup)", namespaces=ns)),
        "accents": int(root.xpath("count(.//m:acc)", namespaces=ns)),
        "delimiters": int(root.xpath("count(.//m:d)", namespaces=ns)),
        "bold_math_runs": int(root.xpath("count(.//m:sty[@m:val='bi'])", namespaces=ns)),
        "legacy_mathtype_parts": sum("equation" in name.lower() or "oleobject" in name.lower() for name in all_names),
    }


if __name__ == "__main__":
    build_document()
    print(audit_document(OUT))
