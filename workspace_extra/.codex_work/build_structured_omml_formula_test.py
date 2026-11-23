from pathlib import Path
from zipfile import ZipFile

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from lxml import etree


ROOT = Path(r"D:\NPU\Matlab_code")
OUT_DIR = ROOT / "output" / "word_formula_skill_test"
OUT = OUT_DIR / "Structured_OMML_LQR_Formula_Test.docx"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"


def m(tag, **attrs):
    node = OxmlElement(f"m:{tag}")
    for key, value in attrs.items():
        node.set(qn(f"m:{key}"), str(value))
    return node


def run(text, plain=False):
    node = m("r")
    if plain:
        props = m("rPr")
        props.append(m("sty", val="p"))
        node.append(props)
    text_node = m("t")
    text_node.text = text
    node.append(text_node)
    return node


def box(tag, children):
    node = m(tag)
    for child in children:
        node.append(child)
    return node


def sub(base, lower):
    node = m("sSub")
    node.append(m("sSubPr"))
    node.append(box("e", base))
    node.append(box("sub", lower))
    return node


def sup(base, upper):
    node = m("sSup")
    node.append(m("sSupPr"))
    node.append(box("e", base))
    node.append(box("sup", upper))
    return node


def nary(symbol, body, lower, upper):
    node = m("nary")
    props = m("naryPr")
    props.append(m("chr", val=symbol))
    props.append(m("limLoc", val="undOvr"))
    props.append(m("subHide", val="0"))
    props.append(m("supHide", val="0"))
    node.append(props)
    node.append(box("sub", lower))
    node.append(box("sup", upper))
    node.append(box("e", body))
    return node


def delimiters(body, left="(", right=")"):
    node = m("d")
    props = m("dPr")
    props.append(m("begChr", val=left))
    props.append(m("endChr", val=right))
    node.append(props)
    node.append(box("e", body))
    return node


def make_formula():
    delta = "\u03b4"
    infinity = "\u221e"
    jt = sub([run("J")], [run("LQR,k", plain=True)])
    dxt = sup([run(delta), run("x")], [run("T", plain=True)])
    dft = sup([run(delta), run("F")], [run("T", plain=True)])
    integrand = delimiters(
        [
            dxt,
            run("Q"),
            run(delta),
            run("x"),
            run("+"),
            dft,
            run("R"),
            run(delta),
            run("F"),
        ]
    )
    integral = nary(
        "\u222b",
        [integrand, run("d"), run("t")],
        [run("0", plain=True)],
        [run(infinity, plain=True)],
    )
    delta_f_lqr = sub([run(delta), run("F")], [run("LQR", plain=True)])
    k_k = sub([run("K")], [run("k")])
    return [
        jt,
        run("="),
        integral,
        run(","),
        delta_f_lqr,
        run("="),
        run("-"),
        k_k,
        run(delta),
        run("x"),
    ]


def append_display_omml(paragraph, items):
    math_para = m("oMathPara")
    math_para_props = m("oMathParaPr")
    math_para_props.append(m("jc", val="center"))
    math_para.append(math_para_props)
    equation = m("oMath")
    for item in items:
        equation.append(item)
    math_para.append(equation)
    paragraph._p.append(math_para)


def build():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    document = Document()
    normal = document.styles["Normal"]
    normal.font.name = "Cambria"

    title = document.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title_run = title.add_run("Structured native OMML test - LQR equation")
    title_run.bold = True
    title_run.font.size = 14

    lead = document.add_paragraph(
        "This equation is written as structured OfficeMath XML, without Word-to-MathType conversion."
    )
    lead.alignment = WD_ALIGN_PARAGRAPH.CENTER

    formula = document.add_paragraph()
    formula.alignment = WD_ALIGN_PARAGRAPH.CENTER
    append_display_omml(formula, make_formula())

    source = document.add_paragraph()
    source.add_run("Reference TeX: ").bold = True
    source.add_run(
        r"J_{\mathrm{LQR},k}=\int_0^\infty(\delta x^TQ\delta x+\delta F^TR\delta F)dt,"
        r"\quad\delta F_{\mathrm{LQR}}=-K_k\delta x"
    )
    document.save(OUT)


def audit():
    with ZipFile(OUT) as archive:
        document_xml = archive.read("word/document.xml")
    root = etree.fromstring(document_xml)
    ns = {"m": M_NS}
    counts = {
        "oMath": int(root.xpath("count(.//m:oMath)", namespaces=ns)),
        "integrals": int(root.xpath("count(.//m:nary[m:naryPr/m:chr[@m:val='\u222b']])", namespaces=ns)),
        "subscripts": int(root.xpath("count(.//m:sSub)", namespaces=ns)),
        "superscripts": int(root.xpath("count(.//m:sSup)", namespaces=ns)),
        "delimiters": int(root.xpath("count(.//m:d)", namespaces=ns)),
    }
    assert counts == {"oMath": 1, "integrals": 1, "subscripts": 3, "superscripts": 2, "delimiters": 1}
    return counts


if __name__ == "__main__":
    build()
    print(OUT)
    print(audit())
