from copy import deepcopy
from lxml import etree
from docx import Document
from docx.oxml import OxmlElement
from docx.oxml.ns import qn


SOURCE_DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿.docx"
OUTPUT_DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_符号格式统一版_步骤1.docx"
XSL_PATH = r"C:\Program Files\Microsoft Office\root\Office16\MML2OMML.XSL"
MATH_NS = "http://www.w3.org/1998/Math/MathML"


def mathml(body):
    return f'<math xmlns="{MATH_NS}">{body}</math>'


def mi(text):
    return f"<mi>{text}</mi>"


def vi(text):
    """Bold-italic vector symbol, matching the source MathType convention."""
    return f'<mi mathvariant="bold-italic">{text}</mi>'


def bm(text):
    """Bold matrix/gain symbol, matching the source MathType convention."""
    return f'<mi mathvariant="bold">{text}</mi>'


def mn(text):
    return f"<mn>{text}</mn>"


def mo(text):
    return f"<mo>{text}</mo>"


def mtext(text):
    return f"<mtext>{text}</mtext>"


def sub(base, index):
    return f"<msub>{base}{index}</msub>"


def sup(base, index):
    return f"<msup>{base}{index}</msup>"


def subsup(base, lower, upper):
    return f"<msubsup>{base}{lower}{upper}</msubsup>"


def over(base, accent):
    return f"<mover accent=\"true\">{base}{mo(accent)}</mover>"


def matrix(rows):
    body = "".join("<mtr>" + "".join(f"<mtd>{cell}</mtd>" for cell in row) + "</mtr>" for row in rows)
    return f"<mfenced open=\"[\" close=\"]\"><mtable>{body}</mtable></mfenced>"


def find_formula_paragraph(doc, number):
    suffix = f"({number})"
    matches = [p for p in doc.paragraphs if p.text.strip().endswith(suffix)]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one equation paragraph {suffix}; found {len(matches)}")
    return matches[0]


def make_number_run(number):
    run = OxmlElement("w:r")
    rpr = OxmlElement("w:rPr")
    rfonts = OxmlElement("w:rFonts")
    rfonts.set(qn("w:cs"), "Times New Roman")
    rpr.append(rfonts)
    run.append(rpr)
    tab = OxmlElement("w:tab")
    run.append(tab)
    text = OxmlElement("w:t")
    text.text = f"({number})"
    run.append(text)
    return run


def make_leading_tab_run():
    run = OxmlElement("w:r")
    rpr = OxmlElement("w:rPr")
    rfonts = OxmlElement("w:rFonts")
    for attr in ("ascii", "hAnsi", "cs"):
        rfonts.set(qn(f"w:{attr}"), "Cambria Math")
    rpr.append(rfonts)
    run.append(rpr)
    tab = OxmlElement("w:tab")
    run.append(tab)
    return run


def replace_equation(paragraph, number, formula_mathml, transform, formula_ppr):
    transformed = transform(etree.fromstring(formula_mathml.encode("utf-8")))
    transformed_root = transformed.getroot()
    if etree.QName(transformed_root).localname in {"oMath", "oMathPara"}:
        roots = [deepcopy(transformed_root)]
    else:
        roots = [deepcopy(item) for item in transformed_root.iterchildren()]
        if not roots:
            roots = [deepcopy(transformed_root)]
    omaths = [root for root in roots if etree.QName(root).localname in {"oMath", "oMathPara"}]
    if len(omaths) != 1:
        raise RuntimeError(f"Expected one OMML root for {number}; got {[etree.QName(root).localname for root in roots]}")

    p = paragraph._p
    if p.pPr is not None:
        p.remove(p.pPr)
    p.insert(0, deepcopy(formula_ppr))
    ppr = p.pPr
    for child in list(p):
        if child is not ppr:
            p.remove(child)
    p.append(make_leading_tab_run())
    omath = omaths[0]
    if etree.QName(omath).localname == "oMathPara":
        nested = omath.find(".//{http://schemas.openxmlformats.org/officeDocument/2006/math}oMath")
        if nested is None:
            raise RuntimeError(f"No oMath in transformed oMathPara for {number}")
        omath = deepcopy(nested)
    p.append(omath)
    p.append(make_number_run(number))


transform = etree.XSLT(etree.parse(XSL_PATH))
doc = Document(SOURCE_DOCX_PATH)
formula_ppr = deepcopy(find_formula_paragraph(doc, "4-4")._p.pPr)

# Keep the original numbering but make every displayed relation native OMML.
# Numeric character entities keep this writer ASCII-only while MML2OMML emits
# correct Greek letters, over-dots, minus signs, and integral symbols.
DELTA = "&#x03B4;"
DOT = "&#x0307;"
MINUS = "&#x2212;"
INTEGRAL = "&#x222B;"
INFINITY = "&#x221E;"
LAMBDA = "&#x03BB;"
LEQ = "&#x2264;"
NORM = "&#x2016;"

delta_x = vi(DELTA + "x")
delta_f = vi(DELTA + "F")
x_i = sub(vi("x"), mi("I"))
x_a = sub(vi("x"), mi("a"))

equations = {
    "4-4": mathml(
        "<mrow>" + over(delta_x, DOT) + mo("=") + sub(bm("A"), mi("k")) + delta_x
        + mo("+") + sub(bm("B"), mi("k")) + delta_f + mo(",") + vi("y")
        + mo("=") + bm("C") + delta_x + "</mrow>"
    ),
    "4-5": mathml(
        "<mrow>" + sub(mi("J"), mtext("LQR")) + mo("=")
        + subsup(mo(INTEGRAL), mn("0"), mo(INFINITY)) + mo("(") + sup(vi("x"), mi("T"))
        + bm("Q") + vi("x") + mo("+") + sup(vi("u"), mi("T")) + bm("R") + vi("u")
        + mo(")") + mi("d") + mi("t") + mo(",") + sub(vi("u"), mtext("LQR"))
        + mo("=") + mo(MINUS) + sub(bm("K"), mi("k")) + vi("x") + "</mrow>"
    ),
    "4-6": mathml(
        "<mrow>" + x_a + mo("=")
        + "<msup><mfenced open='[' close=']'><mrow>" + sup(vi("x"), mi("T")) + mo(",")
        + sup(x_i, mi("T")) + "</mrow></mfenced><mi>T</mi></msup>" + mo(",")
        + over(x_i, DOT) + mo("=") + mo(MINUS) + sub(vi("e"), mi("q")) + "</mrow>"
    ),
    "4-7": mathml(
        "<mrow>" + over(x_a, DOT) + mo("=")
        + matrix([[sub(bm("A"), mi("k")), mn("0")], [mo(MINUS) + bm("C"), mn("0")]])
        + x_a + mo("+") + matrix([[sub(bm("B"), mi("k"))], [mn("0")]]) + vi("u") + "</mrow>"
    ),
    "4-8": mathml(
        "<mrow>" + sub(sub(mi("J"), mtext("LQI")), mi("k")) + mo("=")
        + subsup(mo(INTEGRAL), mn("0"), mo(INFINITY)) + mo("(") + sup(x_a, mi("T"))
        + bm("Q") + x_a + mo("+") + sup(vi("u"), mi("T")) + bm("R") + vi("u")
        + mo(")") + mi("d") + mi("t") + "</mrow>"
    ),
    "4-9": mathml(
        "<mrow>" + sub(vi("F"), mtext("LQI")) + mo("(") + mi("k") + mo(")") + mo("=")
        + mo(MINUS) + sub(bm("K"), mi("x")) + mo("(") + mi("k") + mo(")")
        + over(vi("x"), "^") + mo(MINUS) + sub(bm("K"), mi("I")) + mo("(") + mi("k")
        + mo(")") + x_i + "</mrow>"
    ),
    "4-10": mathml(
        "<mrow>" + over(x_i, DOT) + mo("=") + mo(MINUS) + sub(vi("e"), mi("q"))
        + mo(MINUS) + mi(LAMBDA) + x_i + mo("+") + sub(bm("K"), mtext("aw"))
        + "<mfenced open='(' close=')'><mrow>" + sub(vi("F"), mtext("act"))
        + mo(MINUS) + sub(vi("F"), mtext("raw")) + "</mrow></mfenced></mrow>"
    ),
    "4-11": mathml(
        "<mrow>" + sub(vi("F"), mtext("nom")) + mo("(") + mi("k") + mo(")") + mo("=")
        + sub(vi("F"), mtext("CT")) + mo("(") + mi("k") + mo(")") + mo("+")
        + sub(vi("F"), mtext("LQI")) + mo("(") + mi("k") + mo(")") + "</mrow>"
    ),
    "4-12": mathml(
        "<mrow>" + sub(vi("F"), mtext("cmd")) + mo("=")
        + "<munder><mtext>arg min</mtext>" + vi("F") + "</munder>" + mo(NORM) + vi("F")
        + mo(MINUS) + sub(vi("F"), mtext("nom")) + mo(NORM) + sup(bm("W"), mn("2"))
        + mo(",") + mtext(" s.t. ") + bm("A") + "<mfenced open='(' close=')'><mrow>"
        + vi("q") + mo(",") + over(vi("q"), DOT) + "</mrow></mfenced>" + vi("F")
        + mo(LEQ) + vi("b") + "<mfenced open='(' close=')'><mrow>" + vi("q")
        + mo(",") + over(vi("q"), DOT) + "</mrow></mfenced></mrow>"
    ),
}

for number, expression in equations.items():
    replace_equation(find_formula_paragraph(doc, number), number, expression, transform, formula_ppr)

doc.save(OUTPUT_DOCX_PATH)
print(OUTPUT_DOCX_PATH)
