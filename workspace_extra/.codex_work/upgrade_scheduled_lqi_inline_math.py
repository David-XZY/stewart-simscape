from copy import deepcopy
from lxml import etree
from docx import Document
from docx.oxml import OxmlElement
from docx.oxml.ns import qn


SOURCE_DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_符号格式统一版_步骤1.docx"
OUTPUT_DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_符号格式统一版.docx"
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


def over(base, accent):
    return f"<mover accent=\"true\">{base}{mo(accent)}</mover>"


def fence(content):
    return f"<mfenced open=\"(\" close=\")\">{content}</mfenced>"


MINUS = "&#x2212;"
DOT = "&#x0307;"
DDOT = "&#x00A8;"
DELTA = "&#x03B4;"
LAMBDA = "&#x03BB;"


def q_r():
    return sub(vi("q"), mi("r"))


def qdot_r():
    return sub(over(vi("q"), DOT), mi("r"))


def qddot_r():
    return sub(over(vi("q"), DDOT), mi("r"))


def nu_r():
    return sub(vi("ν"), mi("r"))


def nudot_r():
    return sub(over(vi("ν"), DOT), mi("r"))


def f_ct():
    return sub(vi("F"), mtext("CT"))


def f_lqi():
    return sub(vi("F"), mtext("LQI"))


def f_nom():
    return sub(vi("F"), mtext("nom"))


def f_cmd():
    return sub(vi("F"), mtext("cmd"))


def a_k():
    return sub(bm("A"), mi("k"))


def b_k():
    return sub(bm("B"), mi("k"))


def k_x():
    return sub(bm("K"), mi("x"))


def k_i():
    return sub(bm("K"), mi("I"))


def x_i():
    return sub(vi("x"), mi("I"))


def x_a():
    return sub(vi("x"), mi("a"))


def e_q():
    return sub(vi("e"), mi("q"))


def x_r():
    return sub(vi("x"), mi("r"))


def delta_x():
    return vi(DELTA + "x")


def delta_f():
    return vi(DELTA + "F")


def wrap(body):
    return mathml(f"<mrow>{body}</mrow>")


FORMULAS = {
    "H(q_r,q̇_r)^{-1}J_v(q_r)^T": wrap(
        sup(fence(bm("H") + fence(q_r() + mo(",") + qdot_r())), mi(MINUS + "1"))
        + sub(bm("J"), mi("v")) + fence(q_r()) + sup(mi(""), mi("T"))
    ),
    "H(q_r,q̇_r)": wrap(bm("H") + fence(q_r() + mo(",") + qdot_r())),
    "J_v(q_r)^T": wrap(sup(sub(bm("J"), mi("v")) + fence(q_r()), mi("T"))),
    "J_v^T": wrap(sup(sub(bm("J"), mi("v")), mi("T"))),
    "F_CT(k)": wrap(f_ct() + fence(mi("k"))),
    "F_LQI(k)": wrap(f_lqi() + fence(mi("k"))),
    "F_nom(k)": wrap(f_nom() + fence(mi("k"))),
    "q̈_r(k)": wrap(qddot_r() + fence(mi("k"))),
    "q̇_r(k)": wrap(qdot_r() + fence(mi("k"))),
    "q_r(k)": wrap(q_r() + fence(mi("k"))),
    "x_r(k)": wrap(x_r() + fence(mi("k"))),
    "ν̇_r": wrap(nudot_r()),
    "ν_r": wrap(nu_r()),
    "ẋ=f(x,F)": wrap(over(vi("x"), DOT) + mo("=") + mi("f") + fence(vi("x") + mo(",") + vi("F"))),
    "δẋ=A_kδx+B_kδF": wrap(over(delta_x(), DOT) + mo("=") + a_k() + delta_x() + mo("+") + b_k() + delta_f()),
    "δx=x-x_r(k)": wrap(delta_x() + mo("=") + vi("x") + mo(MINUS) + x_r() + fence(mi("k"))),
    "δF=F-F_CT(k)": wrap(delta_f() + mo("=") + vi("F") + mo(MINUS) + f_ct() + fence(mi("k"))),
    "y=Cδx": wrap(vi("y") + mo("=") + bm("C") + delta_x()),
    "K_x(k)": wrap(k_x() + fence(mi("k"))),
    "K_I(k)": wrap(k_i() + fence(mi("k"))),
    "L(k)": wrap(bm("L") + fence(mi("k"))),
    "K(k)": wrap(bm("K") + fence(mi("k"))),
    "F_LQI": wrap(f_lqi()),
    "F_CT": wrap(f_ct()),
    "F_nom": wrap(f_nom()),
    "F_cmd": wrap(f_cmd()),
    "t_k": wrap(sub(mi("t"), mi("k"))),
    "W_r": wrap(sub(vi("W"), mi("r"))),
    "q̈_r": wrap(qddot_r()),
    "q̇_r": wrap(qdot_r()),
    "q_r": wrap(q_r()),
    "x_r": wrap(x_r()),
    "ẋ": wrap(over(vi("x"), DOT)),
    "δx": wrap(delta_x()),
    "δF": wrap(delta_f()),
    "A_k": wrap(a_k()),
    "B_k": wrap(b_k()),
    "K_x": wrap(k_x()),
    "K_I": wrap(k_i()),
    "x_I": wrap(x_i()),
    "x_a": wrap(x_a()),
    "e_q": wrap(e_q()),
    "J_v": wrap(sub(bm("J"), mi("v"))),
}


def formula_node(mathml_text, transform):
    transformed = transform(etree.fromstring(mathml_text.encode("utf-8")))
    root = transformed.getroot()
    if etree.QName(root).localname == "oMath":
        return deepcopy(root)
    if etree.QName(root).localname == "oMathPara":
        nested = root.find(".//{http://schemas.openxmlformats.org/officeDocument/2006/math}oMath")
        if nested is not None:
            return deepcopy(nested)
    raise RuntimeError("MML2OMML did not generate an oMath node")


def new_text_run(text, run_properties):
    run = OxmlElement("w:r")
    if run_properties is not None:
        run.append(deepcopy(run_properties))
    node = OxmlElement("w:t")
    node.set(qn("xml:space"), "preserve")
    node.text = text
    run.append(node)
    return run


def replace_inline_math(paragraph, transform):
    full_text = paragraph.text
    matched = []
    index = 0
    token_order = sorted(FORMULAS, key=len, reverse=True)
    while index < len(full_text):
        token = next((item for item in token_order if full_text.startswith(item, index)), None)
        if token is None:
            index += 1
        else:
            matched.append((index, index + len(token), token))
            index += len(token)
    if not matched:
        return False

    paragraph_xml = paragraph._p
    ppr = paragraph_xml.pPr
    first_run = next((child for child in paragraph_xml if child.tag == qn("w:r")), None)
    rpr = first_run.find(qn("w:rPr")) if first_run is not None else None
    for child in list(paragraph_xml):
        if child is not ppr:
            paragraph_xml.remove(child)

    cursor = 0
    for start, end, token in matched:
        if start > cursor:
            paragraph_xml.append(new_text_run(full_text[cursor:start], rpr))
        paragraph_xml.append(formula_node(FORMULAS[token], transform))
        cursor = end
    if cursor < len(full_text):
        paragraph_xml.append(new_text_run(full_text[cursor:], rpr))
    return True


transform = etree.XSLT(etree.parse(XSL_PATH))
document = Document(SOURCE_DOCX_PATH)
changed = 0
for paragraph in document.paragraphs:
    # Only prose paragraphs: displayed equations have their own formula style
    # and are already native OMML from the preceding revision.
    if paragraph.text.strip().startswith("(") and paragraph.text.strip().endswith(")"):
        continue
    if replace_inline_math(paragraph, transform):
        changed += 1

document.core_properties.title = "第四章 Stewart平台轨迹跟踪控制（计算力矩-调度LQI，文本内公式OMML统一版）"
document.save(OUTPUT_DOCX_PATH)
print(f"Updated inline-math paragraphs: {changed}")
print(OUTPUT_DOCX_PATH)
