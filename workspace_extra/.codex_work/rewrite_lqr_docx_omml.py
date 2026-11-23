import shutil
import sys
from copy import deepcopy
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

from lxml import etree


SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版.docx")
OUT = SRC.with_name("第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版_OMML公式重写版.docx")
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
NS = {"w": W_NS, "m": M_NS}


def qn(prefix, name):
    return f"{{{M_NS if prefix == 'm' else W_NS}}}{name}"


def m(tag, **attrs):
    node = etree.Element(qn("m", tag))
    for key, value in attrs.items():
        node.set(qn("m", key), str(value))
    return node


def r(text, *, bold=False, plain=False):
    return {"type": "run", "text": str(text), "bold": bold, "plain": plain}


def b(text):
    return r(text, bold=True)


def p(text):
    return r(text, plain=True)


def sub(base, lower):
    return {"type": "sub", "base": base, "sub": lower}


def sup(base, upper):
    return {"type": "sup", "base": base, "sup": upper}


def subsup(base, lower, upper):
    return {"type": "subsup", "base": base, "sub": lower, "sup": upper}


def acc(base, char="\u0307"):
    return {"type": "acc", "base": base, "char": char}


def dot(base):
    return acc(base)


def ddot(base):
    return acc(acc(base))


def nary(symbol, body, lower, upper, lim_loc="subSup"):
    return {"type": "nary", "symbol": symbol, "body": body, "sub": lower, "sup": upper, "lim_loc": lim_loc}


def delim(body, left="(", right=")"):
    return {"type": "delim", "body": body, "left": left, "right": right}


def frac(num, den):
    return {"type": "frac", "num": num, "den": den}


def matrix(rows):
    return {"type": "matrix", "rows": rows}


def eqarr(rows):
    return {"type": "eqarr", "rows": rows}


def vec(name):
    return [b(name)]


def delta_vec(name):
    return [b("δ"), b(name)]


def bold_sub(name, label):
    return sub([b(name)], [p(label)])


def q_ref():
    return sub(vec("q"), [p("r")])


def q_rk():
    return sub(vec("q"), [p("r,k")])


def f_ct():
    return bold_sub("F", "CT")


def f_lqi():
    return bold_sub("F", "LQI")


def f_nom():
    return bold_sub("F", "nom")


def append_nodes(parent, items):
    for item in items:
        if isinstance(item, list):
            append_nodes(parent, item)
        else:
            parent.append(make_node(item))


def container(tag, items):
    node = m(tag)
    append_nodes(node, [items] if isinstance(items, dict) else items)
    return node


def make_node(spec):
    kind = spec["type"]
    if kind == "run":
        node = m("r")
        if spec.get("bold") or spec.get("plain"):
            props = m("rPr")
            style = "p" if spec.get("plain") else "i"
            if spec.get("bold"):
                style = "b" if spec.get("plain") else "bi"
            props.append(m("sty", val=style))
            node.append(props)
        text = m("t")
        text.text = spec["text"]
        node.append(text)
        return node

    if kind in {"sub", "sup", "subsup"}:
        tag = {"sub": "sSub", "sup": "sSup", "subsup": "sSubSup"}[kind]
        node = m(tag)
        node.append(m(f"{tag}Pr"))
        node.append(container("e", spec["base"]))
        if kind in {"sub", "subsup"}:
            node.append(container("sub", spec["sub"]))
        if kind in {"sup", "subsup"}:
            node.append(container("sup", spec["sup"]))
        return node

    if kind == "acc":
        node = m("acc")
        props = m("accPr")
        props.append(m("chr", val=spec.get("char", "\u0307")))
        node.append(props)
        base = spec["base"] if isinstance(spec["base"], list) else [spec["base"]]
        node.append(container("e", base))
        return node

    if kind == "nary":
        node = m("nary")
        props = m("naryPr")
        props.append(m("chr", val=spec["symbol"]))
        props.append(m("limLoc", val=spec.get("lim_loc", "subSup")))
        props.append(m("subHide", val="0" if spec.get("sub") else "1"))
        props.append(m("supHide", val="0" if spec.get("sup") else "1"))
        node.append(props)
        node.append(container("sub", spec.get("sub", [])))
        node.append(container("sup", spec.get("sup", [])))
        node.append(container("e", spec["body"]))
        return node

    if kind == "delim":
        node = m("d")
        props = m("dPr")
        props.append(m("begChr", val=spec.get("left", "(")))
        props.append(m("endChr", val=spec.get("right", ")")))
        node.append(props)
        node.append(container("e", spec["body"]))
        return node

    if kind == "frac":
        node = m("f")
        node.append(container("num", spec["num"]))
        node.append(container("den", spec["den"]))
        return node

    if kind == "matrix":
        node = m("m")
        props = m("mPr")
        cols = m("mcs")
        for _ in range(len(spec["rows"][0])):
            col = m("mc")
            col_props = m("mcPr")
            col_props.append(m("mcJc", val="center"))
            col.append(col_props)
            cols.append(col)
        props.append(cols)
        node.append(props)
        for row in spec["rows"]:
            row_node = m("mr")
            for cell in row:
                row_node.append(container("e", cell))
            node.append(row_node)
        return node

    if kind == "eqarr":
        node = m("eqArr")
        props = m("eqArrPr")
        props.append(m("maxDist", val="1"))
        node.append(props)
        for row in spec["rows"]:
            node.append(container("e", row))
        return node

    raise ValueError(f"Unsupported AST node: {kind}")


def omath(items):
    node = m("oMath")
    append_nodes(node, items)
    return node


def group(items):
    return [r("("), *items, r(")")]


def bracket(items):
    return [r("["), *items, r("]")]


def brace(items):
    return [r("{"), *items]


def norm(items):
    return [r("‖"), *items, r("‖")]


def power(base, exponent):
    return sup(base, exponent)


def display_formulas():
    qdot_r = dot(q_ref())
    qddot_r = ddot(q_ref())
    jv_qr = sub([b("J")], [p("v")])
    jv_qr_t = sup([jv_qr], [p("T")])
    x = vec("x")
    q = vec("q")
    qdot = dot(q)
    xdot = dot(x)
    x_rk = sub(x, [p("r,k")])
    q_rk = sub(q, [p("r,k")])
    qdot_rk = dot(q_rk)
    f_ct_k = sub([f_ct()], [p("k")])
    delta_x = delta_vec("x")
    delta_f = delta_vec("F")
    A_k = sub([b("A")], [p("k")])
    B_k = sub([b("B")], [p("k")])
    P_k = sub([b("P")], [p("k")])
    K_k = sub([b("K")], [p("k")])
    C = vec("C")
    H_qrk = [b("H"), group([q_rk])]
    Jv_qrk = [sub([b("J")], [p("v")]), group([q_rk])]
    x_a = sub(x, [p("a")])
    x_I = sub(x, [p("I")])
    e_q = sub([b("e")], [p("q")])
    K_xk = sub([b("K")], [p("x,k")])
    K_Ik = sub([b("K")], [p("I,k")])
    Q_a = sub([b("Q")], [p("a")])
    F_act = bold_sub("F", "act")
    F_raw = bold_sub("F", "raw")
    K_aw = bold_sub("K", "aw")
    F_cmd = bold_sub("F", "cmd")
    F_nom = f_nom()

    return {
        12: [
            sub([b("W")], [p("r")]), r("="), b("M"), group([q_ref()]), qddot_r,
            r("+"), b("C"), group([q_ref(), r(","), qdot_r]), qdot_r,
            r("+"), b("g"), group([q_ref()]),
        ],
        14: [jv_qr_t, f_ct(), r("="), sub([b("W")], [p("r")])],
        15: [
            f_ct(), r("="), brace([
                eqarr([
                    [power([jv_qr_t], [r("−1")]), sub([b("W")], [p("r")]), r(",  "), jv_qr_t, p("可逆;")],
                    [power([jv_qr_t], [r("†")]), sub([b("W")], [p("r")]), r(",  "), p("其他.")],
                ])
            ]),
        ],
        20: [
            x, r("="), sup(bracket([sup(q, [p("T")]), r("  "), sup(qdot, [p("T")])]), [p("T")]),
            r(",  "), q, r("="), sup(bracket([
                sub([r("p")], [r("x")]), r("  "), sub([r("p")], [r("y")]), r("  "), sub([r("p")], [r("z")]), r("  "),
                r("φ"), r("  "), r("θ"), r("  "), r("ψ")
            ]), [p("T")]), r(",  "), xdot, r("="), b("f"), group([x, r(","), vec("F")]),
        ],
        22: [
            xdot, r("="), sup(bracket([
                sup([qdot], [p("T")]), r("  "),
                group([
                    power([b("H"), group([q])], [r("−1")]),
                    bracket([sup([sub([b("J")], [p("v")]), group([q])], [p("T")]), b("F"), r("−"), sub([b("W")], [p("0")]), group([q, r(","), qdot])])
                ])
            ]), [p("T")]),
        ],
        25: [
            b("f"), group([x_rk, r("+"), *delta_x, r(","), f_ct_k, r("+"), *delta_f]), r("="),
            b("f"), group([x_rk, r(","), f_ct_k]), r("+"), A_k, *delta_x, r("+"), B_k, *delta_f,
            r("+"), p("O"), group([r("‖"), *delta_x, r("‖²+‖"), *delta_f, r("‖²")]),
        ],
        27: [dot(delta_x), r("="), A_k, *delta_x, r("+"), B_k, *delta_f, r(",  "), vec("y"), r("="), C, *delta_x],
        30: [
            A_k, group([r(":"), r(","), r("i")]), r("="),
            frac([
                b("f"), group([x_rk, r("+"), sub([r("h")], [r("i")]), sub([b("e")], [r("i")]), r(","), f_ct_k]), r("−"),
                b("f"), group([x_rk, r("−"), sub([r("h")], [r("i")]), sub([b("e")], [r("i")]), r(","), f_ct_k])
            ], [r("2"), sub([r("h")], [r("i")])]), r(",  "), r("i=1,"), r("…"), r(",12"),
        ],
        32: [
            B_k, r("="), bracket([
                matrix([
                    [[sub([b("0")], [r("6×6")])]],
                    [[power(H_qrk, [r("−1")]), sup(Jv_qrk, [p("T")])]],
                ])
            ]),
        ],
        36: [
            sub([r("J")], [p("LQR,k")]), r("="),
            nary("∫", [r("("), sup(delta_x, [p("T")]), b("Q"), *delta_x, r("+"), sup(delta_f, [p("T")]), b("R"), *delta_f, r(")"), p("d"), r("t")], [r("0", plain=True)], [r("∞", plain=True)]),
            r(",  "), sub(delta_f, [p("LQR")]), r("=−"), K_k, *delta_x,
        ],
        39: [
            sup([A_k], [p("T")]), P_k, r("+"), P_k, A_k, r("−"), P_k, B_k, power([b("R")], [r("−1")]), sup([B_k], [p("T")]), P_k, r("+"), b("Q"), r("=0,"),
            r("  "), K_k, r("="), power([b("R")], [r("−1")]), sup([B_k], [p("T")]), P_k,
        ],
        42: [
            sub([b("s")], [p("q")]), r("="), sup(bracket([r("0.03,0.03,0.03,π/180,π/180,π/180")]), [p("T")]), r(","),
            r("  "), sub([b("Q")], [p("q")]), r("="), sup([b("C")], [p("T")]), p("diag"), group([power([sub([b("s")], [p("q")])], [r("−2")])]), b("C"), r("+"), power([r("10")], [r("−9")]), sub([b("I")], [r("12")]),
            r(",  "), b("R"), r("="), p("diag"), group([power([frac([r("1")], [r("5000")])], [r("2")]), r("·"), sub([b("1")], [r("6")])]),
        ],
        48: [x_a, r("="), sup(bracket([sup(x, [p("T")]), r(","), sup(x_I, [p("T")])]), [p("T")]), r(",  "), dot(x_I), r("=−"), e_q],
        49: [
            dot(x_a), r("="),
            bracket([matrix([[[A_k], [sub([b("0")], [r("12×6")])]], [[r("−"), b("C")], [sub([b("0")], [r("6×6")])]]])]), x_a,
            r("+"), bracket([matrix([[[B_k]], [[sub([b("0")], [r("6×6")])]]])]), vec("u"),
        ],
        51: [
            sub([r("J")], [p("LQI,k")]), r("="),
            nary("∫", [r("("), sup([x_a], [p("T")]), Q_a, x_a, r("+"), sup(vec("u"), [p("T")]), b("R"), vec("u"), r(")"), p("d"), r("t")], [r("0", plain=True)], [r("∞", plain=True)]),
        ],
        52: [f_lqi(), group([r("k")]), r("=−"), K_xk, x, group([r("k")]), r("−"), K_Ik, x_I],
        54: [dot(x_I), r("=−"), e_q, r("−λ"), x_I, r("+"), K_aw, group([F_act, r("−"), F_raw])],
        56: [f_nom(), group([r("k")]), r("="), f_ct(), group([r("k")]), r("+"), f_lqi(), group([r("k")])],
        108: [
            F_cmd, r("="), sub([p("arg min")], [b("F")]), r("  "), sup([sub(norm([b("F"), r("−"), F_nom]), [b("W")])], [r("2")]), r(",  "), p("s.t."), r("  "),
            b("A"), group([q, r(","), qdot]), vec("F"), r("≤"), b("b"), group([q, r(","), qdot]),
        ],
    }


def set_math_style(run, value):
    props = run.find(qn("m", "rPr"))
    if props is None:
        props = m("rPr")
        run.insert(0, props)
    for node in list(props.findall(qn("m", "sty"))):
        props.remove(node)
    for node in list(props.findall(qn("m", "nor"))):
        props.remove(node)
    props.insert(0, m("sty", val=value))


def restyle_inline_equations(root):
    bold_symbols = {"x", "q", "F", "W", "M", "C", "g", "H", "A", "B", "P", "Q", "K", "L", "u", "y", "e", "s", "δ"}
    changed = 0
    for equation in root.xpath(".//m:oMath", namespaces=NS):
        text = "".join(equation.xpath(".//m:t/text()", namespaces=NS))
        contains_real_space = "∈R" in text
        for run in equation.xpath(".//m:r", namespaces=NS):
            run_text = "".join(run.xpath(".//m:t/text()", namespaces=NS))
            bold = run_text in bold_symbols
            bold = bold or (run_text == "R" and not contains_real_space)
            if bold:
                set_math_style(run, "bi")
                changed += 1
        for ssub in equation.xpath(".//m:sSub", namespaces=NS):
            base_text = "".join(ssub.xpath("./m:e//m:t/text()", namespaces=NS))
            if base_text == "I":
                for run in ssub.xpath("./m:e//m:r", namespaces=NS):
                    set_math_style(run, "bi")
                    changed += 1
    return changed


def replace_display_formulas(root):
    paragraphs = root.xpath(".//w:body//w:p", namespaces=NS)
    replacements = display_formulas()
    for index, items in replacements.items():
        paragraph = paragraphs[index]
        existing = paragraph.xpath("./m:oMath", namespaces=NS)
        if len(existing) != 1:
            raise ValueError(f"Paragraph {index} expected one direct oMath, found {len(existing)}")
        paragraph.replace(existing[0], omath(items))
    return sorted(replacements)


def audit(root):
    display_indices = [12, 14, 15, 20, 22, 25, 27, 30, 32, 36, 39, 42, 48, 49, 51, 52, 54, 56, 108]
    paragraphs = root.xpath(".//w:body//w:p", namespaces=NS)
    for index in display_indices:
        math = paragraphs[index].xpath("./m:oMath", namespaces=NS)
        if len(math) != 1:
            raise AssertionError(f"Display formula {index} is missing")
        if len(math[0].xpath(".//m:sty[@m:val='bi']", namespaces=NS)) == 0:
            raise AssertionError(f"Display formula {index} has no bold vector/matrix styling")
    integrals = root.xpath(".//m:nary[m:naryPr/m:chr[@m:val='∫']]", namespaces=NS)
    if not integrals:
        raise AssertionError("No structured integrals found")
    if any(n.xpath("string(./m:naryPr/m:limLoc/@m:val)", namespaces=NS) != "subSup" for n in integrals):
        raise AssertionError("An integral does not use right-side limits")
    return {
        "oMath": int(root.xpath("count(.//m:oMath)", namespaces=NS)),
        "bold_italic_runs": int(root.xpath("count(.//m:sty[@m:val='bi'])", namespaces=NS)),
        "integrals": len(integrals),
        "right_side_integrals": len(integrals),
        "display_formulas": len(display_indices),
    }


def main():
    with ZipFile(SRC) as source:
        document_xml = source.read("word/document.xml")
        root = etree.fromstring(document_xml)
        replaced = replace_display_formulas(root)
        inline_changes = restyle_inline_equations(root)
        report = audit(root)
        report["replaced_display_paragraphs"] = replaced
        report["restyled_inline_runs"] = inline_changes
        rewritten_xml = etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)

        with ZipFile(OUT, "w", ZIP_DEFLATED) as target:
            for item in source.infolist():
                target.writestr(item, rewritten_xml if item.filename == "word/document.xml" else source.read(item.filename))

    print(report)


if __name__ == "__main__":
    main()
