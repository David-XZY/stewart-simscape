import json
import sys
from pathlib import Path
from zipfile import ZipFile

from lxml import etree


DOCX = Path(sys.argv[1])
OUT = Path(sys.argv[2])
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
NS = {"w": W_NS, "m": M_NS}


def flattened_math(node):
    return "".join(node.xpath(".//m:t/text()", namespaces=NS)).strip()


def value(node, tag):
    result = node.xpath(f"string(.//m:{tag}/@m:val)", namespaces=NS)
    return result or None


def main():
    with ZipFile(DOCX) as archive:
        root = etree.fromstring(archive.read("word/document.xml"))

    entries = []
    for index, paragraph in enumerate(root.xpath(".//w:body//w:p", namespaces=NS)):
        equations = paragraph.xpath(".//m:oMath", namespaces=NS)
        if not equations:
            continue
        text = "".join(paragraph.xpath(".//w:t/text() | .//m:t/text()", namespaces=NS)).strip()
        style = paragraph.xpath("string(./w:pPr/w:pStyle/@w:val)", namespaces=NS) or "Normal"
        centered = paragraph.xpath("string(./w:pPr/w:jc/@w:val)", namespaces=NS) == "center"
        in_table = bool(paragraph.xpath("ancestor::w:tbl", namespaces=NS))
        math_items = []
        for math_index, equation in enumerate(equations):
            naries = equation.xpath(".//m:nary", namespaces=NS)
            math_items.append({
                "index": math_index,
                "text": flattened_math(equation),
                "nary_symbols": [value(nary, "chr") for nary in naries],
                "limit_locations": [value(nary, "limLoc") for nary in naries],
                "bold_runs": int(equation.xpath("count(.//m:sty[@m:val='b' or @m:val='bi'])", namespaces=NS)),
                "subscripts": int(equation.xpath("count(.//m:sSub)", namespaces=NS)),
                "superscripts": int(equation.xpath("count(.//m:sSup)", namespaces=NS)),
                "fractions": int(equation.xpath("count(.//m:f)", namespaces=NS)),
                "matrices": int(equation.xpath("count(.//m:m)", namespaces=NS)),
            })
        entries.append({
            "paragraph": index,
            "style": style,
            "centered": centered,
            "in_table": in_table,
            "text": text,
            "math": math_items,
        })

    OUT.write_text(json.dumps(entries, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"paragraphs_with_math": len(entries), "equations": sum(len(x["math"]) for x in entries)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
