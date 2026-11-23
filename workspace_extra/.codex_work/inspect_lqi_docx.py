from __future__ import annotations

import json
import re
import sys
import zipfile
from pathlib import Path

from docx import Document
from lxml import etree


def compact(text: str) -> str:
    return re.sub(r"\s+", " ", text).strip()


def main() -> None:
    src = Path(sys.argv[1])
    doc = Document(src)
    targets = ("lqr", "lqi", "观测", "龙伯格", "Luenberger", "黎卡提", "Riccati")
    print(f"paragraph_count={len(doc.paragraphs)}")
    for i, p in enumerate(doc.paragraphs):
        text = compact(p.text)
        if any(t.lower() in text.lower() for t in targets):
            print(json.dumps({
                "index": i,
                "style": p.style.name,
                "text": text,
                "runs": [
                    {"text": r.text, "bold": bool(r.bold), "italic": bool(r.italic)}
                    for r in p.runs if r.text
                ],
            }, ensure_ascii=False))
    print(f"section_count={len(doc.sections)}")
    for i, section in enumerate(doc.sections):
        print(json.dumps({
            "section": i,
            "page_width": section.page_width,
            "page_height": section.page_height,
            "top_margin": section.top_margin,
            "bottom_margin": section.bottom_margin,
            "left_margin": section.left_margin,
            "right_margin": section.right_margin,
        }, ensure_ascii=False))

    ns = {
        "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
        "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
    }
    with zipfile.ZipFile(src) as zf:
        root = etree.fromstring(zf.read("word/document.xml"))
    body = root.find("w:body", ns)
    print("body_sequence=")
    for i, element in enumerate(body):
        if element.tag.endswith("}p"):
            texts = element.xpath(".//w:t/text()", namespaces=ns)
            math = element.xpath(".//m:oMath//m:t/text() | .//m:oMathPara//m:t/text()", namespaces=ns)
            style = element.xpath("./w:pPr/w:pStyle/@w:val", namespaces=ns)
            if texts or math:
                print(json.dumps({"node": i, "kind": "p", "style": style[0] if style else "", "text": compact("".join(texts)), "math": compact("".join(math))}, ensure_ascii=False))
        elif element.tag.endswith("}tbl"):
            texts = element.xpath(".//w:t/text()", namespaces=ns)
            print(json.dumps({"node": i, "kind": "tbl", "text": compact(" | ".join(texts))}, ensure_ascii=False))

    for target in (48, 49, 51, 52, 54, 56, 73):
        element = body[target]
        print(f"XML_NODE_{target}=" + etree.tostring(element, encoding="unicode"))

    para_id_attr = "{http://schemas.microsoft.com/office/word/2010/wordml}paraId"
    paragraph_ids = [p.get(para_id_attr) for p in body.findall("w:p", ns) if p.get(para_id_attr)]
    duplicate_ids = sorted({value for value in paragraph_ids if paragraph_ids.count(value) > 1})
    print(json.dumps({"paragraph_id_count": len(paragraph_ids), "duplicate_paragraph_ids": duplicate_ids}, ensure_ascii=False))


if __name__ == "__main__":
    main()
