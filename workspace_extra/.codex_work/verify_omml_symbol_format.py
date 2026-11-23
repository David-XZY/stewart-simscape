from zipfile import ZipFile

from docx import Document
from lxml import etree


DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_符号格式统一版.docx"
NS = {
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
    "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
}


def runs_with_styles(paragraph):
    return [
        (
            "".join(run.xpath("m:t/text()", namespaces=NS)),
            run.xpath("m:rPr/m:sty/@m:val", namespaces=NS)[0]
            if run.xpath("m:rPr/m:sty/@m:val", namespaces=NS)
            else None,
        )
        for run in paragraph.xpath(".//m:r", namespaces=NS)
    ]


def has_style(items, token, style):
    return any(text == token and actual == style for text, actual in items)


with ZipFile(DOCX_PATH) as package:
    assert package.testzip() is None
    root = etree.fromstring(package.read("word/document.xml"))

display = {}
for paragraph in root.xpath(".//w:p", namespaces=NS):
    text = "".join(paragraph.xpath(".//w:t/text()", namespaces=NS)).strip()
    if text.startswith("(4-") and text.endswith(")"):
        display[text] = paragraph

assert set(display) == {f"(4-{index})" for index in range(1, 13)}, sorted(display)
for paragraph in display.values():
    assert len(paragraph.xpath("./m:oMath", namespaces=NS)) == 1
    assert paragraph.xpath("./w:pPr/w:jc[@w:val='center']", namespaces=NS)

eq1 = runs_with_styles(display["(4-1)"])
assert has_style(eq1, "W", "bi") and has_style(eq1, "=M", "b") and has_style(eq1, "+C", "b")
eq2 = runs_with_styles(display["(4-2)"])
assert has_style(eq2, "J", "b") and has_style(eq2, "F", "bi") and has_style(eq2, "W", "bi")
eq4 = runs_with_styles(display["(4-4)"])
assert has_style(eq4, "δx", "bi") and has_style(eq4, "A", "b") and has_style(eq4, "B", "b")
eq5 = runs_with_styles(display["(4-5)"])
assert has_style(eq5, "x", "bi") and has_style(eq5, "u", "bi") and has_style(eq5, "Q", "b") and has_style(eq5, "R", "b")
eq9 = runs_with_styles(display["(4-9)"])
assert has_style(eq9, "F", "bi") and has_style(eq9, "K", "b") and has_style(eq9, "x", "bi")
eq12 = runs_with_styles(display["(4-12)"])
assert has_style(eq12, "F", "bi") and has_style(eq12, "W", "b") and has_style(eq12, "A", "b") and has_style(eq12, "b", "bi")

style_counts = {
    value: len(root.xpath(f".//m:rPr/m:sty[@m:val='{value}']", namespaces=NS))
    for value in ("b", "bi", "p")
}
assert style_counts["b"] > 0 and style_counts["bi"] > 0

document = Document(DOCX_PATH)
assert len(document.inline_shapes) == 4 and len(document.tables) == 1
assert all(shape._inline.docPr.get("descr") for shape in document.inline_shapes)
print("OMML_SYMBOL_FORMAT_OK", style_counts)
