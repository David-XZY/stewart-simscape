from zipfile import ZipFile
from lxml import etree
from docx import Document
import re

DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx"
NS = {
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
    "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
}

package = ZipFile(DOCX_PATH)
assert package.testzip() is None
root = etree.fromstring(package.read("word/document.xml"))
equations = {}
paragraphs = []
for index, paragraph in enumerate(root.xpath(".//w:p", namespaces=NS), start=1):
    text = "".join(paragraph.xpath(".//w:t/text()", namespaces=NS))
    math_count = len(paragraph.xpath("./m:oMath", namespaces=NS))
    if "(4-" in text:
        equations[index] = (text, math_count, bool(paragraph.xpath("./w:pPr/w:jc[@w:val='center']", namespaces=NS)))
    if index in {20, 22, 24, 26, 32, 39, 86}:
        paragraphs.append((index, text, math_count))

print("DISPLAY", equations)
print("INLINE", paragraphs)
assert len(equations) == 12, equations
assert all(count >= 1 and centered for _, count, centered in equations.values()), equations
assert all(count > 0 for _, _, count in paragraphs), paragraphs
document = Document(DOCX_PATH)
assert len(document.inline_shapes) == 4 and len(document.tables) == 1
assert all(shape._inline.docPr.get("descr") for shape in document.inline_shapes)
print("INLINE_OMML_STRUCTURE_OK")
