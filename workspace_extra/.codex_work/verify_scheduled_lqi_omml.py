from zipfile import ZipFile
from lxml import etree
from docx import Document
import re


DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_OMML公式统一版.docx"
NS = {
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
    "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
}


package = ZipFile(DOCX_PATH)
assert package.testzip() is None
root = etree.fromstring(package.read("word/document.xml"))
report = {}
for paragraph in root.xpath(".//w:p", namespaces=NS):
    text = "".join(paragraph.xpath(".//w:t/text()", namespaces=NS))
    match = re.search(r"\(4-(\d+)\)$", text)
    if match:
        number = match.group(1)
        report[number] = {
            "omml": bool(paragraph.xpath("./m:oMath", namespaces=NS)),
            "center": bool(paragraph.xpath("./w:pPr/w:jc[@w:val='center']", namespaces=NS)),
            "tabs": len(paragraph.xpath("./w:pPr/w:tabs/w:tab", namespaces=NS)),
        }

assert set(report) == {str(index) for index in range(1, 13)}, report
assert all(item["omml"] and item["center"] and item["tabs"] == 2 for item in report.values()), report
document = Document(DOCX_PATH)
assert len(document.inline_shapes) == 4 and len(document.tables) == 1
assert all(shape._inline.docPr.get("descr") for shape in document.inline_shapes)
print("FORMULA_OMML_LAYOUT_OK", report)
print("OMML_COUNT", len(root.xpath(".//m:oMath", namespaces=NS)))
