from zipfile import ZipFile
from lxml import etree
import re


DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx"
NS = {"w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main"}
PATTERN = re.compile(r"(?:[A-Za-zδν]+_[A-Za-z]+(?:\([^)]*\))?|[A-Za-zδν]+_[A-Za-z]+\^T)")


root = etree.fromstring(ZipFile(DOCX_PATH).read("word/document.xml"))
hits = []
for index, paragraph in enumerate(root.xpath(".//w:p", namespaces=NS), start=1):
    # Only normal Word text is inspected. Math text resides in m:t and is
    # intentionally excluded from this residual-text audit.
    text = "".join(paragraph.xpath("./w:r//w:t/text()", namespaces=NS))
    if PATTERN.search(text):
        hits.append((index, text))

assert not hits, hits
print("NO_PLAIN_TEXT_SUBSCRIPT_FORMULAS")
