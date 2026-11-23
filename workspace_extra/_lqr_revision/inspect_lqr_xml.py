from docx import Document
from pathlib import Path
from lxml import etree

SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx")
NS = {"w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main", "m": "http://schemas.openxmlformats.org/officeDocument/2006/math"}
doc = Document(SRC)
for i in range(14, 36):
    p = doc.paragraphs[i]
    all_text = "".join(p._p.xpath('.//w:t/text() | .//m:t/text()'))
    print("=" * 36)
    print(i, p.style.name, all_text)
    print(etree.tostring(p._p, encoding="unicode", pretty_print=True)[:10000])
