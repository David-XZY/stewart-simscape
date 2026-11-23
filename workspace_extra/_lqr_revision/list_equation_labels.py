from docx import Document
from pathlib import Path
import re

SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx")
doc = Document(SRC)
for i, p in enumerate(doc.paragraphs):
    text = "".join(p._p.xpath('.//w:t/text() | .//m:t/text()'))
    labels = re.findall(r"\(4-\d+\)", text)
    if labels:
        print(i, labels, text)
