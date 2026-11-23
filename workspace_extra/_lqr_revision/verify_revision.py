from docx import Document
from pathlib import Path
import re

OUT = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版.docx")
doc = Document(OUT)
print("paragraphs", len(doc.paragraphs), "tables", len(doc.tables), "sections", len(doc.sections))
for index, paragraph in enumerate(doc.paragraphs):
    text = "".join(paragraph._p.xpath('.//w:t/text() | .//m:t/text()'))
    if "4.2.2" in text or "4.2.3" in text or "J_LQR" in text or "AₖᵀPₖ" in text:
        print(index, paragraph.style.name, text)
labels = []
for paragraph in doc.paragraphs:
    text = "".join(paragraph._p.xpath('.//w:t/text() | .//m:t/text()'))
    labels += re.findall(r"\(4-\d+\)", text)
print("labels", labels)
print("duplicates", sorted({label for label in labels if labels.count(label) > 1}))
print("omath", sum(len(p._p.xpath('.//m:oMath')) for p in doc.paragraphs))
