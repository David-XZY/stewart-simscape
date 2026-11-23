from docx import Document
from pathlib import Path

OUT = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版.docx")
doc = Document(OUT)
for index in range(17, 47):
    paragraph = doc.paragraphs[index]
    text = "".join(paragraph._p.xpath('.//w:t/text() | .//m:t/text()'))
    print(f"{index:02d}\t{paragraph.style.name}\t{text}")
