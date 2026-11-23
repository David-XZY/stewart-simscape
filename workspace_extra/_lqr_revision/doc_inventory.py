from docx import Document
from pathlib import Path

SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx")

doc = Document(SRC)
for i, p in enumerate(doc.paragraphs):
    text = p.text.replace("\t", " ")[:500]
    if any(k.lower() in text.lower() for k in ("LQR", "LQI", "二次型", "Riccati", "黎卡提", "线性化", "线性系统")):
        print(f"{i}\t{p.style.name}\t{text}")
print("PARAGRAPHS", len(doc.paragraphs), "TABLES", len(doc.tables), "SECTIONS", len(doc.sections))
