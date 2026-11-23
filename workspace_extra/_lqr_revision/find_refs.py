from docx import Document
from pathlib import Path
src=Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx")
for i,p in enumerate(Document(src).paragraphs):
  t=''.join(p._p.xpath('.//w:t/text() | .//m:t/text()'))
  if '4-' in t:
    print(i, t)
