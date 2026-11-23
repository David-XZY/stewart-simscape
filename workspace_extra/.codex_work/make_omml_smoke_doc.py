from copy import deepcopy
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import sys

from docx import Document
from lxml import etree


SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版_OMML公式重写版.docx")
OUT_DIR = Path(r"D:\NPU\Matlab_code\_lqr_revision\formula_smoke_cases")
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
NS = {"w": W_NS, "m": M_NS}
DISPLAY = [12, 14, 15, 20, 22, 25, 27, 30, 32, 36, 39, 42, 48, 49, 51, 52, 54, 56, 108]


def main():
    selected = [int(value) for value in sys.argv[1].split(",")] if len(sys.argv) > 1 else DISPLAY
    out = OUT_DIR / ("smoke_" + "_".join(str(value) for value in selected) + ".docx")
    with ZipFile(SRC) as package:
        source = etree.fromstring(package.read("word/document.xml"))
    source_paragraphs = source.xpath(".//w:body//w:p", namespaces=NS)
    document = Document()
    document.add_paragraph("OMML formula smoke test")
    for index in selected:
        paragraph = document.add_paragraph()
        source_math = source_paragraphs[index].xpath("./m:oMath", namespaces=NS)[0]
        paragraph._p.append(deepcopy(source_math))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    document.save(out)
    print(out)


if __name__ == "__main__":
    main()
