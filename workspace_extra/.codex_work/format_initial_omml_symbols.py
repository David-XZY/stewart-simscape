"""Match the retained early equations (4-1)--(4-3) to the new OMML convention."""

from copy import deepcopy
from pathlib import Path
from tempfile import NamedTemporaryFile
from zipfile import ZIP_DEFLATED, ZipFile

from lxml import etree


DOCX_PATH = Path(
    r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_符号格式统一版.docx"
)
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
NS = {"w": W_NS, "m": M_NS}


def set_math_style(run, value):
    rpr = run.find(f"{{{M_NS}}}rPr")
    if rpr is None:
        rpr = etree.Element(f"{{{M_NS}}}rPr")
        run.insert(0, rpr)
    for old in rpr.findall(f"{{{M_NS}}}sty"):
        rpr.remove(old)
    style = etree.Element(f"{{{M_NS}}}sty")
    style.set(f"{{{M_NS}}}val", value)
    rpr.insert(0, style)


with ZipFile(DOCX_PATH, "r") as source:
    members = {info.filename: source.read(info.filename) for info in source.infolist()}

root = etree.fromstring(members["word/document.xml"])
for paragraph in root.xpath(".//w:p", namespaces=NS):
    paragraph_text = "".join(paragraph.xpath(".//w:t/text()", namespaces=NS)).strip()
    if paragraph_text not in {"(4-1)", "(4-2)", "(4-3)"}:
        continue
    for run in paragraph.xpath(".//m:r", namespaces=NS):
        token = "".join(run.xpath("m:t/text()", namespaces=NS))
        if token in {"q", "ν", "W", "F"} or token == "+g":
            set_math_style(run, "bi")
        elif token in {"J", "=M", "+C"}:
            set_math_style(run, "b")

members["word/document.xml"] = etree.tostring(
    root, xml_declaration=True, encoding="UTF-8", standalone=True
)

with NamedTemporaryFile(suffix=".docx", delete=False, dir=DOCX_PATH.parent) as temporary:
    temp_path = Path(temporary.name)

try:
    with ZipFile(temp_path, "w", ZIP_DEFLATED) as target:
        for name, payload in members.items():
            target.writestr(name, payload)
    temp_path.replace(DOCX_PATH)
finally:
    if temp_path.exists():
        temp_path.unlink()

print(DOCX_PATH)
