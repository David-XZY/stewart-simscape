from zipfile import ZipFile
from lxml import etree
import json

PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_符号格式统一版.docx"
NS = {
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
    "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
}

root = etree.fromstring(ZipFile(PATH).read("word/document.xml"))
report = []
for index, paragraph in enumerate(root.xpath(".//w:p", namespaces=NS), start=1):
    maths = paragraph.xpath("./m:oMath", namespaces=NS)
    if not maths:
        continue
    report.append(
        {
            "paragraph": index,
            "text": "".join(paragraph.xpath(".//w:t/text()", namespaces=NS)),
            "math": [
                {
                    "tokens": math.xpath(".//m:r/m:t/text()", namespaces=NS),
                    "styles": math.xpath(".//m:rPr/m:sty/@m:val", namespaces=NS),
                }
                for math in maths
            ],
        }
    )

print(json.dumps(report, ensure_ascii=False, indent=2))
