from zipfile import ZipFile
from lxml import etree

PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx"
NS = {"m": "http://schemas.openxmlformats.org/officeDocument/2006/math"}
root = etree.fromstring(ZipFile(PATH).read("word/document.xml"))
for index, math in enumerate(root.xpath(".//m:oMath", namespaces=NS), start=1):
    tokens = math.xpath(".//m:r/m:t/text()", namespaces=NS)
    print(index, "|".join(tokens))
