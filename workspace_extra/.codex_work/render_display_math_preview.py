from pathlib import Path
from zipfile import ZipFile

from lxml import etree


DOCX = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版_OMML公式重写版.docx")
XSL = Path(r"C:\Program Files\Microsoft Office\root\Office16\omml2mml.xsl")
OUT = Path(r"D:\NPU\Matlab_code\_lqr_revision\qa_mathml_display")
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
XHTML_NS = "http://www.w3.org/1999/xhtml"
NS = {"w": W_NS, "m": M_NS, "mml": "http://www.w3.org/1998/Math/MathML"}
DISPLAY = [12, 14, 15, 20, 22, 25, 27, 30, 32, 36, 39, 42, 48, 49, 51, 52, 54, 56, 108]


def main():
    with ZipFile(DOCX) as archive:
        root = etree.fromstring(archive.read("word/document.xml"))
    transform = etree.XSLT(etree.parse(str(XSL)))
    paragraphs = root.xpath(".//w:body//w:p", namespaces=NS)
    OUT.mkdir(parents=True, exist_ok=True)

    for page_index, batch_start in enumerate(range(0, len(DISPLAY), 5), 1):
        html = etree.Element(f"{{{XHTML_NS}}}html", nsmap={None: XHTML_NS})
        head = etree.SubElement(html, f"{{{XHTML_NS}}}head")
        etree.SubElement(head, f"{{{XHTML_NS}}}meta", charset="utf-8")
        style = etree.SubElement(head, f"{{{XHTML_NS}}}style")
        style.text = "body{margin:36px 70px;font-family:Cambria,serif;color:#111}.row{margin:28px 0 36px}.label{font:15px Consolas,monospace;color:#666;margin-bottom:6px}math{font-family:'Cambria Math',Cambria,serif;font-size:28px}"
        body = etree.SubElement(html, f"{{{XHTML_NS}}}body")
        for paragraph_index in DISPLAY[batch_start:batch_start + 5]:
            row = etree.SubElement(body, f"{{{XHTML_NS}}}div", attrib={"class": "row"})
            label = etree.SubElement(row, f"{{{XHTML_NS}}}div", attrib={"class": "label"})
            label.text = f"Paragraph {paragraph_index}"
            equation = paragraphs[paragraph_index].xpath("./m:oMath", namespaces=NS)[0]
            row.append(etree.fromstring(etree.tostring(transform(equation))))
        path = OUT / f"display-{page_index}.xhtml"
        path.write_bytes(etree.tostring(html, encoding="utf-8", xml_declaration=True))
        print(path)


if __name__ == "__main__":
    main()
