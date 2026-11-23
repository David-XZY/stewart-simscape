from pathlib import Path
from zipfile import ZipFile

from lxml import etree


DOCX = Path(r"D:\NPU\Matlab_code\output\word_formula_skill_test\LQR_OMML_side_limits_bold_test.docx")
XSL = Path(r"C:\Program Files\Microsoft Office\root\Office16\omml2mml.xsl")
OUT = Path(r"D:\NPU\Matlab_code\output\word_formula_skill_test\qa_mathml\preview.xhtml")
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
XHTML_NS = "http://www.w3.org/1999/xhtml"


def main():
    with ZipFile(DOCX) as archive:
        document = etree.fromstring(archive.read("word/document.xml"))
    equation = document.xpath("//m:oMath", namespaces={"m": M_NS})[0]
    mathml = etree.fromstring(etree.tostring(etree.XSLT(etree.parse(str(XSL)))(equation)))
    html = etree.Element(f"{{{XHTML_NS}}}html", nsmap={None: XHTML_NS})
    head = etree.SubElement(html, f"{{{XHTML_NS}}}head")
    etree.SubElement(head, f"{{{XHTML_NS}}}meta", charset="utf-8")
    style = etree.SubElement(head, f"{{{XHTML_NS}}}style")
    style.text = "body{margin:0;display:grid;place-items:center;height:100vh;background:#fff} math{font-family:'Cambria Math',Cambria,serif;font-size:48px}"
    body = etree.SubElement(html, f"{{{XHTML_NS}}}body")
    body.append(mathml)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(etree.tostring(html, encoding="utf-8", xml_declaration=True))
    print(OUT)


if __name__ == "__main__":
    main()
