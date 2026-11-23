from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn


root = Path(r"D:\NPU\Matlab_code")
source = next(path for path in root.glob("*Simscape*.docx") if not path.name.startswith("~$"))
out = root / "_docx_work" / "word-render"

doc = Document(source)
for paragraph in list(doc.paragraphs):
    if paragraph._p.xpath(".//w:drawing"):
        paragraph._element.getparent().remove(paragraph._element)
doc.save(out / "diag_no_images.docx")

doc = Document(source)
for table in list(doc.tables):
    table._element.getparent().remove(table._element)
doc.save(out / "diag_no_tables.docx")

doc = Document(source)
for section in doc.sections:
    for paragraph in section.footer.paragraphs:
        for run in list(paragraph.runs):
            run._element.getparent().remove(run._element)
        paragraph.add_run("1")
doc.save(out / "diag_plain_footer.docx")
