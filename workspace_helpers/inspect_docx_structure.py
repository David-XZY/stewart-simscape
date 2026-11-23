from pathlib import Path
import sys

from docx import Document
from docx.oxml.ns import qn


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    path = Path(sys.argv[1])
    doc = Document(path)
    print(f"paragraphs={len(doc.paragraphs)} tables={len(doc.tables)} sections={len(doc.sections)}")
    for i, p in enumerate(doc.paragraphs):
        text = p.text.replace("\t", "\\t").replace("\n", "\\n")
        has_eq = bool(p._p.xpath('.//m:oMath | .//m:oMathPara'))
        has_draw = bool(p._p.xpath('.//w:drawing | .//w:pict'))
        num = p._p.find(qn("w:pPr"))
        flags = []
        if has_eq:
            flags.append("EQ")
        if has_draw:
            flags.append("DRAW")
        math_text = "".join(p._p.xpath('.//m:t/text()'))
        if math_text:
            text = f"{text} | MATH={math_text}"
        print(f"P{i:04d}\t[{p.style.name}]\t[{','.join(flags)}]\t{text}")


if __name__ == "__main__":
    main()
