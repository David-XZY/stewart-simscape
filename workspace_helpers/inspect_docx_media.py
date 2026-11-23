from pathlib import Path
import sys

from docx import Document
from docx.oxml.ns import qn


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    doc = Document(Path(sys.argv[1]))
    for i, p in enumerate(doc.paragraphs):
        embeds = p._p.xpath('.//a:blip/@r:embed')
        for rid in embeds:
            part = doc.part.related_parts[rid]
            print(i, rid, part.partname, part.content_type, len(part.blob))


if __name__ == "__main__":
    main()
