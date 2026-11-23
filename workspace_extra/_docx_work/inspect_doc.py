from docx import Document
from docx.oxml.ns import qn


path = r"D:\NPU\并联机器人\完善\并联机器人计算力矩_LQI与CLF_CBF_QP控制研究.docx"
doc = Document(path)

print("SECTION")
for index, section in enumerate(doc.sections, start=1):
    print(
        index,
        "page",
        section.page_width,
        section.page_height,
        "margins",
        section.top_margin,
        section.right_margin,
        section.bottom_margin,
        section.left_margin,
        "header",
        section.header_distance,
        "footer",
        section.footer_distance,
    )
    print("header:", [paragraph.text for paragraph in section.header.paragraphs])
    print("footer:", [paragraph.text for paragraph in section.footer.paragraphs])

for paragraph_index in [128, 129, 130, 132, 134, 135, 170, 171, 172, 225, 226]:
    paragraph = doc.paragraphs[paragraph_index]
    paragraph_format = paragraph.paragraph_format
    print("\nP", paragraph_index, "style", paragraph.style.name, "text=", repr(paragraph.text))
    print(
        "align",
        paragraph.alignment,
        "before",
        paragraph_format.space_before,
        "after",
        paragraph_format.space_after,
        "line",
        paragraph_format.line_spacing,
        "first",
        paragraph_format.first_line_indent,
        "left",
        paragraph_format.left_indent,
        "keep",
        paragraph_format.keep_with_next,
    )
    for run_index, run in enumerate(paragraph.runs[:8]):
        rfonts = run._element.rPr.rFonts if run._element.rPr is not None else None
        print(
            " run",
            run_index,
            repr(run.text),
            "font",
            run.font.name,
            "size",
            run.font.size,
            "b",
            run.bold,
            "i",
            run.italic,
            "east",
            rfonts.get(qn("w:eastAsia")) if rfonts is not None else None,
        )

print("\nSTYLES")
for style_name in [
    "Normal",
    "Heading 1",
    "Heading 2",
    "Heading 3",
    "Caption Thesis",
    "Reference Thesis",
    "toc 1",
    "toc 2",
]:
    try:
        style = doc.styles[style_name]
        paragraph_format = style.paragraph_format
        font = style.font
        print(
            style_name,
            "type",
            style.type,
            "font",
            font.name,
            "size",
            font.size,
            "b",
            font.bold,
            "color",
            font.color.rgb,
            "align",
            paragraph_format.alignment,
            "before",
            paragraph_format.space_before,
            "after",
            paragraph_format.space_after,
            "line",
            paragraph_format.line_spacing,
            "first",
            paragraph_format.first_line_indent,
        )
    except Exception as error:
        print(style_name, error)
