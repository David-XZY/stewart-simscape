from pathlib import Path
import fitz


pdf_path = Path(r"D:\NPU\Matlab_code\_docx_work\final-render\simscape_section.pdf")
out_dir = pdf_path.parent / "pages"
out_dir.mkdir(parents=True, exist_ok=True)
document = fitz.open(pdf_path)
matrix = fitz.Matrix(2.0, 2.0)
for index, page in enumerate(document, start=1):
    pixmap = page.get_pixmap(matrix=matrix, alpha=False)
    pixmap.save(out_dir / f"page-{index}.png")
print(f"pages={len(document)} out={out_dir}")
