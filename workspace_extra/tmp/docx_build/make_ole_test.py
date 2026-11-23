from docx import Document

path = r"D:\NPU\Matlab_code\tmp\docx_build\ole_direct_test.docx"
doc = Document()
p = doc.add_paragraph()
p.alignment = 1
p.add_run("[[MT1]]")
doc.save(path)
print(path)
