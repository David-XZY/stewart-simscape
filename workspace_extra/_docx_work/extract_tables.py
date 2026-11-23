from docx import Document


path = r"D:\NPU\并联机器人\完善\并联机器人计算力矩_LQI与CLF_CBF_QP控制研究.docx"
doc = Document(path)

for index, table in enumerate(doc.tables, start=1):
    print(f"\n===== TABLE {index} ({len(table.rows)}x{len(table.columns)}) =====")
    for row in table.rows:
        print(" | ".join(cell.text.replace("\n", " / ").strip() for cell in row.cells))
