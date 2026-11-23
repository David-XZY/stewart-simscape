import importlib.util
import sys
from pathlib import Path

limit = int(sys.argv[1])
target = Path(sys.argv[2])
source = Path(r"D:\NPU\Matlab_code\output\build_qp_section_docx.py")
spec = importlib.util.spec_from_file_location("qp_doc", source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.OUT = target
original = module.append_equation
count = 0

def limited(document, items):
    global count
    count += 1
    if count <= limit:
        original(document, items)

module.append_equation = limited
module.build_document()
print(target)
