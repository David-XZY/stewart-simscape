from pathlib import Path
from zipfile import ZipFile

SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx")
OUT = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版.docx")

with ZipFile(SRC) as source, ZipFile(OUT) as output:
    source_names = set(source.namelist())
    output_names = set(output.namelist())
    print("missing", sorted(source_names-output_names))
    print("added", sorted(output_names-source_names))
    changed = []
    for name in sorted(source_names & output_names):
        if source.read(name) != output.read(name):
            changed.append(name)
    print("changed", changed)
