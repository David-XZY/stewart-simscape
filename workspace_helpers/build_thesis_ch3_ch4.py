from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
from pathlib import Path

from PIL import Image, ImageOps
from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt


PROJECT_ROOT = Path(r"D:\NPU\Matlab_code\stewart-simscape")
OUT_DIR = Path(r"D:\NPU\并联机器人\完善")
ASSET_DIR = OUT_DIR / "_thesis_ch3_ch4_assets"
FIG_DIR = ASSET_DIR / "figures"
DOCX_PATH = OUT_DIR / "第三章轨迹规划与第四章跟踪控制实现_论文正文稿.docx"


def ensure_dirs() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def read_key_value_summary(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def fmt_float(value: str | float, digits: int = 4) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        value = value.strip()
        if value.lower() == "nan" or value == "":
            return "不适用"
        value = float(value)
    return f"{value:.{digits}f}"


def fmt_sci(value: str | float, digits: int = 3) -> str:
    if isinstance(value, str):
        value = float(value)
    return f"{value:.{digits}e}"


def write_tikz_sources() -> None:
    ensure_dirs()
    common = r"""\documentclass[border=6pt]{standalone}
\usepackage{tikz}
\usepackage{fontspec}
\usepackage{xeCJK}
\usetikzlibrary{arrows.meta,positioning,calc,fit,backgrounds}
\setmainfont{Times New Roman}
\setCJKmainfont{Microsoft YaHei}
\tikzset{
  font=\sffamily\small,
  >=Latex,
  block/.style={draw=black, fill=gray!8, rounded corners=1pt, align=center, minimum height=10mm, text width=28mm, inner sep=2.2mm, line width=0.65pt},
  smallblock/.style={block, text width=22mm, minimum height=8mm, font=\sffamily\footnotesize},
  sum/.style={circle, draw=black, fill=white, minimum size=6.5mm, inner sep=0pt, line width=0.65pt},
  arr/.style={draw=black, line width=0.75pt, -{Latex[length=2mm]}},
  darr/.style={draw=black, dashed, line width=0.65pt, -{Latex[length=2mm]}},
  group/.style={draw=black!55, dashed, rounded corners=1.5pt, inner sep=3mm},
  note/.style={font=\sffamily\footnotesize, align=center}
}
"""
    ch3 = common + r"""
\begin{document}
\begin{tikzpicture}[node distance=12mm and 13mm]
  \node[block] (scene) {挂装场景与\\Stewart 参数模型};
  \node[block, right=of scene] (nlp) {两阶段 IHSID\\轨迹优化模型};
  \node[block, right=of nlp] (solver) {CasADi MX\\IPOPT/MA27 求解};
  \node[block, right=of solver] (check) {稠密后验\\工程约束验收};
  \node[block, right=of check] (publish) {发布 Simscape\\参考轨迹};

  \draw[arr] (scene) -- (nlp);
  \draw[arr] (nlp) -- node[note, above] {决策变量与约束} (solver);
  \draw[arr] (solver) -- node[note, above] {节点轨迹} (check);
  \draw[arr] (check) -- node[note, above] {$q,\dot q,F,L$} (publish);

  \node[smallblock, below=13mm of nlp, xshift=-20mm] (stage) {阶段 1：接近\\阶段 2：插入};
  \node[smallblock, right=of stage] (constraints) {碰撞间隙\\腿长/速度/力\\奇异性};
  \node[smallblock, right=of constraints] (accept) {求解器通过\\工程后验通过};
  \draw[darr] (stage) -- (nlp);
  \draw[darr] (constraints) -- (check);
  \draw[darr] (accept) -- (publish);

  \begin{scope}[on background layer]
    \node[group, fit=(scene)(nlp)(solver)(check)(publish)(stage)(constraints)(accept)] {};
  \end{scope}
\end{tikzpicture}
\end{document}
"""
    ch4 = common + r"""
\begin{document}
\begin{tikzpicture}[node distance=11mm and 12mm]
  \node[block] (ref) {Hermite 重建\\$q_r,\dot q_r,\ddot q_r$};
  \node[block, right=of ref] (ct) {逆动力学前馈\\$u_{CT}$};
  \node[sum, right=of ct] (sumu) {};
  \node[block, right=of sumu] (act) {非理想力执行器\\延迟/死区/速率/滞后/饱和};
  \node[block, right=of act] (plant) {Simscape\\Stewart 平台};

  \node[block, below=17mm of ct] (lqi) {LQI 输出反馈\\观测器与积分补偿};
  \node[block, below=17mm of sumu] (aw) {抗饱和回灌\\$u_{act}-u_{cmd}$};

  \coordinate[left=12mm of ref] (input);
  \coordinate[right=12mm of plant] (output);
  \draw[arr] (input) -- node[note, above] {已验收轨迹} (ref);
  \draw[arr] (ref) -- (ct);
  \draw[arr] (ct) -- node[note, above] {$+$} (sumu);
  \draw[arr] (sumu) -- node[note, above] {$u_{cmd}$} (act);
  \draw[arr] (act) -- node[note, above] {$u_{act}$} (plant);
  \draw[arr] (plant) -- node[note, above] {$q,L,F$} (output);

  \draw[darr] (plant.south) |- (lqi.east);
  \draw[arr] (lqi.north) -- node[note, left] {$u_{fb}$} (sumu.south);
  \draw[darr] (act.south) |- (aw.east);
  \draw[darr] (aw.west) -- (lqi.east);

  \node[note] at ([xshift=-1.5mm,yshift=1.5mm]sumu.center) {$+$};
  \node[note] at ([xshift=1.5mm,yshift=-1.5mm]sumu.center) {$+$};

  \begin{scope}[on background layer]
    \node[group, fit=(ref)(ct)(sumu)(act)(plant)(lqi)(aw)] {};
  \end{scope}
\end{tikzpicture}
\end{document}
"""
    (FIG_DIR / "ch3_ihsid_workflow.tex").write_text(ch3, encoding="utf-8")
    (FIG_DIR / "ch4_ct_lqi_nonideal_workflow.tex").write_text(ch4, encoding="utf-8")


def grayscale_copy(src: Path, dst: Path) -> Path | None:
    if not src.exists():
        return None
    image = Image.open(src)
    gray = ImageOps.grayscale(image).convert("RGB")
    gray.save(dst)
    return dst


def prepare_raster_assets() -> dict[str, Path]:
    ensure_dirs()
    assets: dict[str, Path] = {}
    candidates = {
        "trajectory_check": PROJECT_ROOT / "opt_minimal" / "examples" / "ihsid_40x20_limited_memory" / "trajectory_check.png",
        "tracking_pose_force": PROJECT_ROOT / "results" / "controller" / "tracking_simscape_pose_force_control_20260617_214358.png",
        "four_scheme_overview": PROJECT_ROOT / "results" / "reports" / "four_control_group_meeting_comparison_20260615_225458" / "01_四方案核心性能总览.png",
        "control_burden": PROJECT_ROOT / "results" / "reports" / "four_control_group_meeting_comparison_20260615_225458" / "04_控制负担对比.png",
    }
    for key, src in candidates.items():
        dst = FIG_DIR / f"{key}_gray.png"
        copied = grayscale_copy(src, dst)
        if copied:
            assets[key] = copied
    return assets


def set_font(run, east_asia: str = "宋体", latin: str = "Times New Roman", size: float | None = None, bold: bool | None = None) -> None:
    run.font.name = latin
    run._element.rPr.rFonts.set(qn("w:eastAsia"), east_asia)
    run._element.rPr.rFonts.set(qn("w:ascii"), latin)
    run._element.rPr.rFonts.set(qn("w:hAnsi"), latin)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold


def configure_doc_styles(doc: Document) -> None:
    section = doc.sections[0]
    section.page_width = Cm(21)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.54)
    section.bottom_margin = Cm(2.54)
    section.left_margin = Cm(2.5)
    section.right_margin = Cm(2.5)
    section.header_distance = Cm(1.5)
    section.footer_distance = Cm(1.75)

    normal = doc.styles["Normal"]
    normal.font.name = "Times New Roman"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    normal._element.rPr.rFonts.set(qn("w:ascii"), "Times New Roman")
    normal._element.rPr.rFonts.set(qn("w:hAnsi"), "Times New Roman")
    normal.font.size = Pt(12)
    normal.paragraph_format.first_line_indent = Pt(24)
    normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    normal.paragraph_format.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
    normal.paragraph_format.space_after = Pt(0)

    for name, size, before, after, align in [
        ("Heading 1", 16, 18, 12, WD_ALIGN_PARAGRAPH.CENTER),
        ("Heading 2", 14, 12, 6, WD_ALIGN_PARAGRAPH.LEFT),
        ("Heading 3", 12, 8, 4, WD_ALIGN_PARAGRAPH.LEFT),
    ]:
        style = doc.styles[name]
        style.font.name = "Arial"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "黑体")
        style._element.rPr.rFonts.set(qn("w:ascii"), "Times New Roman")
        style._element.rPr.rFonts.set(qn("w:hAnsi"), "Times New Roman")
        style.font.size = Pt(size)
        style.font.bold = False
        style.paragraph_format.first_line_indent = Pt(0)
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.alignment = align
        style.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE


def add_header_footer(doc: Document) -> None:
    section = doc.sections[0]
    header = section.header.paragraphs[0]
    header.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = header.add_run("面向载荷挂装的并联机器人轨迹规划与跟踪控制")
    set_font(run, "宋体", "Times New Roman", 9)
    p_pr = header._p.get_or_add_pPr()
    p_bdr = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), "4")
    bottom.set(qn("w:space"), "1")
    bottom.set(qn("w:color"), "000000")
    p_bdr.append(bottom)
    p_pr.append(p_bdr)

    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = footer.add_run()
    fld_begin = OxmlElement("w:fldChar")
    fld_begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = "PAGE"
    fld_end = OxmlElement("w:fldChar")
    fld_end.set(qn("w:fldCharType"), "end")
    run._r.append(fld_begin)
    run._r.append(instr)
    run._r.append(fld_end)
    set_font(run, "宋体", "Times New Roman", 9)


def add_paragraph(doc: Document, text: str = "", style: str | None = None, first_line: bool = True) -> None:
    p = doc.add_paragraph(style=style)
    p.paragraph_format.first_line_indent = Pt(24) if first_line and style is None else Pt(0)
    p.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY if style is None else p.paragraph_format.alignment
    for part in re.split(r"(`[^`]+`)", text):
        if not part:
            continue
        run = p.add_run(part.strip("`") if part.startswith("`") and part.endswith("`") else part)
        set_font(run, "宋体", "Times New Roman", 12)
        if part.startswith("`") and part.endswith("`"):
            run.font.name = "Consolas"
            run._element.rPr.rFonts.set(qn("w:eastAsia"), "黑体")


def add_heading(doc: Document, text: str, level: int) -> None:
    p = doc.add_paragraph(style=f"Heading {level}")
    run = p.add_run(text)
    set_font(run, "黑体", "Times New Roman", {1: 16, 2: 14, 3: 12}[level], False)


def add_equation(doc: Document, text: str) -> None:
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Pt(0)
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    run = p.add_run(text)
    set_font(run, "宋体", "Cambria Math", 12)


def add_caption(doc: Document, text: str, kind: str = "figure") -> None:
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Pt(0)
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(6)
    run = p.add_run(text)
    set_font(run, "宋体", "Times New Roman", 10.5)


def set_cell_text(cell, text: str, bold: bool = False, align=WD_ALIGN_PARAGRAPH.CENTER) -> None:
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
    p = cell.paragraphs[0]
    p.alignment = align
    p.paragraph_format.first_line_indent = Pt(0)
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    p.paragraph_format.space_after = Pt(0)
    p.text = ""
    run = p.add_run(str(text))
    set_font(run, "宋体", "Times New Roman", 10.5, bold)


def set_cell_border(cell, **kwargs) -> None:
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_borders = tc_pr.first_child_found_in("w:tcBorders")
    if tc_borders is None:
        tc_borders = OxmlElement("w:tcBorders")
        tc_pr.append(tc_borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        tag = f"w:{edge}"
        element = tc_borders.find(qn(tag))
        if element is None:
            element = OxmlElement(tag)
            tc_borders.append(element)
        edge_data = kwargs.get(edge)
        if edge_data is None:
            element.set(qn("w:val"), "nil")
            continue
        for key, value in edge_data.items():
            element.set(qn(f"w:{key}"), str(value))


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120) -> None:
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for m, v in {"top": top, "start": start, "bottom": bottom, "end": end}.items():
        node = tc_mar.find(qn(f"w:{m}"))
        if node is None:
            node = OxmlElement(f"w:{m}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(v))
        node.set(qn("w:type"), "dxa")


def add_three_line_table(doc: Document, caption: str, headers: list[str], rows: list[list[str]], widths: list[float]) -> None:
    add_caption(doc, caption, "table")
    table = doc.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    for idx, cell in enumerate(table.rows[0].cells):
        cell.width = Inches(widths[idx])
        set_cell_text(cell, headers[idx], bold=True)
    for row in rows:
        cells = table.add_row().cells
        for idx, text in enumerate(row):
            cells[idx].width = Inches(widths[idx])
            align = WD_ALIGN_PARAGRAPH.LEFT if idx == 0 and len(headers) > 2 else WD_ALIGN_PARAGRAPH.CENTER
            set_cell_text(cells[idx], text, align=align)
    for r_idx, row in enumerate(table.rows):
        for cell in row.cells:
            set_cell_margins(cell)
            set_cell_border(cell)
            if r_idx == 0:
                set_cell_border(cell, top={"val": "single", "sz": "12", "color": "000000"}, bottom={"val": "single", "sz": "6", "color": "000000"})
            elif r_idx == len(table.rows) - 1:
                set_cell_border(cell, bottom={"val": "single", "sz": "12", "color": "000000"})
    spacer = doc.add_paragraph()
    spacer.paragraph_format.space_after = Pt(6)


def add_image_if_exists(doc: Document, path: Path, caption: str, width: float = 5.6) -> None:
    if not path.exists():
        add_paragraph(doc, f"[待补充：图像文件未找到，原计划插入 {path.name}]")
        return
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Pt(0)
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.add_run().add_picture(str(path), width=Inches(width))
    add_caption(doc, caption)


def build_document() -> None:
    ensure_dirs()
    write_tikz_sources()
    raster_assets = prepare_raster_assets()
    traj = read_key_value_summary(PROJECT_ROOT / "opt_minimal" / "examples" / "ihsid_40x20_limited_memory" / "summary.txt")
    tracking = read_key_value_summary(PROJECT_ROOT / "results" / "controller" / "summary_simscape_pose_force_control_20260617_214358.txt")
    control_rows = read_csv_rows(PROJECT_ROOT / "results" / "reports" / "four_control_group_meeting_comparison_20260615_225458" / "四方案总体指标.csv")

    doc = Document()
    configure_doc_styles(doc)
    add_header_footer(doc)

    add_heading(doc, "第 3 章 轨迹规划", 1)
    add_heading(doc, "3.1 轨迹规划任务与总体思路", 2)
    add_paragraph(doc, "面向载荷挂装过程的并联机器人运动规划，需要同时满足平台位姿到达、障碍物避让、支链运动范围、驱动力限制以及机构远离奇异位形等约束。与只给出几何路径的规划问题相比，本文研究对象具有明显的动力学耦合特征：动平台位姿变化会同时改变六条支链长度、速度、Jacobian 以及轴向驱动力分配，若仅在笛卡尔空间生成路径，后续控制阶段可能出现腿长越界、驱动力饱和或插入阶段间隙不足等问题。")
    add_paragraph(doc, "因此，第三章将轨迹规划问题写成面向工程执行的两阶段优化问题。第一阶段用于完成圆柱体载荷向挂装区域的接近运动，重点保证平台远离顶棚和侧向障碍物；第二阶段用于完成固定姿态下的插入运动，重点约束横向偏差、高度偏差和最终装配间隙。轨迹规划只负责生成经过求解器和后验检查的参考轨迹，Simscape 多体仿真和控制器设计不进入非线性规划内部，而是在第四章作为下游验证环节读取已发布参考。")
    add_image_if_exists(doc, FIG_DIR / "ch3_ihsid_workflow.png", "图 3-1  两阶段 IHSID 轨迹规划与后验验证流程", 6.1)

    add_heading(doc, "3.2 Stewart 平台轨迹变量与两阶段运动描述", 2)
    add_paragraph(doc, "设 Stewart 平台动平台广义位姿为 q=[x,y,z,φ,θ,ψ]^T，其中前三项为平台平移，后三项为 ZYX 欧拉角。轨迹优化以平台位姿 q、广义速度 qd、支腿驱动力 Fleg 以及支链长度 L 为主要变量，并通过逆运动学和 Jacobian 将平台空间运动与六条支链的运动状态联系起来。对于第 i 条支链，其长度由上下铰点空间位置决定，支链速度由 Jacobian 与广义速度相乘得到。")
    add_equation(doc, "L_i = || p + R(q)B_i - A_i ||")
    add_equation(doc, "Ldot = J_q(q) qdot")
    add_paragraph(doc, "在该表达下，轨迹规划不再只关心动平台中心轨迹是否平滑，而是把六条支链的长度、速度、加速度和驱动力作为同一问题的工程约束。这样得到的参考轨迹能够直接转换为 Simscape 所需的位姿参考、腿长参考和驱动力前馈，为后续闭环跟踪提供统一输入。")

    add_heading(doc, "3.3 IHSID 非线性规划模型", 2)
    add_paragraph(doc, "当前代码主线采用 standard IHSID、CasADi MX、IPOPT/MA27 和 limited-memory Hessian 形成默认轨迹生成入口。主脚本 `opt_minimal/run_01_ihsid_trajectory.m` 在仓库根目录下运行，默认将结果写入 `results/trajectory_planning/`；稳定样例保存在 `opt_minimal/examples/ihsid_40x20_limited_memory/`。该入口将场景、模型、离散网格和求解器配置集中在同一脚本中，避免轨迹规划结果与控制仿真之间出现参数不一致。")
    add_paragraph(doc, "优化目标由阶段一名义路径偏差、驱动力变化率、支链加速度和奇异性指标共同构成。其作用不是追求单一最短路径，而是在满足插入任务的前提下平衡驱动力平滑性、支链运动平顺性以及机构远离奇异位形的程度。代码中对应的目标分解保存在 summary 文件中，后续结果分析也按各项贡献比例进行解释。")
    add_equation(doc, "min J = J_nominal + J_force-rate + J_leg-accel + J_singularity")
    add_three_line_table(
        doc,
        "表 3-1  IHSID 轨迹规划默认设置",
        ["项目", "当前设置"],
        [
            ["默认入口", "opt_minimal/run_01_ihsid_trajectory.m"],
            ["方法", "standard IHSID"],
            ["离散网格", f"N1={traj.get('N1')}, N2={traj.get('N2')}"],
            ["阶段时间", f"T1={fmt_float(traj.get('T1', '0'), 2)} s, T2={fmt_float(traj.get('T2', '0'), 2)} s"],
            ["求解器", "CasADi MX + IPOPT/MA27"],
            ["Hessian", "limited-memory"],
            ["安全间隙", f"{float(traj.get('safeDistance', '0'))*1000:.1f} mm"],
            ["最终间隙", f"{float(traj.get('finalGapTarget', '0'))*1000:.1f} mm"],
        ],
        [2.0, 4.0],
    )

    add_heading(doc, "3.4 求解结果与约束后验验证", 2)
    add_paragraph(doc, "求解完成后，本文不直接以求解器返回状态作为轨迹可用的唯一依据，而是进一步执行稠密后验检查。后验检查包括等式残差、不等式违反量、连续碰撞间隙、插入阶段横向误差、高度误差、姿态误差、支链长度范围、支链速度与加速度、驱动力约束以及 Jacobian 奇异性指标。只有当求解器通过且工程后验通过时，轨迹才允许导出到 Simscape 参考文件。")
    add_three_line_table(
        doc,
        "表 3-2  IHSID 轨迹规划主要数值结果",
        ["指标", "数值", "说明"],
        [
            ["求解状态", traj.get("return_status", ""), "IPOPT 返回状态"],
            ["迭代次数", traj.get("iterations", ""), "非线性规划迭代"],
            ["求解时间", f"{fmt_float(traj.get('solveTime', '0'), 2)} s", "不含后续绘图和动画"],
            ["决策变量数", traj.get("numZ", ""), "包含状态、力和碰撞分离变量"],
            ["等式约束数", traj.get("numEq", ""), "动力学、运动学和阶段约束"],
            ["不等式约束数", traj.get("numIneq", ""), "边界、碰撞、力和奇异性约束"],
            ["最大等式残差", fmt_sci(traj.get("maxEqResidual", "0")), "求解器层面"],
            ["最小安全间隙", f"{float(traj.get('minClearance', '0'))*1000:.3f} mm", "稠密后验结果"],
            ["最大支腿力", f"{fmt_float(traj.get('maxAbsForce', '0'), 2)} N", "低于模型驱动力限制"],
            ["工程后验", "通过" if traj.get("engineeringTrajectoryPassed") == "1" else "未通过", "Simscape 发布前置条件"],
        ],
        [1.75, 1.55, 2.7],
    )
    add_three_line_table(
        doc,
        "表 3-3  目标函数分解结果",
        ["目标项", "加权值", "占比"],
        [
            ["阶段一名义路径偏差", fmt_sci(traj.get("objectiveNominalStage1", "0")), f"{fmt_float(traj.get('objectiveNominalStage1Percent', '0'), 2)}%"],
            ["驱动力变化率", fmt_sci(traj.get("objectiveForceRate", "0")), f"{fmt_float(traj.get('objectiveForceRatePercent', '0'), 2)}%"],
            ["支链加速度", fmt_sci(traj.get("objectiveLegAccel", "0")), f"{fmt_float(traj.get('objectiveLegAccelPercent', '0'), 2)}%"],
            ["奇异性指标", fmt_sci(traj.get("objectiveSingularity", "0")), f"{fmt_float(traj.get('objectiveSingularityPercent', '0'), 2)}%"],
        ],
        [2.2, 1.9, 1.3],
    )
    add_image_if_exists(doc, raster_assets.get("trajectory_check", Path()), "图 3-2  两阶段 IHSID 样例轨迹与约束检查结果", 5.7)

    add_heading(doc, "3.5 面向 Simscape 的参考轨迹发布", 2)
    add_paragraph(doc, "轨迹规划结果通过 `exportTrajectoryToSimscape.m` 转换为控制仿真可直接读取的结构体。该函数保存 `refs.t`、`refs.q`、`refs.qd`、`refs.Fleg`、`refs.L` 和 `refs.q0`，并进一步生成与 Simscape From Workspace 输入兼容的 `references.r`、`references.rL` 和 `references.uFF`。其中 `references.r` 表示相对初始位姿，`references.rL` 表示相对初始腿长，`references.uFF` 表示 IHSID 输出的六腿驱动力前馈。")
    add_paragraph(doc, "由于第四章的 computed-torque 控制需要参考加速度，控制准备阶段会在节点轨迹基础上进行分段三次 Hermite 重建，同步得到 q、qd 和 qdd。这样既保持节点处位姿和速度连续，又避免简单线性插值在速度和加速度层面引入不真实突变。")
    add_equation(doc, "q(τ)=h00 qk + h10 Δt qdotk + h01 qk+1 + h11 Δt qdotk+1")
    add_paragraph(doc, "综合以上分析，第三章得到的轨迹不仅满足几何可行性，而且已经按控制仿真所需的数据契约完成发布。后续章节将以该轨迹为输入，进一步验证在多体动力学模型和非理想力执行器条件下的闭环跟踪能力。")

    doc.add_page_break()
    add_heading(doc, "第 4 章 跟踪控制实现", 1)
    add_heading(doc, "4.1 跟踪控制总体结构", 2)
    add_paragraph(doc, "第四章的目标是在第三章已发布轨迹的基础上，建立从参考位姿到 Simscape 多体模型的闭环控制链。当前 controller 主线采用 computed-torque force controller 与 nonideal-force actuator 组合，即先由参考位姿、速度和加速度计算逆动力学腿力前馈，再由 LQI 输出反馈对位姿误差进行动态补偿，最后通过含输入延迟、死区、速率限制、一阶滞后和饱和环节的非理想力执行器作用于六条支链。")
    add_image_if_exists(doc, FIG_DIR / "ch4_ct_lqi_nonideal_workflow.png", "图 4-1  computed-torque、LQI 与非理想力执行器闭环结构", 6.1)
    add_paragraph(doc, "该控制结构与中期阶段的“驱动力前馈与腿长误差反馈相结合”相比，主要变化在于反馈对象由腿长误差扩展到位姿输出反馈，前馈力由 IHSID 节点力扩展为同一时间网格上的逆动力学计算力，同时在执行器层加入非理想动态和抗饱和回灌。因此，第四章的重点不是重新规划轨迹，而是验证上游轨迹在更接近实际执行器约束的闭环模型中是否仍能满足跟踪和物理约束。")

    add_heading(doc, "4.2 参考重建与控制输入契约", 2)
    add_paragraph(doc, "控制准备函数 `prepareSimscapePoseForceControl.m` 首先读取第三章发布的 `simscape_references.mat`，检查其中是否包含 `refs`，并在存在 `publishPassed` 字段时要求其为真。随后调用 `reconstructHermitePoseReference.m` 将节点 q 和 qd 重建到 0.01 s 规则时间网格，同时计算 qdd、支链长度 L、支链速度 Ld 以及 computed-torque 前馈力 Fcomputed。")
    add_paragraph(doc, "重建后的输入被写入基础工作区：`references.r`、`references.rd` 和 `references.rdd` 分别对应位姿、速度和加速度参考；`references.uCT` 为 computed-torque 前馈力；`references.uIHSID` 保留上游优化驱动力以便对比；当控制律选择 computed-torque 时，`references.uFF` 由 `references.uCT` 覆盖。通过这种方式，原有 Simscape Reference bus 得到兼容扩展，旧控制入口也可以通过零占位字段保持运行。")

    add_heading(doc, "4.3 computed-torque 腿力前馈", 2)
    add_paragraph(doc, "computed-torque 前馈由 `computeComputedTorqueForce.m` 计算。该函数首先在参考位姿处计算 Stewart 平台 Jacobian，然后根据参考位姿、速度和加速度得到平台所需合成广义力 Wreq，最后求解 Jv(q)^T F = Wreq 得到六腿轴向力。若矩阵条件较差，则使用伪逆求解；求解结果再按照执行器上下限进行饱和。")
    add_equation(doc, "J_v(q)^T F = W_req(q, qdot, qddot)")
    add_paragraph(doc, "与把 PD 项混入前馈的写法不同，当前 helper 只承担纯逆动力学前馈计算，误差修正交给后续 LQI 动态反馈。这样的划分使前馈项能够解释为模型给出的名义驱动力需求，也便于在结果分析中区分前馈承担的重力和运动载荷补偿，以及反馈承担的模型误差和执行器误差修正。")

    add_heading(doc, "4.4 LQI 输出反馈与抗饱和设计", 2)
    add_paragraph(doc, "LQI 控制器由 `designLqiPoseForceController.m` 生成。设计对象为 6×6 的力到位姿线性化模型，代码首先要求对象严格真有理，并根据平移误差、旋转误差、速度误差、积分误差和控制量尺度构造权重矩阵。LQI 增广状态包含平台线性化状态和六维积分状态，输出反馈观测器由对偶 LQR 形式得到。")
    add_paragraph(doc, "为了适应非理想执行器的饱和和速率限制，控制器引入实际执行量与原始命令之间的差值作为第二组输入，并通过积分抗饱和增益回灌到积分状态。该设计避免在执行器无法继续增加输出时积分项继续累积，从而降低长时间饱和导致的恢复滞后。")
    add_equation(doc, "u_cmd = u_CT + u_fb")
    add_equation(doc, "xI_dot = -e_q + Kaw (u_act - u_cmd) - λ xI")
    add_three_line_table(
        doc,
        "表 4-1  力输入位姿跟踪控制主要配置",
        ["参数", "当前值", "作用"],
        [
            ["控制律", "computed-torque", "逆动力学前馈"],
            ["反馈律", "lqi-output", "位姿输出动态反馈"],
            ["执行器模式", "nonideal-force", "加入非理想力执行器动态"],
            ["带宽参数", f"{fmt_float(tracking.get('bandwidthHz', '15'), 1)} Hz", "线性化与滤波配置"],
            ["增益缩放", tracking.get("gainScale", "0.45"), "平移通道反馈缩放"],
            ["旋转增益缩放", tracking.get("rotationGainScale", "1.4"), "姿态通道反馈缩放"],
            ["反馈力限幅", "120 N", "限制 LQI 修正量"],
            ["抗饱和时间常数", "0.02 s", "积分回灌速度"],
        ],
        [1.7, 1.5, 2.7],
    )

    add_heading(doc, "4.5 非理想力执行器建模", 2)
    add_paragraph(doc, "非理想力执行器由 `makeNonidealForceActuator.m` 构造，并在 `installSimscapeComputedTorqueForceVariants.m` 中接入 Simscape 支链模型。该执行器链路依次包含输入延迟、死区、力变化率限制、一阶滞后和饱和环节。与理想力源相比，非理想模型不能瞬时跟随目标力，因此控制器需要同时检查指令力、实际力和两者之间的误差。")
    add_paragraph(doc, "当前模型还设置了 `initialForce` 字段，在 computed-torque 模式下使用第一拍前馈力初始化执行器延迟和滞后状态。该处理对应实际系统在轨迹开始前需要建立静载支撑力的工程需求，可以避免执行器从 0 N 起步时平台在重力作用下产生初始下坠。")
    add_equation(doc, "u_act = sat( G_lag · rate( deadzone( delay(u_cmd) ) ) )")

    add_heading(doc, "4.6 完整轨迹跟踪结果与验收", 2)
    add_paragraph(doc, "仿真结果通过 `evaluateSimscapePoseForceControl.m` 解析。该函数从 `simout` 中读取实际腿长、实际位姿、控制力、反馈力、前馈力以及实际轴向力，分别计算腿长误差、位姿误差、等效位姿误差、腿速度、腿加速度和力跟踪误差。验收条件包括信号有限性、线性设计稳定性、腿长上下限、力上下限、速度限制、加速度限制以及位姿/腿长跟踪误差阈值。")
    add_three_line_table(
        doc,
        "表 4-2  当前力输入位姿跟踪结果摘要",
        ["指标", "当前值", "验收边界或说明"],
        [
            ["仿真完成", tracking.get("simulationCompleted", ""), "summary 文件记录"],
            ["线性设计", "通过" if tracking.get("linearDesignPassed") == "true" else tracking.get("linearDesignPassed", ""), "LQI/观测器稳定性"],
            ["跟踪子项", "通过" if tracking.get("trackingPassed") == "true" else tracking.get("trackingPassed", ""), "腿长、平移和姿态峰值"],
            ["最大腿长误差", f"{float(tracking.get('maxLengthTrackingPeak', '0'))*1000:.3f} mm", "非理想阈值 30 mm"],
            ["最大平移误差", f"{float(tracking.get('maxTranslationPeak', '0'))*1000:.3f} mm", "非理想阈值 30 mm"],
            ["最大姿态误差", f"{math.degrees(float(tracking.get('maxRotationPeak', '0'))):.3f} deg", "阈值 1 deg"],
            ["最大控制力", f"{fmt_float(tracking.get('maxAbsControlForce', '0'), 2)} N", "控制指令峰值"],
            ["目标-实际力 RMS", f"{fmt_float(tracking.get('targetActualForceRms', '0'), 2)} N", "非理想执行器误差"],
            ["相对基线综合 passed", tracking.get("passed", ""), "历史摘要中为 false，正文按边界解释"],
        ],
        [1.75, 1.45, 2.8],
    )
    add_image_if_exists(doc, raster_assets.get("tracking_pose_force", Path()), "图 4-2  非理想力输入位姿控制完整轨迹跟踪结果", 5.9)
    add_paragraph(doc, "从表 4-2 可以看出，当前仿真在单项约束和跟踪阈值上能够完成完整轨迹跟踪，但历史摘要中相对基线综合 `passed` 字段仍为 false。其原因在于该字段包含与旧长度级联基线的综合评分比较，而不是单纯的物理约束验收。因此，本文将该结果表述为“非理想力输入位姿控制完成了完整轨迹硬约束跟踪，但相对旧基线的综合误差评分仍有改进空间”，避免把不同层级的通过标识混为同一结论。")

    add_heading(doc, "4.7 四种控制方案对比", 2)
    add_paragraph(doc, "为了进一步说明执行器实际化、辨识反馈和 UKF 估计对控制性能的影响，当前结果目录中保存了四种方案的组会对比：纯力控制、高保真控制、真实位姿反馈辨识控制和 UKF 反馈辨识控制。该比较可以展示系统级误差随信息结构和执行器模型变化的趋势，但需要注意纯力控制为理想力源基线，其余三条为 PWM 高保真执行器链路，因此不能被解释为严格的单变量消融。")
    table_rows = []
    for row in control_rows:
        table_rows.append([
            row["方案"],
            fmt_float(row["平移RMS_mm"], 4),
            fmt_float(row["平移峰值_mm"], 4),
            fmt_float(row["旋转RMS_deg"], 4),
            fmt_float(row["旋转峰值_deg"], 4),
            fmt_float(row["输出力峰值_N"], 2),
        ])
    add_three_line_table(
        doc,
        "表 4-3  四种控制方案核心性能对比",
        ["方案", "平移 RMS/mm", "平移峰值/mm", "旋转 RMS/(°)", "旋转峰值/(°)", "力峰值/N"],
        table_rows,
        [1.55, 0.9, 0.9, 0.9, 0.9, 0.85],
    )
    add_image_if_exists(doc, raster_assets.get("four_scheme_overview", Path()), "图 4-3  四种控制方案核心性能灰度对比", 5.9)
    add_image_if_exists(doc, raster_assets.get("control_burden", Path()), "图 4-4  四种控制方案控制负担灰度对比", 5.9)
    add_paragraph(doc, "从表 4-3 可以看出，理想纯力控制误差最小，高保真执行器引入后误差明显增大，进一步使用辨识反馈和 UKF 反馈时误差继续上升。这说明当前系统误差并非只由轨迹规划造成，而是与执行器动态、可测信息、力估计误差和状态估计误差共同相关。后续若要面向实物部署，应优先围绕 PWM 饱和、采样延迟、估计器鲁棒性和执行器参数辨识开展改进，而不是仅调整轨迹优化权重。")

    add_heading(doc, "4.8 测试契约与实现边界", 2)
    add_paragraph(doc, "当前 controller 目录通过 `controller/tests/run_all_active_tests.m` 作为活动测试入口。与本章直接相关的测试包括 `test_17_pose_force_control_contract.m`、`test_18_pose_force_preparation_contract.m`、`test_19_pose_force_full_tracking.m`、`test_40_nonideal_force_actuator_contract.m` 和 `test_41_lqi_pose_force_design_contract.m`。这些测试分别约束控制器类型、参考准备字段、完整轨迹跟踪、非理想执行器参数和 LQI 设计接口。")
    add_three_line_table(
        doc,
        "表 4-4  跟踪控制实现相关测试契约",
        ["测试文件", "主要检查内容"],
        [
            ["test_17_pose_force_control_contract.m", "力输入位姿控制器接口和控制律契约"],
            ["test_18_pose_force_preparation_contract.m", "参考重建、uCT/uIHSID、非理想执行器初始化"],
            ["test_19_pose_force_full_tracking.m", "完整轨迹仿真与硬验收"],
            ["test_40_nonideal_force_actuator_contract.m", "非理想力执行器参数、方向和初始化"],
            ["test_41_lqi_pose_force_design_contract.m", "LQI 输出反馈、观测器、积分和稳定性"],
        ],
        [2.55, 3.4],
    )
    add_paragraph(doc, "需要说明的是，当前第四章结果主要说明从已发布 IHSID 轨迹到 Simscape 多体模型的闭环跟踪链路已经建立，并且非理想力执行器与 LQI/抗饱和结构已进入 controller 主线。完整 PWM 物理执行器、灰箱加残差 NARX 力估计和实物可部署 UKF 仍属于后续进一步统一到同一 Simscape 模型的工作内容。因而本文在结论中应区分理想力源基线、非理想力执行器验证和 PWM 高保真数值闭环三类结果，避免超出当前代码证据范围。")

    add_heading(doc, "4.9 本章小结", 2)
    add_paragraph(doc, "本章在第三章轨迹规划结果的基础上，完成了从参考重建、computed-torque 前馈、LQI 输出反馈、抗饱和补偿到非理想力执行器的闭环控制实现。仿真评价表明，当前控制链能够在设定阈值下完成完整轨迹的跟踪，并提供控制力、实际力、腿长、位姿、腿速度和腿加速度等多维验收指标。四方案对比进一步表明，执行器动态和状态/力估计误差会显著影响最终跟踪精度，是后续实物部署前需要重点解决的问题。")

    doc.add_page_break()
    add_heading(doc, "证据来源说明", 1)
    add_paragraph(doc, "本稿依据当前工作区代码和结果文件生成，未虚构未完成的实验指标。后续若重新运行轨迹规划或控制仿真，应同步更新下表对应文件中的数值。")
    add_three_line_table(
        doc,
        "表 A-1  本稿主要证据来源",
        ["内容", "来源文件"],
        [
            ["轨迹规划入口", r"D:\NPU\Matlab_code\stewart-simscape\opt_minimal\run_01_ihsid_trajectory.m"],
            ["标准轨迹结果摘要", r"D:\NPU\Matlab_code\stewart-simscape\opt_minimal\examples\ihsid_40x20_limited_memory\summary.txt"],
            ["Simscape 参考导出", r"D:\NPU\Matlab_code\stewart-simscape\controller\simscape_tracking\exportTrajectoryToSimscape.m"],
            ["位姿力控制准备", r"D:\NPU\Matlab_code\stewart-simscape\controller\simscape_tracking\prepareSimscapePoseForceControl.m"],
            ["computed-torque 前馈", r"D:\NPU\Matlab_code\stewart-simscape\controller\simscape_tracking\computeComputedTorqueForce.m"],
            ["LQI 设计", r"D:\NPU\Matlab_code\stewart-simscape\controller\simscape_tracking\designLqiPoseForceController.m"],
            ["非理想执行器", r"D:\NPU\Matlab_code\stewart-simscape\controller\simscape_tracking\makeNonidealForceActuator.m"],
            ["跟踪结果摘要", r"D:\NPU\Matlab_code\stewart-simscape\results\controller\summary_simscape_pose_force_control_20260617_214358.txt"],
            ["四方案对比", r"D:\NPU\Matlab_code\stewart-simscape\results\reports\four_control_group_meeting_comparison_20260615_225458"],
        ],
        [1.65, 4.8],
    )

    doc.core_properties.title = "第三章轨迹规划与第四章跟踪控制实现"
    doc.core_properties.author = "谢振宇"
    doc.save(DOCX_PATH)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tex-only", action="store_true", help="只生成 TikZ 图源")
    args = parser.parse_args()
    ensure_dirs()
    write_tikz_sources()
    if args.tex_only:
        return
    build_document()
    print(DOCX_PATH)


if __name__ == "__main__":
    main()
