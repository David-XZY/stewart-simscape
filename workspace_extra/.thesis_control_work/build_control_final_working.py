from __future__ import annotations

import json
import re
import subprocess
import sys
import zipfile
from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches

sys.path.insert(0, r"D:\NPU\Matlab_code")
import build_control_thesis as base


WORKSPACE = Path(r"D:\NPU\Matlab_code")
WORK_DIR = WORKSPACE / ".thesis_control_work"
WORKING_DOCX = WORK_DIR / "control_final_working.docx"
EQUATION_JSON = WORK_DIR / "control_final_working_equations.json"
FINALIZER = WORK_DIR / "finalize_control_initial_word.ps1"
QA_DIR = WORK_DIR / "control_final_qa"
PDF_PATH = QA_DIR / "control_final_working.pdf"
NEW_EQUATION_COUNT = 44

EXACT_DIR = WORKSPACE / "stewart-simscape" / "results" / "reports" / "strict_clf_cbf_qp" / "exact_model_study"
FIG_ABLATION = EXACT_DIR / "exact_model_ablation_safety.png"
FIG_BARRIERS = EXACT_DIR / "exact_model_nominal_strict_barriers.png"
FIG_IDEAL = (
    WORKSPACE / "stewart-simscape" / "results" / "reports"
    / "strict_clf_cbf_qp_simscape_buffered2_20260712_050114"
    / "strict_qp_ideal_force.png"
)


def replace_paragraph(paragraph, text: str, *, first_line: bool | None = None,
                      east_asia: str = "宋体", latin: str = "Times New Roman",
                      size: float = 12, bold: bool | None = None) -> None:
    for run in list(paragraph.runs):
        paragraph._p.remove(run._r)
    run = paragraph.add_run(text)
    base.set_font(run, east_asia, latin, size, bold)
    base.set_east_asia_language(run)
    if first_line is not None:
        from docx.shared import Pt
        paragraph.paragraph_format.first_line_indent = Pt(24) if first_line else Pt(0)


def find_unique(doc: Document, *, exact: str | None = None, prefix: str | None = None):
    matches = []
    for paragraph in doc.paragraphs:
        value = paragraph.text.strip()
        if exact is not None and value == exact:
            matches.append(paragraph)
        elif prefix is not None and value.startswith(prefix):
            matches.append(paragraph)
    if len(matches) != 1:
        raise RuntimeError(f"Expected one paragraph, found {len(matches)}: exact={exact!r}, prefix={prefix!r}")
    return matches[0]


def prepare_existing_chapter_five(doc: Document) -> None:
    """Remove numbering gaps left by the compact initial-draft selection.

    The locked initial snapshot intentionally retained only the two baseline
    result figures and two result tables from Chapter 5.  In the final copy,
    renumber those retained objects consecutively and move the compact metrics
    table ahead of the first tall figure so the preceding page is not left
    mostly blank.  The snapshot itself is never modified.
    """
    tracking_caption = find_unique(
        doc, exact="图5-2  最新计算力矩-LQI基线位姿与腿长跟踪误差")
    leg_caption = find_unique(
        doc, exact="图5-3  最新基线六支链速度、加速度与中期边界")
    metrics_caption = find_unique(doc, exact="表5-3  最新基线总体跟踪指标")

    image_paragraph = tracking_caption._p.getprevious()
    table_element = metrics_caption._p.getnext()
    if image_paragraph is None or not image_paragraph.xpath(".//wp:inline"):
        raise RuntimeError("Chapter 5 tracking image paragraph not found")
    if table_element is None or table_element.tag != qn("w:tbl"):
        raise RuntimeError("Chapter 5 metrics table not found")
    spacer_element = table_element.getnext()
    if spacer_element is None or spacer_element.tag != qn("w:p"):
        raise RuntimeError("Chapter 5 metrics-table spacer not found")

    image_paragraph.addprevious(metrics_caption._p)
    image_paragraph.addprevious(table_element)
    image_paragraph.addprevious(spacer_element)

    replace_paragraph(
        find_unique(doc, prefix="图5-2给出了六维位姿"),
        "表5-1汇总基线总体跟踪指标，图5-1给出了六维位姿和六腿长度的真实跟踪误差。"
        "平移误差全程小于0.8 mm，最大值出现在y方向；姿态误差最大分量为β，其峰值约0.146°；"
        "腿长误差峰值约1.9 mm。误差曲线在轨迹阶段转换处保持连续，没有出现旧结果中的明显初始下坠或大幅腿长瞬态。",
    )
    replace_paragraph(tracking_caption,
                      "图5-1  最新计算力矩-LQI基线位姿与腿长跟踪误差",
                      first_line=False, size=10.5)
    replace_paragraph(metrics_caption, "表5-1  最新基线总体跟踪指标",
                      first_line=False, size=10.5)
    replace_paragraph(
        find_unique(doc, prefix="图5-3给出了六条支链速度"),
        "图5-2给出了六条支链速度与加速度。最大腿速为0.279 m/s，低于0.60 m/s；"
        "最大腿加速度为0.165 m/s²，低于中期1.20 m/s²边界。由此可见，当前标称基线并未发生腿加速度超限，"
        "QP改进的动机应表述为在扰动和边界工况下建立显式保证，而不是修正一个本次并不存在的违反。",
    )
    replace_paragraph(
        leg_caption,
        "图5-2  最新基线六支链速度、加速度与中期边界",
        first_line=False, size=10.5)
    replace_paragraph(
        find_unique(doc, exact="表5-5  最新基线物理约束复核"),
        "表5-2  最新基线物理约束复核",
        first_line=False, size=10.5)

    replace_inline_figure(
        doc, tracking_caption, base.FRESH_TRACKING, 5.55,
        "图5-1  最新计算力矩-LQI基线位姿与腿长跟踪误差")
    replace_inline_figure(
        doc, leg_caption, base.FRESH_LEG, 5.75,
        "图5-2  最新基线六支链速度、加速度与中期边界")


def replace_inline_figure(doc: Document, caption_paragraph, image_path: Path,
                          width_in: float, alt_text: str) -> None:
    """Replace the inline picture immediately preceding a caption."""
    if not image_path.exists():
        raise FileNotFoundError(image_path)
    image_element = caption_paragraph._p.getprevious()
    if image_element is None or not image_element.xpath(".//wp:inline"):
        raise RuntimeError(f"Inline image not found before caption: {caption_paragraph.text}")
    image_paragraph = next(
        (paragraph for paragraph in doc.paragraphs if paragraph._p is image_element),
        None)
    if image_paragraph is None:
        raise RuntimeError("Unable to map inline-image XML to a paragraph")
    for child in list(image_element):
        if child.tag != qn("w:pPr"):
            image_element.remove(child)
    shape = image_paragraph.add_run().add_picture(str(image_path), width=Inches(width_in))
    base.set_image_alt(shape, alt_text)


def add_continued_three_line_table(doc: Document, caption: str,
                                   headers: list[str], rows: list[list[str]],
                                   widths_dxa: list[int], *, split_at: int,
                                   left_columns: set[int] | None = None,
                                   font_size: float = 10.5) -> None:
    """Create two explicitly captioned fragments of one logical long table."""
    if not 0 < split_at < len(rows):
        raise ValueError("split_at must divide the table into two nonempty parts")
    base.add_three_line_table(
        doc, caption, headers, rows[:split_at], widths_dxa,
        left_columns=left_columns, font_size=font_size)
    first_spacer = doc.paragraphs[-1]
    if first_spacer.text.strip():
        raise RuntimeError("Expected an empty spacer after the first table fragment")
    first_spacer._p.getparent().remove(first_spacer._p)
    base.add_three_line_table(
        doc, "续" + caption, headers, rows[split_at:], widths_dxa,
        left_columns=left_columns, font_size=font_size)
    final_spacer = doc.paragraphs[-1]
    if final_spacer.text.strip():
        raise RuntimeError("Expected an empty spacer after the continuation table")
    final_spacer._p.getparent().remove(final_spacer._p)


def update_front_matter(doc: Document) -> None:
    cover = find_unique(doc, prefix="面向载荷挂装的并联机器人")
    replace_paragraph(
        cover,
        "面向载荷挂装的并联机器人\n计算力矩-LQI与CLF-CBF-QP控制研究",
        first_line=False,
        east_asia="黑体",
        size=22,
        bold=True,
    )

    chinese = [
        "Stewart并联机器人具有承载能力强、结构刚度高和运动精度高等特点，适用于重载定位、装配对接与载荷挂装。此类任务既要求动平台准确跟踪六维位姿轨迹，又要求支链长度、速度、加速度、驱动力、碰撞间隙和运动学条件始终满足边界。针对固定控制律难以显式协调多类约束的问题，本文研究以计算力矩-LQI为基线、以严格CLF-CBF-QP为安全滤波层的分层控制方法。",
        "首先，在统一坐标系和符号约定下建立Stewart平台逆运动学、速度映射和输入仿射动力学模型，严格区分平台扭量Jacobian与欧拉角广义坐标Jacobian。基线控制器以逆动力学计算力矩承担重力和参考运动载荷，以含状态观测、积分泄漏和抗饱和回灌的LQI补偿跟踪误差，并统一定义名义支链力为计算力矩项与LQI项之和。2026年7月12日重新运行的非理想力Simscape基线通过综合验收，最大平移和姿态误差分别为0.788 mm和0.146°，最大腿加速度为0.165 m/s²。",
        "在基线总力之后，本文进一步构造七变量严格CLF-CBF-QP。时变CLF仅允许一个非负松弛量；腿长采用二阶高阶控制屏障函数，腿速采用一阶控制屏障函数，腿加速度、支链力和力变化率作为硬仿射边界。圆柱体与有向包围盒之间采用固定法向支持函数建立保守分离证书，归一化Jacobian的最小奇异值采用迹逆量构造严格下界；相关梯度、Hessian和Lie导数由CasADi算法微分生成。连续时间、模型匹配、理想力输入、初始递归条件成立且QP持续可行时，可据此得到安全集合前向不变性。",
        "精确非线性模型消融表明，计算力矩-LQI、仅含CLF和执行器边界的CLF-QP最小物理碰撞裕度分别为-0.783 mm和-0.786 mm，完整CLF-CBF-QP则为+0.422 mm，751个节点均可行且无回退，QP求解时间95%分位为4.042 ms。匹配参数理想力Simscape工况的最小物理碰撞裕度为+0.504 mm、最小实际奇异值为0.284，QP求解时间95%分位为2.944 ms。名义非理想执行器和时间常数增大至2倍的工况通过逐点数值验收；质心偏移2 mm以及载荷质量增加2%、5%和10%的试验未通过，表明当前方法不具有模型参数失配下的鲁棒安全保证。10 ms零阶保持结果属于给定工况的采样验证，不能替代连续时间定理。",
    ]
    chinese_prefixes = [
        "Stewart并联机器人具有承载能力强",
        "首先，在统一坐标系和符号约定下建立Stewart平台",
        "在基线基础上，本文进一步给出约束QP",
        "采用2026年7月12日重新运行并保存的Simscape",
    ]
    for prefix, text in zip(chinese_prefixes, chinese):
        replace_paragraph(find_unique(doc, prefix=prefix), text, first_line=True)
    replace_paragraph(
        find_unique(doc, prefix="关键词："),
        "关键词：Stewart并联机器人；计算力矩；LQI；控制Lyapunov函数；控制屏障函数；二次规划；Simscape",
        first_line=False,
    )

    english = [
        "This thesis investigates a hierarchical computed-torque-LQI and CLF-CBF-QP control method for a Stewart parallel robot performing payload installation. The task requires accurate six-degree-of-freedom tracking while respecting leg motion, actuator, collision-clearance, and singularity limits. A unified kinematic and input-affine dynamic model is established, with the twist Jacobian distinguished from the generalized-coordinate Jacobian associated with ZYX Euler angles. The baseline combines inverse-dynamics computed-torque feedforward with LQI output feedback, including state observation, leaky integration, and anti-windup back-calculation. The complete nominal leg-force command is the sum of the computed-torque and LQI terms. A fresh nonideal-force Simscape run passed all baseline checks: the peak translational and rotational errors were 0.788 mm and 0.146 degrees, respectively, and the peak leg acceleration was 0.165 m/s².",
        "A seven-variable strict CLF-CBF-QP safety filter is then placed after the nominal controller. Tracking is represented by a time-varying control Lyapunov function with one nonnegative slack variable, whereas every control-barrier, leg-acceleration, force, and force-rate condition is hard. Second-order high-order barrier functions enforce leg length, collision clearance, and a strict singularity lower bound; first-order barriers enforce leg speed. A fixed-normal support-function certificate conservatively separates the carried cylinder from each oriented bounding box, and CasADi provides analytic derivatives.",
        "In the exact nonlinear ablation, the minimum physical clearance margins of computed-torque-LQI, CLF-QP, and full CLF-CBF-QP were -0.783 mm, -0.786 mm, and +0.422 mm, respectively. The full method was feasible without fallback at all 751 nodes, with a 4.042 ms 95th-percentile QP time. In the matched ideal-force Simscape case, the minimum clearance margin was +0.504 mm, the minimum actual singular value was 0.284, and the 95th-percentile QP time was 2.944 ms. The nominal nonideal actuator and doubled actuator time constant passed the sampled checks, whereas a 2 mm center-of-mass offset and payload-mass increases of 2%, 5%, and 10% did not. Hence, the continuous-time invariance statement is limited to matched ideal-force dynamics, and the 10 ms zero-order-hold simulations provide scenario-specific evidence rather than an unconditional robustness guarantee.",
    ]
    english_prefixes = [
        "The Stewart parallel robot is attractive",
        "A constrained quadratic-programming layer is subsequently formulated",
        "A fresh Simscape simulation with a nonideal force actuator",
    ]
    for prefix, text in zip(english_prefixes, english):
        replace_paragraph(find_unique(doc, prefix=prefix), text, first_line=False)
    replace_paragraph(
        find_unique(doc, prefix="KEY WORDS:"),
        "KEY WORDS: Stewart parallel robot; computed torque; LQI; control Lyapunov function; control barrier function; quadratic programming; Simscape",
        first_line=False,
    )

    header_text = "并联机器人计算力矩-LQI与CLF-CBF-QP控制研究"
    for section in doc.sections:
        header = section.header
        for paragraph in header.paragraphs:
            if paragraph.text.strip():
                replace_paragraph(paragraph, header_text, first_line=False, size=9)


def update_initial_stage_language(doc: Document) -> None:
    replacements = {
        "最新结果表明，计算力矩能够准确承担": (
            "最新结果表明，计算力矩能够承担重力和轨迹运动所需载荷，LQI以较小反馈修正获得良好标称跟踪，且中期物理边界均满足。这一结果为严格QP提供了质量较高的名义控制器：约束不活动时应尽量复现基线性能，约束趋于活动时才进行必要修正。第6章的消融结果据此区分CLF性能约束与硬CBF安全约束的作用。"
        ),
        "另一方面，单次标称仿真不能证明": (
            "另一方面，单次标称仿真不能证明负载质量变化、质心偏移、执行器滞后增大或接近几何边界时仍满足约束。第6章采用与基线一致的参考轨迹和边界，分别在精确非线性模型、匹配参数理想力Simscape和非理想执行器模型上检验约束滤波器，并把消融与参数扰动结果分开报告。"
        ),
        "严格方法的验收标准应包括：": (
            "第6章的严格方法验收同时检查QP持续可行与回退状态、支链力和支链运动边界、奇异性下界、真实几何距离、CLF松弛量、活动约束和求解时间。任何失败节点均保留在结果中，不对误差或鲁棒性指标作人工缩放。"
        ),
    }
    for prefix, text in replacements.items():
        replace_paragraph(find_unique(doc, prefix=prefix), text, first_line=True)
    replace_paragraph(
        find_unique(doc, exact="5.9 结果讨论与后续严格验证要求"),
        "5.9 结果讨论与严格验证口径",
        first_line=False,
        east_asia="黑体",
        size=14,
        bold=False,
    )


def remove_old_conclusion_and_references(doc: Document) -> None:
    start = find_unique(doc, exact="结论与本阶段工作界限")._p
    body = doc._element.body
    removing = False
    for child in list(body):
        if child is start:
            removing = True
        if removing and child.tag != qn("w:sectPr"):
            body.remove(child)


def set_two_level_toc(doc: Document) -> None:
    found = 0
    for instr in doc.element.body.xpath(".//w:instrText"):
        if instr.text and "TOC" in instr.text:
            instr.text = ' TOC \\o "1-2" \\h \\z \\u '
            found += 1
    if found != 1:
        raise RuntimeError(f"Expected one TOC field, found {found}")


def enforce_table_pagination(doc: Document) -> None:
    for table in doc.tables:
        for row_index, row in enumerate(table.rows):
            tr_pr = row._tr.get_or_add_trPr()
            if tr_pr.find(qn("w:cantSplit")) is None:
                marker = OxmlElement("w:cantSplit")
                marker.set(qn("w:val"), "true")
                tr_pr.append(marker)
            if row_index == 0:
                for cell in row.cells:
                    for paragraph in cell.paragraphs:
                        paragraph.paragraph_format.keep_with_next = True
    for paragraph in doc.paragraphs:
        if paragraph.style.name == "Caption Thesis":
            next_node = paragraph._p.getnext()
            if next_node is not None and next_node.tag == qn("w:tbl"):
                paragraph.paragraph_format.keep_with_next = True


def eq(doc: Document, number: int, linear: str) -> None:
    base.add_equation(doc, f"(6-{number})", linear)


def eq_label(doc: Document, label: str, linear: str) -> None:
    base.add_equation(doc, f"(6-{label})", linear)


def build_chapter_six(doc: Document) -> None:
    base.add_heading(doc, "第6章 严格CLF-CBF-QP补充研究", 1)
    base.add_paragraph(doc, "前述计算力矩-LQI控制器在名义轨迹上取得了较小误差，但其支链力由固定反馈规律直接给出，接近腿长、腿速、碰撞间隙或奇异性边界时不能主动协调相互竞争的要求。第5章初步QP建立了在名义控制量附近修正输入的基本框架，其中碰撞和奇异性仍主要是监测指标。本章在完整名义支链力之后增加严格CLF-CBF-QP安全滤波层：控制Lyapunov函数只描述跟踪性能并允许一个非负松弛量，全部控制屏障函数、支链力和力变化率约束均为无松弛硬约束。")
    base.add_paragraph(doc, "本章的“严格”首先指动力学、碰撞证书和奇异性指标的导数来自同一解析模型及算法微分，不以有限差分代替在线Lie导数；其次指求解失败必须显式记为不可行或非认证回退，不能被计作安全成功；最后指理论结论仅适用于连续时间、模型匹配和理想力输入。当前10 ms零阶保持实现仍需依靠逐点仿真与额外几何缓冲验证，不能与连续时间前向不变性命题等同。")

    base.add_heading(doc, "6.1 控制对象与输入仿射模型", 2)
    base.add_heading(doc, "6.1.1 广义坐标、速度映射与动力学", 3)
    base.add_paragraph(doc, "延续第2章符号，动平台广义坐标由质心参考点位置和ZYX欧拉角组成：")
    eq(doc, 1, "q=[p^T,θ^T]^T,  p=[x,y,z]^T,  θ=[ϕ,ϑ,ψ]^T")
    base.add_paragraph(doc, "平台角速度与ZYX欧拉角速度通过姿态相关矩阵E(θ)转换，相应支链速度映射为")
    eq(doc, 2, "L̇=J_q(q)q̇,  J_q=J_v blkdiag(I_3,E(θ))")
    base.add_paragraph(doc, "考虑质量、转动惯量、质心偏置、重力和速度耦合项后，平台动力学统一写为")
    eq(doc, 3, "H(q)q̈+h(q,q̇)=J_v(q)^T F")
    base.add_paragraph(doc, "其中H为广义惯性矩阵，h汇集重力、离心、科氏和欧拉角参数化偏置，F为六维支链轴向力。该写法与前文和中期实现一致，Stewart平台建模基础参见文献[1-2]。定义漂移项和输入矩阵")
    eq_label(doc, "4a", "x=[q^T,q̇^T]^T,  f_2=-H^(-1)h,  g_2=H^(-1)J_v^T")
    base.add_paragraph(doc, "则精确输入仿射状态方程为")
    eq_label(doc, "4b", "ẋ=f(x)+g(x)F=[q̇;f_2(q,q̇)]+[0_(6×6);g_2(q)]F")
    base.add_paragraph(doc, "后续推导在H非奇异、欧拉角速度映射可逆且模型连续可微的工作域内进行。")
    base.add_heading(doc, "6.1.2 名义控制量与安全滤波层", 3)
    base.add_paragraph(doc, "严格QP不改变基线内部结构，计算力矩负责前馈补偿，LQI负责状态估计、积分与反馈校正，其总输出定义为")
    eq(doc, 5, "F_(nom)=F_(CT)+F_(LQI)")
    base.add_paragraph(doc, "QP位于上述总力之后。当约束远离边界时，优化器保持F接近Fnom；当某项约束趋于活动时，只沿必要方向修正支链力，从而把多约束协调转化为每个采样周期内的凸二次规划。")

    base.add_heading(doc, "6.2 时变跟踪CLF与高阶屏障函数", 2)
    base.add_heading(doc, "6.2.1 跟踪CLF及唯一松弛量", 3)
    base.add_paragraph(doc, "定义位姿误差、速度误差和增广误差，并以正定矩阵P构造时变CLF：")
    eq(doc, 6, "e=q-q_r,  ė=q̇-q̇_r,  e_x=[e;ė],  V(x,t)=e_x^T P e_x")
    base.add_paragraph(doc, "矩阵P由期望二阶误差模型的Lyapunov方程确定，所用比例与微分增益为")
    eq_label(doc, "7a", "K_p=diag(30,30,30,18,18,18),  K_d=diag(11,11,11,8,8,8)")
    eq_label(doc, "7b", "A_e=[0,I;-K_p,-K_d],  A_e^T P+P A_e=-I_(12),  P=P^T>0")
    base.add_paragraph(doc, "参考轨迹显含时间，因此V的全导数必须保留时间项。沿式（6-4b）有")
    eq(doc, 8, "V̇=L_f V+L_g V F+∂_t V=2e_x^T P([q̇-q̇_r;f_2-q̈_r]+[0;g_2]F)")
    base.add_paragraph(doc, "在QP中仅为CLF设置非负松弛量，衰减率取值见表6-1：")
    eq(doc, 9, "L_f V+L_g V F+∂_t V≤-c_V V+δ_V,  δ_V≥0")
    base.add_paragraph(doc, "安全边界与跟踪目标冲突时，允许跟踪性能退让而不能破坏硬安全约束。由比较原理[12]，CLF满足")
    eq(doc, 10, "V(t)≤exp(-c_V t)V(0)+∫_0^t exp(-c_V(t-τ)) δ_V(τ) dτ")
    base.add_paragraph(doc, "因此δV恒为零时获得指数衰减；δV有界时只能得到相应的实用跟踪界，不能表述为任意安全约束活动下的无条件渐近跟踪。")
    base.add_heading(doc, "6.2.2 HOCBF递归", 3)
    base.add_paragraph(doc, "相对输入阶次为2的安全函数采用高阶控制屏障函数递归[11,14,18]。本实现使用线性扩展类K函数：")
    eq(doc, 11, "ψ_0=h,  ψ_1=ḣ+α_1 h,  ψ_2=ψ̇_1+α_2 ψ_1=ḧ+(α_1+α_2)ḣ+α_1 α_2 h≥0")
    base.add_paragraph(doc, "相对阶为2的约束必须满足式（6-11）的递归初值条件；相对阶为1的速度边界采用一阶屏障不等式，具体见式（6-15）。")

    base.add_heading(doc, "6.3 支链运动与执行器硬约束", 2)
    base.add_heading(doc, "6.3.1 腿长与腿速屏障", 3)
    base.add_paragraph(doc, "对支链速度式求导，支链加速度可精确写成控制输入的仿射形式")
    eq(doc, 12, "L̈=d_L(x)+A_L(q)F,  d_L=b_L+J_q f_2,  A_L=J_q g_2")
    base.add_paragraph(doc, "其中bL包含支链方向变化、平台转动和端点向心加速度等几何项，不由相邻采样点差分得到。第i条腿的长度上下界安全函数为")
    eq(doc, 13, "h_(L,i)^-=L_i-L_(i,min),  h_(L,i)^+=L_(i,max)-L_i")
    base.add_paragraph(doc, "令dL,i为dL的第i项、aL,iᵀ为AL第i行，采用表6-1的长度屏障增益，两侧HOCBF为")
    eq(doc, 14, "±[d_(L,i)+a_(L,i)^T F+(α_(L1)+α_(L2))L̇_i]+α_(L1)α_(L2)h_(L,i)^(∓)≥0")
    base.add_paragraph(doc, "式（6-14）取上号时对应长度下界hL,i⁻，取下号时对应长度上界hL,i⁺。")
    base.add_paragraph(doc, "腿速边界对支链力的相对阶为1。采用表6-1的速度屏障增益，定义上下界安全函数并得到")
    eq_label(doc, "15a", "h_(v,i)^+=v_(L,i)^max-L̇_i,  h_(v,i)^-=v_(L,i)^max+L̇_i")
    eq_label(doc, "15b", "±[d_(L,i)+a_(L,i)^T F]+α_v h_(v,i)^(∓)≥0")
    base.add_paragraph(doc, "式（6-15b）上号对应速度下界hᵥ,ᵢ⁻，下号对应速度上界hᵥ,ᵢ⁺。")
    base.add_heading(doc, "6.3.2 腿加速度、支链力与力变化率", 3)
    base.add_paragraph(doc, "腿加速度不再经过预测近似，而是直接对式（6-12）的仿射量施加硬边界：")
    eq(doc, 16, "-a_(L,i)^max≤d_(L,i)+a_(L,i)^T F≤a_(L,i)^max,  i=1,…,6")
    base.add_paragraph(doc, "支链力和离散力变化率约束分别为")
    eq(doc, 17, "F_min≤F≤F_max,  -Ḟ_max T_s≤F_k-F_(k-1)≤Ḟ_max T_s")
    base.add_paragraph(doc, "式（6-17）的第二组边界属于数字执行器约束而不是连续时间CBF。程序将力幅值和力率区间求交，形成当前QP变量的动态上下界。")

    base.add_heading(doc, "6.4 圆柱体与OBB的保守分离证书", 2)
    base.add_heading(doc, "6.4.1 固定法向支持函数证书", 3)
    base.add_paragraph(doc, "将转运物体表示为有限圆柱体C(q)，将顶板和左右裙板表示为有向包围盒Bj：")
    eq_label(doc, "18a", "C(q)={c_C+s a_C+r: |s|≤l_C/2, r^T a_C=0, ‖r‖_2≤r_C}")
    eq_label(doc, "18b", "B_j={c_(B,j)+R_(B,j)ξ: |ξ_m|≤b_(j,m), m=1,2,3}")
    base.add_paragraph(doc, "对任意固定单位法向n，OBB的最小投影与圆柱体的最大支持函数为")
    eq_label(doc, "19a", "β_j(n)=n^T c_(B,j)-Σ_(m=1)^3 b_(j,m)|n^T R_(B,j)e_m|")
    eq_label(doc, "19b", "s_C(n,q)=n^T c_C+l_C|n^T a_C|/2+r_C√(1-(n^T a_C)^2)")
    base.add_paragraph(doc, "为获得处处可微的梯度和Hessian，采用表6-1所列平滑参数构造向外平滑支持函数。由于平滑值不小于真实支持函数，相应候选间隙是保守下界。")
    eq_label(doc, "20a", "z=n^T a_C,  s̃_C=n^T c_C+l_C√(z^2+ε^2)/2+r_C√(1-z^2+ε^2)")
    eq_label(doc, "20b", "g_(j,n)=β_j(n)-s̃_C(n,q)")
    base.add_paragraph(doc, "顶板使用其下表面内法向，左右裙板使用固定有限法向组。多个候选间隙通过递归平滑最大值组合：")
    eq_label(doc, "21a", "m_ε(a,b)=(a+b+√((a-b)^2+ε^2)-ε)/2≤max(a,b)")
    eq_label(doc, "21b", "d_j^(cert)≤max_(n∈N_j) g_(j,n)≤d_j")
    base.add_paragraph(doc, "因此平滑证书不小于所需间隙足以推出真实欧氏距离满足要求；证书为负只表示当前法向组不能认证安全，不必然意味着实体已相交。")
    base.add_heading(doc, "6.4.2 时变间隙与碰撞HOCBF", 3)
    base.add_paragraph(doc, "顶板所需间隙在5 s接近阶段后，经2.5 s五次平滑函数由16 mm过渡至5 mm；左右裙板保持15 mm。")
    eq_label(doc, "22a", "s(τ)=10τ^3-15τ^4+6τ^5")
    eq_label(doc, "22b", "τ(t)=min(1,max(0,(t-T_a)/T_i)),  d_R^(req)(t)=d_1+(d_f-d_1)s(τ(t))")
    base.add_paragraph(doc, "对第j个障碍物构造证书裕度，算法微分给出梯度和Hessian，从而有")
    eq_label(doc, "23a", "h_(C,j)=d_j^(cert)-d_j^(req),  ḣ_(C,j)=∇d_j^(cert) q̇-ḋ_j^(req)")
    eq_label(doc, "23b", "ḧ_(C,j)=∇d_j^(cert)(f_2+g_2 F)+q̇^T∇^2 d_j^(cert) q̇-d̈_j^(req)")
    base.add_paragraph(doc, "采用表6-1的碰撞屏障增益，顶板和两侧裙板均施加二阶HOCBF：")
    eq(doc, 24, "ḧ_(C,j)+(α_(C1)+α_(C2))ḣ_(C,j)+α_(C1)α_(C2)h_(C,j)≥0,  j∈{roof,left,right}")
    base.add_paragraph(doc, "终端参考位姿的真实顶板间隙为5.000000 mm，平滑证书为4.999400 mm，保守差为0.600 μm。Simscape采样实现另在三项碰撞阈值上增加0.5 mm缓冲，结果评价仍按16→5 mm和15 mm物理阈值进行；缓冲不能被解释为放宽物理要求。")

    base.add_heading(doc, "6.5 奇异性严格下界及HOCBF", 2)
    base.add_paragraph(doc, "平移和转动分量量纲不同，先以特征长度ℓ*归一化速度Jacobian，并定义迹逆形式下界")
    eq(doc, 25, "J̄=J_v diag(I_3,l_*^(-1)I_3),  G=J̄J̄^T,  σ̲^2=[tr(G^(-1))]^(-1)")
    base.add_paragraph(doc, "G的特征值等于归一化Jacobian各奇异值的平方，因此可得")
    eq_label(doc, "26a", "tr(G^(-1))=Σ_(i=1)^6 1/λ_i")
    eq_label(doc, "26b", "tr(G^(-1))≥1/λ_min,  σ̲^2≤λ_min=σ_min^2")
    base.add_paragraph(doc, "据此构造奇异性安全函数，阈值和屏障增益见表6-1；安全函数非负即可推出实际最小奇异值不小于阈值。")
    eq_label(doc, "27a", "h_σ=σ̲^2-σ_(safe)^2,  ḣ_σ=∇h_σ q̇")
    eq_label(doc, "27b", "ḧ_σ=∇h_σ(f_2+g_2 F)+(q̇)^T(∇^2 h_σ)q̇")
    eq_label(doc, "27c", "ḧ_σ+(α_(σ1)+α_(σ2))ḣ_σ+α_(σ1)α_(σ2)h_σ≥0")

    base.add_heading(doc, "6.6 严格QP与前向不变性", 2)
    base.add_heading(doc, "6.6.1 优化问题", 3)
    base.add_paragraph(doc, "在第k个控制时刻，以六维支链力和一个CLF松弛量组成七维决策变量，求解")
    eq(doc, 28, "min_(F,δ_V)  0.5‖F-F_(nom)‖_(W_F)^2+0.5‖F-F_(k-1)‖_(W_Δ)^2+0.5ρ_V δ_V^2")
    base.add_paragraph(doc, "约束集合为")
    eq_label(doc, "29a", "L_f V+L_g V F+∂_t V≤-c_V V+δ_V,  A_(CBF)(x,t)F≤b_(CBF)(x,t)")
    eq_label(doc, "29b", "F_min≤F≤F_max,  -Ḟ_max T_s≤F-F_(k-1)≤Ḟ_max T_s,  δ_V≥0")
    base.add_paragraph(doc, "其中ACBF包含12条腿长HOCBF、12条腿速CBF、12条腿加速度、3条碰撞HOCBF和1条奇异性HOCBF，共40行；再计入12条力幅值和12条力变化率边界，共64条硬约束。权重取值见表6-1。目标Hessian正定，因此可行时最优解唯一[10,16]。求解失败或后验硬约束残差超限时，程序只输出限幅、限速回退，并置位“回退未获安全认证”。")
    base.add_heading(doc, "6.6.2 前向不变性命题与采样限定", 3)
    base.add_paragraph(doc, "命题6-1　在连续时间理想力输入模型（6-4b）下，设模型与屏障函数在工作域内连续可微；相对阶2约束满足式（6-11）的递归初值条件，相对阶1约束的初始屏障值非负；QP（6-28）—（6-29）持续可行，且最优控制关于状态局部Lipschitz。则下列安全集合前向不变：")
    eq(doc, 30, "S=∩_(j∈I_1){x:h_j≥0}∩∩_(j∈I_2){x:h_j≥0,ψ_(1,j)≥0}")
    base.add_paragraph(doc, "证明要点为：全部CBF行均为硬约束，相对阶1与相对阶2约束分别满足相应的一阶屏障不等式和第二递归不等式；结合初始递归条件和比较原理，可依次得到递归量及原屏障函数在后续时刻保持非负[11,18]。有限个安全集合取交即得式（6-30）。式（6-21）和式（6-26）分别给出碰撞与奇异性证书到真实物理指标的单向蕴含；CLF松弛量不进入CBF行，因而不改变上述安全推理。")
    base.add_paragraph(doc, "若CLF松弛量存在非负上界，则跟踪性能进一步满足")
    eq(doc, 31, "V(t)≤exp(-c_V t)V(0)+δ̄(1-exp(-c_V t))/c_V")
    base.add_paragraph(doc, "命题6-1是连续时间结论。当前代码每10 ms求解一次QP并保持支链力，尚未建立采样数据HOCBF不变性定理；因此仿真只能陈述给定模型、轨迹与采样周期下的逐点物理边界结果。0.5 mm碰撞缓冲是工程补偿，不替代采样数据理论。非理想执行器和参数失配使真实输入或漂移项偏离QP模型，其结果仅为工程鲁棒性证据。")

    base.add_heading(doc, "6.7 实现与数学验收", 2)
    base.add_heading(doc, "6.7.1 控制接口与参数", 3)
    base.add_paragraph(doc, "严格控制器采用独立模式controller.type=11，单步接口接收平台位姿、广义速度、参考位姿及其一、二阶导数、名义支链力、上一时刻支链力、模型、场景和配置，并输出支链力、CLF松弛量、屏障值、活动约束、求解时间、可行与回退标志。Simscape中QP位于计算力矩-LQI总力之后；线速度和角速度由传感器直接测量，不以位姿数值差分代替速度。")
    base.add_paragraph(doc, "CasADi[19]生成并缓存输入仿射动力学、支链加速度、碰撞证书与奇异性下界的梯度和Hessian。在线控制器不调用有限差分，测试中的中心差分只作独立方向导数核验。QP使用凸内点算法，约束与最优性容差均为10⁻⁸，最大迭代100次，解返回后再次计算硬约束最大违反量。")
    add_continued_three_line_table(
        doc,
        "表6-1  严格CLF-CBF-QP主要参数",
        ["参数", "数值", "说明"],
        [
            ["控制周期 Ts", "0.01 s", "100 Hz零阶保持"],
            ["腿长/腿速/腿加速度", "1.05～2.00 m；0.60 m/s；1.20 m/s²", "六条腿双侧硬边界"],
            ["支链力/力变化率", "±2000 N；8000 N/s", "每步最大变化80 N"],
            ["奇异值阈值", "0.10", "归一化Jacobian"],
            ["长度/速度增益", "4, 4 s⁻¹；5 s⁻¹", "HOCBF；一阶CBF"],
            ["碰撞/奇异性增益", "12, 12 s⁻¹；5, 5 s⁻¹", "二阶HOCBF"],
            ["CLF衰减率/松弛惩罚", "2 s⁻¹；10⁶", "唯一性能松弛"],
            ["WF / WΔ", "I₆；2×10⁻³I₆", "名义力与相邻力权重"],
            ["平滑参数 ε", "10⁻⁶", "支持函数与平滑最大值"],
            ["物理碰撞阈值", "顶板16→5 mm；裙板15 mm", "五次平滑过渡"],
            ["Simscape控制阈值", "顶板16.5→5.5 mm；裙板15.5 mm", "含0.5 mm采样缓冲"],
        ],
        [2500, 3100, 3232],
        split_at=7,
        left_columns={0, 2},
        font_size=9.5,
    )
    base.add_heading(doc, "6.7.2 数学与程序一致性", 3)
    base.add_paragraph(doc, "2026年7月12日重新执行测试70～75，六项均通过。检查范围覆盖控制器接口、精确动力学、算法微分、独立几何距离、奇异性下界、QP硬约束结构和闭环证据工件。")
    base.add_three_line_table(
        doc,
        "表6-2  严格控制模块数学验收结果",
        ["测试", "核心检查", "数值判据与结果"],
        [
            ["70 接口", "模式11、P正定、单一CLF松弛、回退不认证", "全部断言成立，通过"],
            ["71 动力学", "3组状态下与复合刚体动力学逐点比较", "加速度/H/偏置误差<2×10⁻⁷，腿加速度误差<3×10⁻⁷，通过"],
            ["72 导数", "碰撞与奇异性梯度/Hessian、下界关系", "梯度<2×10⁻⁵，Hessian方向<3×10⁻⁴，下界方向成立，通过"],
            ["73 几何", "13个位姿与独立圆柱体—OBB凸距离比较", "证书≤独立距离+2 μm，顶板保守量<20 μm，通过"],
            ["74 QP", "7变量、40行CBF、64个硬约束及不可行情形", "可行点违反≤10⁻⁸；不可行必须非认证回退，通过"],
            ["75 闭环", "六组精确模型证据、保证范围和证书方向", "名义0不可行/回退，QP 95%≤10 ms，通过"],
        ],
        [1250, 3500, 4082],
        left_columns={0, 1, 2},
        font_size=9.0,
    )
    base.add_paragraph(doc, "Simscape初始状态审计的屏障状态最小值为0.0284159，HOCBF递归量最小值为0.113664，最小硬约束残差为0.471881；初始QP可行且未回退。三项来自不同量纲的诊断向量，只用于判断符号和递归可行性，不能相互比较物理大小。")
    base.add_heading(doc, "6.7.3 单步求解与后验认证流程", 3)
    base.add_paragraph(doc, "每个控制周期首先读取平台绝对位姿、世界坐标系线速度和角速度，并将角速度转换为与ZYX欧拉角一致的广义速度。参考轨迹同步给出位姿、速度和加速度，基线控制器据此生成Fnom。随后调用缓存的算法微分映射，一次性计算f2、g2、支链长度和速度、支链加速度仿射系数、碰撞证书及其一、二阶导数，以及奇异性下界及其导数。该顺序保证同一控制步内的目标函数和全部约束使用同一状态快照。")
    base.add_paragraph(doc, "约束组装阶段先形成CLF行，再依次加入腿长HOCBF、腿速CBF、腿加速度、三项碰撞HOCBF和奇异性HOCBF。力幅值区间与由上一时刻支链力确定的力率区间求交，作为当前决策变量的动态边界。QP使用上一时刻可行力作为热启动信息；初始化阶段预先构建算法图和预热求解器，因而冷启动开销与在线耗时分开统计。")
    base.add_paragraph(doc, "优化器返回后不直接接受其退出标志，而是把最优力重新代入全部不等式，计算最大硬约束违反量、CLF松弛量和活动约束集合。只有退出状态正常、变量有限且后验违反量不超过容差时，当前步才标记为可行。若任一条件不满足，控制器输出经幅值和力率限制的回退力，同时记录不可行原因与“安全未认证”标志；该控制步在统计和论文结论中均按失败处理。")
    base.add_paragraph(doc, "日志层保存物理状态、证书状态、真实几何距离、奇异值、求解与完整单步耗时，并区分命令侧模型残差和执行器作用后的实现侧残差。该区分对非理想执行器和参数失配试验尤为重要：命令侧QP可行只说明名义模型约束存在解，不能替代真实执行输入和真实状态对屏障条件的满足。")

    base.add_heading(doc, "6.8 闭环仿真与结果讨论", 2)
    base.add_heading(doc, "6.8.1 验证层级与统计口径", 3)
    base.add_paragraph(doc, "验证分为三层。第一层采用与控制器一致的精确非线性模型，以四阶Runge-Kutta方法按10 ms控制周期和2个积分子步推进7.5 s，对计算力矩-LQI、CLF-QP和完整CLF-CBF-QP作同轨迹消融，并考察30 ms力滞后、质量与转动惯量增加20%及组合质心偏移。第二层采用匹配参数、理想力输入的Simscape多体模型。第三层采用非理想力执行器，考察名义参数、时间常数增至2倍、质心偏移2 mm以及载荷质量增加2%、5%和10%。")
    base.add_paragraph(doc, "精确模型的平移和姿态RMSE是每时刻误差向量二范数的均方根；Simscape采用三个分量与全部时刻合并后的分量均方根。两张表只在各自口径内比较，不跨表计算改善率。控制能量统一为支链力平方积分。")
    base.add_heading(doc, "6.8.2 精确模型消融", 3)
    base.add_paragraph(doc, "表6-3显示，计算力矩-LQI和CLF-QP的最小物理碰撞裕度分别为-0.783094 mm和-0.786030 mm。负裕度表示未保持规定间隙，并不表示圆柱体已经与顶板相交。完整CLF-CBF-QP把该裕度提高到+0.422003 mm，腿加速度峰值降至0.462263 m/s²，同时保持接近的位姿误差与控制能量，说明改善来自硬CBF层而不是单独的CLF。")
    base.add_three_line_table(
        doc,
        "表6-3  精确非线性模型下三种控制方法的消融对比",
        ["指标", "计算力矩-LQI", "CLF-QP", "完整CLF-CBF-QP"],
        [
            ["平移RMSE/峰值（mm）", "1.164/1.844", "1.169/1.850", "1.161/1.850"],
            ["姿态RMSE/峰值（°）", "0.1385/0.2264", "0.1391/0.2273", "0.1383/0.2272"],
            ["腿长范围（m）", "1.07835～1.90747", "1.07835～1.90748", "1.07836～1.90638"],
            ["腿速/加速度峰值", "0.2791/1.1395", "0.2791/1.1380", "0.2791/0.4623"],
            ["支链力/力率峰值", "1104.39/3264.02", "1104.42/3257.50", "1104.42/2838.89"],
            ["实际/下界奇异值最小", "0.28317/0.21329", "0.28316/0.21329", "0.28316/0.21329"],
            ["几何距离最小（mm）", "4.21691", "4.21397", "5.42200"],
            ["物理碰撞裕度最小（mm）", "-0.783094", "-0.786030", "+0.422003"],
            ["CLF松弛最大/平均", "—", "6.04×10⁻⁵/2.17×10⁻⁵", "1.97×10⁻⁴/2.16×10⁻⁵"],
            ["控制能量（10⁷ N²·s）", "1.340642", "1.340694", "1.340643"],
            ["不可行/回退", "0/0", "0/0", "0/0"],
            ["综合验收", "跟踪通过/边界失败", "跟踪通过/边界失败", "通过"],
        ],
        [2600, 2077, 2077, 2078],
        left_columns={0},
        font_size=9.0,
    )
    base.add_figure(doc, FIG_ABLATION, "图6-1  精确非线性模型下三种控制方法的安全指标消融", width_in=5.85)
    base.add_paragraph(doc, "完整QP在751个节点均可行且无回退，最小硬CBF残差为-1.45128×10⁻¹²，最大CLF松弛为1.96742×10⁻⁴。CLF约束在550个节点活动，顶板碰撞约束在30个节点活动，第3、6条腿长度下界各活动1个节点。QP耗时均值和95%分位分别为2.653 ms和4.042 ms，满足统计验收；最大24.232 ms说明仍存在单次超期，不能声称最坏情况硬实时。")
    base.add_figure(doc, FIG_BARRIERS, "图6-2  精确模型名义严格QP的碰撞、奇异性与屏障诊断", width_in=5.85)

    base.add_heading(doc, "6.8.3 Simscape理想力与名义执行器", 3)
    base.add_paragraph(doc, "匹配参数理想力Simscape工况完成7.5 s仿真，751个节点均可行，无回退和求解器错误；最小物理碰撞裕度0.504493 mm，最小实际奇异值0.284341。名义非理想执行器也无不可行与回退，最小物理碰撞裕度0.583759 mm，最小奇异值0.284115。由于后者实际力动态不属于式（6-4b）的理想输入通道，其通过结果只构成本工况的工程证据。")
    add_continued_three_line_table(
        doc,
        "表6-4  Simscape名义工况结果",
        ["指标", "匹配参数理想力", "名义非理想执行器"],
        [
            ["平移分量RMS/峰值（mm）", "0.5653/1.6422", "0.4806/1.1899"],
            ["姿态分量RMS/峰值（°）", "0.06017/0.17948", "0.05890/0.13917"],
            ["腿长范围（m）", "1.07842～1.90432", "1.07842～1.90448"],
            ["腿速峰值（m/s）", "0.278871", "0.279044"],
            ["模型/实测腿加速度峰值", "0.26793/0.21267", "0.26046/0.20548"],
            ["命令力/力率峰值", "1098.56 N/493.35 N·s⁻¹", "1100.50 N/494.10 N·s⁻¹"],
            ["实际/下界奇异值最小", "0.284341/0.213824", "0.284115/0.213716"],
            ["几何距离/物理裕度（mm）", "5.50449/+0.504493", "5.58376/+0.583759"],
            ["硬CBF残差最小", "+1.24×10⁻¹⁴", "-4.97×10⁻¹⁴"],
            ["CLF松弛最大", "2.75×10⁻⁴", "1.83×10⁻⁴"],
            ["不可行/回退/错误", "0/0/0", "0/0/0"],
            ["活动节点/综合验收", "35/通过", "31/通过"],
        ],
        [3000, 2916, 2916],
        split_at=5,
        left_columns={0},
        font_size=9.0,
    )
    base.add_figure(doc, FIG_IDEAL, "图6-3  匹配参数理想力Simscape的跟踪、硬CBF残差与QP耗时", width_in=5.75)
    base.add_paragraph(doc, "理想力工况在QP求解时的最小硬CBF残差为+1.2434×10⁻¹⁴，但相对“物理阈值+0.5 mm”的加严证书集合，采样状态最小值为-0.147940 mm、最小ψ1为-0.00441751；扣除缓冲并按原物理阈值评估后，证书最小裕度为+0.352060 mm，独立几何裕度为+0.504493 mm。这表明离散零阶保持可使下一采样点轻微越出加严集合，0.5 mm缓冲吸收了本次偏差；本节只陈述记录到的物理安全裕度，不把采样实现写成连续时间定理的完全复现。")
    base.add_heading(doc, "6.8.4 在线耗时与鲁棒性边界", 3)
    base.add_three_line_table(
        doc,
        "表6-5  严格QP在线计算时间",
        ["工况", "QP均值", "QP 95%", "QP最大", "单步均值", "单步95%", "单步最大"],
        [
            ["精确模型CLF-QP", "2.123", "3.156", "53.441", "—", "—", "—"],
            ["精确模型完整QP", "2.653", "4.042", "24.232", "—", "—", "—"],
            ["Simscape理想力", "2.366", "2.944", "7.990", "4.953", "5.913", "11.453"],
            ["Simscape非理想", "2.380", "2.987", "7.792", "5.043", "5.995", "10.376"],
        ],
        [1900, 1155, 1155, 1155, 1155, 1155, 1157],
        left_columns={0},
        font_size=9.0,
    )
    base.add_paragraph(doc, "两类Simscape工况的QP 95%分位均小于3 ms，完整控制步95%分位均小于6 ms；完整单步最大值分别为11.453 ms和10.376 ms，仍有少量时刻超过10 ms周期。理想力工况算法图构建与冷启动总墙钟时间1.596 s，首次冷QP求解0.605 s；这些一次性开销未计入在线统计，但应纳入系统启动预算。")
    base.add_three_line_table(
        doc,
        "表6-6  执行器动态与参数失配的鲁棒性边界",
        ["模型/工况", "平移RMS/峰值", "最小碰撞裕度", "最小实际奇异值", "不可行/回退", "关键诊断与结论"],
        [
            ["精确模型，30 ms力滞后", "9.698/16.631 mm", "物理边界通过", "通过", "0/0", "实现侧CBF残差-0.2995，跟踪失败"],
            ["精确模型，质量/惯量+20%", "5.58 s提前终止", "—", "—", "505/505", "惯性矩阵病态，失败"],
            ["精确模型，质心[10,-8,5] mm", "跟踪失败", "物理边界失败", "失败", "348/348", "实现侧残差-1539.64，失败"],
            ["Simscape，名义非理想", "0.481/1.190 mm", "+0.5838 mm", "0.2841", "0/0", "逐点数值验收通过"],
            ["Simscape，时间常数×2", "0.528/1.191 mm", "+0.5816 mm", "0.2840", "0/0", "逐点数值验收通过"],
            ["Simscape，质心x+2 mm", "1.094/2.642 mm", "+0.5382 mm", "0.2823", "0/0", "按原阈值换算的控制器模型ψ₁=-6.22×10⁻⁴，失败"],
            ["Simscape，质量+2%", "6.589/33.018 mm", "+7.5215 mm", "0.2758", "1/1", "硬CBF残差-64.95，失败"],
            ["Simscape，质量+5%", "2292.6/14004 mm", "+20.014 mm", "5.37×10⁻⁵", "190/190", "腿速/加速度越界，失败"],
            ["Simscape，质量+10%", "8573.1/47575 mm", "+26.6408 mm", "1.29×10⁻⁵", "304/304", "状态发散，失败"],
        ],
        [1900, 1450, 1300, 1250, 1050, 1882],
        left_columns={0, 5},
        font_size=8.8,
    )
    base.add_paragraph(doc, "名义非理想执行器和时间常数×2均通过751个节点的数值验收，说明当前轨迹和控制带宽对所测试滞后增量有一定工程容忍度。质心偏移2 mm虽然QP无回退且独立几何距离为正，但按原阈值换算的控制器模型递归量ψ1为负，综合结果必须判为失败；该递归量不是依据偏移后几何重新构造的屏障。载荷质量增加2%即出现一次不可行和非认证回退，增加5%与10%时分别出现190次和304次回退及严重状态发散。发散运动可能使物体远离某些障碍物，正碰撞裕度不能据此解释为控制成功。")
    base.add_paragraph(doc, "各失配工况的QP和完整控制步95%分位仍小于10 ms，说明失败主要源于名义屏障模型失准和硬约束可行域丧失，而不是平均计算超时。当前CBF未显式包含质量、质心和输入动态的不确定性界，因此不能声称参数鲁棒安全保证。扩展适用范围需引入有界不确定性、鲁棒屏障裕度、在线参数辨识或更充分执行器余量，并重新完成可行域与采样数据验证。")

    base.add_heading(doc, "6.9 对初步约束QP的修正", 2)
    base.add_paragraph(doc, "第5章的初步方案保留了“最小修改名义支链力”的工程接口，但本章对其安全含义作出实质修正。名义输入严格沿用式（3-14）的计算力矩与LQI总力；跟踪性能由时变CLF和唯一性能松弛量表达；腿长、腿速、碰撞和奇异性分别采用具有正确相对阶的CBF或HOCBF；腿加速度、力与力变化率全部成为硬仿射约束。碰撞由固定法向支持函数下界认证，奇异性由最小奇异值平方的严格下界认证，导数统一来自算法微分。")
    base.add_paragraph(doc, "因此，第5章有关碰撞与奇异性的含义限定为在线监测和初步协调，其历史轻量级定量结果不进入本章结论。严格方案只有在结果文件可追溯、模型与采样条件明确、QP全程可行、无回退、硬CBF残差和真实物理指标均通过时，才被记为成功；非理想执行器结果只对应相应工况，质心偏移和载荷质量失配失败均作为适用边界保留。")

    base.add_heading(doc, "6.10 本章小结", 2)
    base.add_paragraph(doc, "本章在计算力矩-LQI总力之后构建七变量严格CLF-CBF-QP：CLF仅承担可松弛的跟踪要求，腿长、腿速、碰撞和奇异性屏障均无安全松弛，腿加速度、支链力与力率均为硬边界。保守碰撞证书、奇异性下界、算法微分和测试70～75共同构成数学验收链。")
    base.add_paragraph(doc, "完整方法在精确模型中把最小碰撞裕度由约-0.78 mm提高到+0.422 mm，理想力Simscape裕度为+0.504 mm。名义非理想执行器和时间常数×2通过逐点验收，质心偏移及质量增加工况失败，说明模型匹配、采样效应与QP持续可行性不能省略，当前方案不具备参数失配下的鲁棒安全保证。")


def add_final_conclusion(doc: Document) -> None:
    base.add_heading(doc, "结论", 1)
    base.add_paragraph(doc, "本文围绕Stewart并联机器人载荷挂装轨迹的闭环执行问题，建立了从六维位姿参考到六腿轴向力的统一控制描述。通过区分扭量Jacobian与广义坐标Jacobian，给出了运动学速度映射、功率一致的力映射和支链加速度仿射模型，并统一沿用式（2-10）的动力学输入映射和式（3-14）的名义力定义，使论文符号、控制接口与仿真单位保持一致。")
    base.add_paragraph(doc, "基线控制器由逆动力学计算力矩前馈和LQI输出反馈构成，并加入状态观测、积分泄漏和抗饱和回灌。重新运行的非理想力Simscape基线综合验收通过，最大平移误差0.788 mm、最大姿态误差0.146°、等效位姿RMSE 0.364 mm，最大腿加速度0.165 m/s²，说明基线在标称轨迹上具有良好跟踪性能。但精确模型消融显示，其最小碰撞裕度为-0.783 mm，固定控制律不能由一次标称验收推出边界邻域的前向安全性。")
    base.add_paragraph(doc, "为此，本文在名义总力之后构造严格CLF-CBF-QP。完整方法在精确模型中将最小碰撞裕度提高至+0.422 mm，751个节点无不可行和回退，QP求解时间95%分位为4.042 ms；匹配参数理想力Simscape的最小物理碰撞裕度为+0.504 mm、最小实际奇异值0.284，QP求解时间95%分位为2.944 ms。结果同时验证了碰撞证书的保守方向、奇异性下界关系和硬约束结构，说明在模型匹配与给定采样条件下，严格安全滤波层能够在基本保持跟踪性能的同时协调多类边界。")
    base.add_paragraph(doc, "理论保证的适用范围必须明确。前向不变性命题依赖连续时间、理想力输入、初始HOCBF递归条件、模型匹配、QP持续可行和局部Lipschitz控制解；10 ms零阶保持仿真属于采样实证。名义非理想执行器和时间常数×2工况通过，但质心偏移2 mm以及质量增加2%、5%和10%未通过，后者分别出现1、190和304次不可行与回退。由此可知，当前名义屏障模型尚不能提供参数失配下的鲁棒安全保证。后续研究应面向采样数据屏障条件、执行器动态扩维、模型不确定性界与在线参数辨识开展，并继续以不放宽物理边界、不删除失败数据为验证原则。")


def add_references(doc: Document) -> None:
    base.add_heading(doc, "参考文献", 1)
    refs = [
        "[1] STEWART D. A platform with six degrees of freedom[J]. Proceedings of the Institution of Mechanical Engineers, 1965, 180(1): 371-386.",
        "[2] MERLET J P. Parallel Robots[M]. 2nd ed. Dordrecht: Springer, 2006.",
        "[3] FICHTER E F. A Stewart platform-based manipulator: general theory and practical construction[J]. The International Journal of Robotics Research, 1986, 5(2): 157-182.",
        "[4] DASGUPTA B, MRUTHYUNJAYA T S. The Stewart platform manipulator: a review[J]. Mechanism and Machine Theory, 2000, 35(1): 15-40.",
        "[5] SPONG M W, HUTCHINSON S, VIDYASAGAR M. Robot Modeling and Control[M]. Hoboken: John Wiley & Sons, 2006.",
        "[6] SLOTINE J J E, LI W. Applied Nonlinear Control[M]. Englewood Cliffs: Prentice Hall, 1991.",
        "[7] LEWIS F L, VRABIE D L, SYRMOS V L. Optimal Control[M]. 3rd ed. Hoboken: John Wiley & Sons, 2012.",
        "[8] ANDERSON B D O, MOORE J B. Linear Optimal Control[M]. Englewood Cliffs: Prentice Hall, 1971.",
        "[9] ÅSTRÖM K J, RUNDQWIST L. Integrator windup and how to avoid it[C]//Proceedings of the 1989 American Control Conference. Pittsburgh: IEEE, 1989: 1693-1698.",
        "[10] BOYD S, VANDENBERGHE L. Convex Optimization[M]. Cambridge: Cambridge University Press, 2004.",
        "[11] AMES A D, XU X, GRIZZLE J W, et al. Control barrier function based quadratic programs for safety critical systems[J]. IEEE Transactions on Automatic Control, 2017, 62(8): 3861-3876.",
        "[12] KHALIL H K. Nonlinear Systems[M]. 3rd ed. Upper Saddle River: Prentice Hall, 2002.",
        "[13] SICILIANO B, SCIAVICCO L, VILLANI L, et al. Robotics: Modelling, Planning and Control[M]. London: Springer, 2009.",
        "[14] NGUYEN Q, SREENATH K. Exponential control barrier functions for enforcing high relative-degree safety-critical constraints[C]//Proceedings of the 2016 American Control Conference. Boston: IEEE, 2016: 322-328.",
        "[15] RAWLINGS J B, MAYNE D Q, DIEHL M. Model Predictive Control: Theory, Computation, and Design[M]. 2nd ed. Madison: Nob Hill Publishing, 2017.",
        "[16] NOCEDAL J, WRIGHT S J. Numerical Optimization[M]. 2nd ed. New York: Springer, 2006.",
        "[17] THE MATHWORKS, INC. Simscape Multibody User's Guide[EB/OL]. Natick: The MathWorks, Inc., 2026[2026-07-12]. https://www.mathworks.com/help/sm/.",
        "[18] XIAO W, BELTA C. High-order control barrier functions[J]. IEEE Transactions on Automatic Control, 2022, 67(7): 3655-3662.",
        "[19] ANDERSSON J A E, GILLIS J, HORN G, et al. CasADi: a software framework for nonlinear optimization and optimal control[J]. Mathematical Programming Computation, 2019, 11(1): 1-36.",
    ]
    seen = set()
    for ref in refs:
        key = re.sub(r"^\[\d+\]\s*", "", ref).casefold()
        if key in seen:
            raise RuntimeError(f"Duplicate reference: {ref}")
        seen.add(key)
        base.add_paragraph(doc, ref, first_line=False, align=WD_ALIGN_PARAGRAPH.JUSTIFY, style="Reference Thesis")


def package_text(path: Path) -> str:
    chunks: list[str] = []
    with zipfile.ZipFile(path) as archive:
        for name in archive.namelist():
            if name.endswith((".xml", ".rels")):
                chunks.append(archive.read(name).decode("utf-8", errors="ignore"))
    return "\n".join(chunks)


def structural_qa(path: Path) -> dict[str, object]:
    doc = Document(str(path))
    text = base.document_text(doc)
    xml = package_text(path)
    forbidden = "M" + "CP"
    if forbidden.casefold() in (text + xml).casefold():
        raise RuntimeError("Forbidden method string found")
    for marker in ("[[EQ:", "待补充"):
        if marker.casefold() in (text + xml).casefold():
            raise RuntimeError(f"Unresolved marker: {marker}")
    for marker in ("TODO", "placeholder"):
        if marker.casefold() in text.casefold():
            raise RuntimeError(f"Unresolved visible marker: {marker}")
    with zipfile.ZipFile(path) as archive:
        document_xml = archive.read("word/document.xml").decode("utf-8", errors="ignore")
    omath = len(re.findall(r"<m:oMath(?:\s|>)", document_xml))
    equation_paragraphs = sum(1 for p in doc.paragraphs if p._p.xpath(".//m:oMath"))
    headings = [p.text.strip() for p in doc.paragraphs if p.style.name.startswith("Heading")]
    if "第6章 严格CLF-CBF-QP补充研究" not in headings or "结论" not in headings:
        raise RuntimeError("Required headings missing")
    if headings.index("第6章 严格CLF-CBF-QP补充研究") > headings.index("结论"):
        raise RuntimeError("Chapter 6 is not before conclusion")
    if headings[-1] != "参考文献":
        raise RuntimeError("References are not last")
    if equation_paragraphs != 55 + NEW_EQUATION_COUNT:
        raise RuntimeError(f"Equation paragraph mismatch: {equation_paragraphs}")
    toc_texts = [
        p.text for p in doc.paragraphs
        if p.style.name in {"TOC 1", "TOC 2", "TOC 3"}
    ]
    toc_blob = " ".join(toc_texts)
    if "F_(nom)" in toc_blob or "H(q)" in toc_blob or "(6-" in toc_blob:
        raise RuntimeError("Equation text leaked into TOC")
    refs = [p.text.strip() for p in doc.paragraphs if p.style.name == "Reference Thesis"]
    captions = [p.text.strip() for p in doc.paragraphs if p.style.name == "Caption Thesis"]
    expected_sequences = {
        "chapter5_figures": (r"^图5-(\d+)\s", [1, 2]),
        "chapter5_tables": (r"^表5-(\d+)\s", [1, 2]),
        "chapter6_figures": (r"^图6-(\d+)\s", [1, 2, 3]),
        "chapter6_tables": (r"^表6-(\d+)\s", [1, 2, 3, 4, 5, 6]),
    }
    for name, (pattern, expected) in expected_sequences.items():
        actual = [int(match.group(1)) for caption in captions
                  if (match := re.match(pattern, caption))]
        if actual != expected:
            raise RuntimeError(f"Caption sequence mismatch for {name}: {actual}")
    continuation_count = sum(1 for caption in captions if caption.startswith("续表"))
    return {
        "omath_count": omath,
        "equation_paragraph_count": equation_paragraphs,
        "new_equations": NEW_EQUATION_COUNT,
        "heading_count": len(headings),
        "figure_count": len(doc.inline_shapes),
        "logical_table_count": 8,
        "physical_table_count": len(doc.tables),
        "continuation_table_count": continuation_count,
        "reference_count": len(refs),
        "forbidden_absent": True,
        "markers_absent": True,
        "toc_equation_text_absent": True,
    }


def build() -> dict[str, object]:
    for required in (WORKING_DOCX, FINALIZER, FIG_ABLATION, FIG_BARRIERS, FIG_IDEAL):
        if not required.exists():
            raise FileNotFoundError(required)
    QA_DIR.mkdir(parents=True, exist_ok=True)
    base.EQUATIONS.clear()
    doc = Document(str(WORKING_DOCX))
    update_front_matter(doc)
    update_initial_stage_language(doc)
    prepare_existing_chapter_five(doc)
    remove_old_conclusion_and_references(doc)
    build_chapter_six(doc)
    add_final_conclusion(doc)
    add_references(doc)
    set_two_level_toc(doc)
    enforce_table_pagination(doc)
    base.set_update_fields(doc)
    doc.core_properties.title = "并联机器人计算力矩-LQI与CLF-CBF-QP控制研究"
    doc.core_properties.subject = "计算力矩-LQI基线与严格CLF-CBF-QP控制研究"
    doc.core_properties.keywords = "Stewart; computed torque; LQI; CLF; CBF; quadratic programming; Simscape"
    doc.core_properties.comments = "第6章结果来自2026-07-12可追溯仿真；连续理论与10 ms采样证据分开表述。"
    doc.save(WORKING_DOCX)
    EQUATION_JSON.write_text(json.dumps(base.EQUATIONS, ensure_ascii=False, indent=2), encoding="utf-8")
    subprocess.run(
        [
            "powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", str(FINALIZER),
            "-DocxPath", str(WORKING_DOCX),
            "-EquationJson", str(EQUATION_JSON),
            "-PdfPath", str(PDF_PATH),
        ],
        cwd=WORKSPACE,
        check=True,
    )
    qa = structural_qa(WORKING_DOCX)
    qa.update({"docx": str(WORKING_DOCX), "pdf": str(PDF_PATH)})
    (QA_DIR / "structural_qa.json").write_text(json.dumps(qa, ensure_ascii=False, indent=2), encoding="utf-8")
    return qa


if __name__ == "__main__":
    print(json.dumps(build(), ensure_ascii=False, indent=2))
