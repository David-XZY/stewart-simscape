from __future__ import annotations

from copy import deepcopy
from io import BytesIO
from pathlib import Path
import shutil
import sys

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Emu, Pt


REFERENCE = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制.docx")
TECHNICAL = Path(r"D:\NPU\并联机器人\完善\并联机器人计算力矩_LQI与CLF_CBF_QP控制研究.docx")
OUTPUT = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩LQI逻辑重构稿.docx")


def set_paragraph_text(paragraph, text: str) -> None:
    paragraph.clear()
    paragraph.add_run(text)


def add_body(doc: Document, text: str):
    return doc.add_paragraph(text, style="Normal")


def add_heading3(doc: Document, text: str):
    return doc.add_paragraph(text, style="Heading 3")


def append_equation_from_source(doc: Document, src_doc: Document, src_idx: int, number: str):
    src_p = src_doc.paragraphs[src_idx]._p
    clone = deepcopy(src_p)
    for node in clone.xpath('.//w:t'):
        if node.text and node.text.strip().startswith("("):
            node.text = number
    body = doc._body._element
    sect_pr = body.find(qn("w:sectPr"))
    if sect_pr is None:
        body.append(clone)
    else:
        sect_pr.addprevious(clone)
    return clone


def add_text_equation(doc: Document, equation: str, number: str):
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = None
    p.paragraph_format.tab_stops.clear_all()
    r = p.add_run(equation)
    r.font.name = "Cambria Math"
    r.font.size = Pt(10.5)
    r2 = p.add_run(f"\t\t{number}")
    r2.font.name = "Times New Roman"
    r2.font.size = Pt(10.5)
    return p


def add_source_image(doc: Document, src_doc: Document, paragraph_index: int, width_emu: int):
    src_p = src_doc.paragraphs[paragraph_index]
    embeds = src_p._p.xpath('.//a:blip/@r:embed')
    if not embeds:
        raise ValueError(f"No embedded image in source paragraph {paragraph_index}")
    part = src_doc.part.related_parts[embeds[0]]
    stream = BytesIO(part.blob)
    stream.name = Path(str(part.partname)).name
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = None
    p.add_run().add_picture(stream, width=Emu(width_emu))
    return p


def shade_cell(cell, fill: str) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_width(cell, width_twips: int) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_w = tc_pr.find(qn("w:tcW"))
    if tc_w is None:
        tc_w = OxmlElement("w:tcW")
        tc_pr.append(tc_w)
    tc_w.set(qn("w:w"), str(width_twips))
    tc_w.set(qn("w:type"), "dxa")


def set_cell_margins(cell, top=50, start=90, bottom=50, end=90) -> None:
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for tag, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{tag}"))
        if node is None:
            node = OxmlElement(f"w:{tag}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def add_results_table(doc: Document) -> None:
    rows = [
        ("最大平移误差", "0.788 mm", "—", "标称轨迹跟踪误差小"),
        ("最大姿态误差", "0.146°", "—", "姿态跟踪保持平稳"),
        ("最大腿长误差", "约1.9 mm", "—", "支链长度跟踪连续"),
        ("最大腿速", "0.279 m/s", "0.60 m/s", "满足边界"),
        ("最大腿加速度", "0.165 m/s²", "1.20 m/s²", "满足边界"),
        ("支链总驱动力峰值", "1094.59 N", "±2000 N", "满足边界"),
        ("指令力—实际力偏差", "RMS 0.156 N；峰值1.251 N", "—", "执行器可较好跟随"),
        ("LQI反馈力峰值", "4.19 N", "反馈保护120 N", "标称工况以前馈承担主要载荷"),
    ]
    table = doc.add_table(rows=1, cols=4)
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    widths = [2150, 2250, 1600, 3000]
    headers = ["评价指标", "仿真结果", "约束或保护值", "结果说明"]
    for j, text in enumerate(headers):
        cell = table.rows[0].cells[j]
        set_cell_width(cell, widths[j])
        shade_cell(cell, "D9EAF7")
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        set_cell_margins(cell)
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.first_line_indent = None
        run = p.add_run(text)
        run.bold = True
        run.font.size = Pt(9)
    for row_data in rows:
        row = table.add_row()
        for j, text in enumerate(row_data):
            cell = row.cells[j]
            set_cell_width(cell, widths[j])
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            p = cell.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if j in (1, 2) else WD_ALIGN_PARAGRAPH.LEFT
            p.paragraph_format.first_line_indent = None
            run = p.add_run(text)
            run.font.size = Pt(9)
        tr_pr = row._tr.get_or_add_trPr()
        cant_split = OxmlElement("w:cantSplit")
        tr_pr.append(cant_split)
    header_tr_pr = table.rows[0]._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    header_tr_pr.append(tbl_header)
    tbl_pr = table._tbl.tblPr
    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.insert(0, tbl_w)
    tbl_w.set(qn("w:w"), str(sum(widths)))
    tbl_w.set(qn("w:type"), "dxa")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths:
        grid_col = OxmlElement("w:gridCol")
        grid_col.set(qn("w:w"), str(width))
        grid.append(grid_col)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(REFERENCE, OUTPUT)
    doc = Document(OUTPUT)
    technical = Document(TECHNICAL)

    set_paragraph_text(
        doc.paragraphs[1],
        "本章研究Stewart并联机器人的轨迹跟踪与在线约束控制。首先以传统PID控制作为基础对照，说明仅依靠误差反馈时的作用与局限；随后引入计算力矩前馈，并依次从LQR状态反馈和LQI积分增广说明计算力矩-LQI基线控制器的形成过程；最后结合Simscape仿真结果分析其跟踪能力与结构性不足，由此引出在名义控制力之后增加QP在线校正层的必要性。",
    )

    # Complete the otherwise sparse PID subsection and create an explicit bridge.
    set_paragraph_text(
        doc.paragraphs[3],
        "传统PID控制器以位姿误差及其积分、微分项直接生成六条支链的反馈修正量。该方法结构简单、参数含义直观，适合作为轨迹跟踪的基础对照；但其控制作用主要由误差触发，不能在误差产生之前主动补偿平台惯性、重力、科氏力及六自由度耦合载荷。对于负载较大或参考轨迹加速度较高的Stewart平台，单纯提高PID增益还可能放大测量噪声并增加超调与执行器负担。",
    )
    set_paragraph_text(
        doc.paragraphs[6],
        "因此，PID小节给出的结果只能说明基础误差反馈能够实现轨迹跟随，并不能回答“参考运动本身需要多大驱动力”这一问题。为把主要动力学载荷从反馈环节中分离出来，下一节首先引入计算力矩前馈；随后再以LQR/LQI处理模型误差、外部扰动和稳态偏差，形成“模型前馈承担主载荷、最优反馈修正剩余误差”的递进结构。",
    )

    set_paragraph_text(
        doc.paragraphs[8],
        "本节在4.1节基础PID控制的基础上，依次讨论计算力矩前馈的引入动机与动力学原理、计算力矩之后仍需最优反馈的原因、LQR向LQI增广的必要性，并结合仿真结果分析该基线控制器的能力与局限。最终得到的计算力矩-LQI总力将作为后续QP在线约束控制的名义输入。",
    )

    add_heading3(doc, "4.2.1 计算力矩前馈的引入与原理")
    add_body(
        doc,
        "基础PID完全依赖跟踪误差产生控制力。当平台携带较大载荷或执行快速轨迹时，重力、惯性力和多自由度耦合项必须先转化为误差，反馈控制器才能作出补偿。这会造成明显的相位滞后，并迫使比例、微分增益承担本应由动力学模型预测的载荷。计算力矩前馈的核心思想是：利用参考位姿、参考速度和参考加速度，在误差尚未形成时直接计算完成参考运动所需的合力与合力矩。",
    )
    add_body(
        doc,
        "设参考位姿为q_r，参考平台扭量及其导数为ν_r和ν̇_r，M、C和g分别表示平台等效质量矩阵、科氏/离心项和重力项。参考运动所需平台空间力旋量W_r写为",
    )
    append_equation_from_source(doc, technical, 141, "(4-1)")
    add_body(
        doc,
        "Stewart平台的执行器输入是六条支链轴向力，而式（4-1）给出的是平台空间力旋量。根据瞬时功率守恒，二者通过平台扭量Jacobian的转置映射。于是计算力矩支链力F_CT满足",
    )
    append_equation_from_source(doc, technical, 143, "(4-2)")
    append_equation_from_source(doc, technical, 144, "(4-3)")
    add_body(
        doc,
        "当J_v^T条件良好时可直接求解线性方程；接近奇异位形时采用Moore-Penrose伪逆可避免直接求逆造成的数值放大，但这只改善数值求解，并不能消除机构接近奇异时的物理风险。与基础PID相比，计算力矩前馈能够提前补偿参考加速度、重力和耦合载荷，降低反馈增益需求并减小快速轨迹中的滞后。然而，它本质上仍是依赖模型的开环预测：质量、惯量、质心、摩擦或执行器动态一旦与模型不一致，剩余误差仍需要反馈环节修正。",
    )

    add_heading3(doc, "4.2.2 计算力矩之后引入LQR的必要性与原理")
    add_body(
        doc,
        "计算力矩只回答了“按标称模型完成参考运动需要多大力”，不能保证实际平台始终与参考轨迹重合。初始状态偏差、负载参数误差、未建模摩擦、传感噪声以及执行器滞后都会使实际运动偏离标称轨迹。因此，计算力矩之后仍需闭环反馈；此时反馈的职责不再是承担全部动力学载荷，而是稳定误差动态并抑制模型失配和扰动。",
    )
    add_body(
        doc,
        "在工作点附近，将“支链反馈力—平台位姿偏差”通道线性化。设x为局部状态偏差，u=F_LQR为反馈修正力，y=Cx为可测位姿偏差，则局部模型为",
    )
    append_equation_from_source(doc, technical, 147, "(4-4)")
    add_body(
        doc,
        "LQR通过在无限时域内同时惩罚状态偏差和控制输入，求取稳定闭环且综合代价最小的状态反馈增益。其性能指标和控制律可写为",
    )
    add_text_equation(doc, "J_LQR = ∫₀^∞(xᵀQx + uᵀRu)dt，  u_LQR = −Kx", "(4-5)")
    add_body(
        doc,
        "其中Q⪰0决定不同状态误差的相对权重，R≻0限制反馈力幅值；K由连续代数Riccati方程得到。与逐通道PID调参相比，LQR在同一二次型指标中处理多变量耦合，使六条支链的反馈力分配具有统一的优化依据。但LQR仍以瞬时状态误差为调节对象，当系统存在常值负载偏差或模型中未包含的恒定扰动时，闭环可能保留稳态误差。",
    )

    add_heading3(doc, "4.2.3 从LQR到LQI的增广设计")
    add_body(
        doc,
        "为消除LQR在常值扰动和静态模型偏差下可能出现的稳态误差，将位姿误差积分作为新增状态。令e_q=q−q_r，积分状态x_I满足ẋ_I=−e_q，并与原状态组成增广状态x_a：",
    )
    append_equation_from_source(doc, technical, 149, "(4-6)")
    append_equation_from_source(doc, technical, 150, "(4-7)")
    add_body(
        doc,
        "在增广系统上应用与LQR相同的二次型最优控制思想，即得到线性二次型积分控制LQI。增广性能指标与反馈修正力分别为",
    )
    append_equation_from_source(doc, technical, 152, "(4-8)")
    append_equation_from_source(doc, technical, 153, "(4-9)")
    add_body(
        doc,
        "式中K_x调节瞬时动态误差，K_I利用累计误差恢复零稳态偏差。LQI不是对计算力矩的替代，而是其误差修正层：计算力矩负责已知模型下的主要载荷，LQI只补偿两者之间的差异。对于不能直接测量的线性化状态，可采用Luenberger观测器重构；为避免执行器饱和、死区和速率限制导致积分持续累积，还需引入积分泄漏和基于实际执行力的抗饱和回灌：",
    )
    append_equation_from_source(doc, technical, 160, "(4-10)")
    lead = add_body(
        doc,
        "计算力矩前馈与LQI反馈相加，形成完整名义支链力",
    )
    lead.paragraph_format.keep_with_next = True
    append_equation_from_source(doc, technical, 163, "(4-11)")

    add_heading3(doc, "4.2.4 计算力矩-LQI控制结构与实现")
    add_source_image(doc, technical, 131, 4754880)
    cap = doc.add_paragraph("图4.2  计算力矩、LQI与非理想力执行器控制结构", style="Normal")
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.first_line_indent = None
    add_body(
        doc,
        "如图4.2所示，参考轨迹经连续重建后同时提供q_r、q̇_r和q̈_r。计算力矩模块仅根据参考状态和动力学参数生成F_CT；LQI模块根据实际位姿、观测状态和积分状态生成F_LQI；二者相加得到F_nom，再经延迟、死区、力变化率限制、一阶滞后和饱和等非理想执行器环节作用于平台。该并联结构保持了前馈与反馈的职责可解释，也为后续QP在不破坏标称性能的前提下修改F_nom提供了清晰接口。",
    )

    add_heading3(doc, "4.2.5 仿真结果与分析")
    add_body(
        doc,
        "采用完整两阶段参考轨迹、0.01 s控制周期和非理想力执行器进行Simscape闭环仿真。结果表明，计算力矩承担了绝大部分重力和轨迹运动载荷，LQI只需较小修正即可获得稳定跟踪。主要结果汇总如下。",
    )
    cap = doc.add_paragraph("表4.1  计算力矩-LQI基线控制主要仿真指标", style="Normal")
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.first_line_indent = None
    add_results_table(doc)
    add_source_image(doc, technical, 232, 5074920)
    cap = doc.add_paragraph("图4.3  计算力矩-LQI基线位姿与腿长跟踪误差", style="Normal")
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.first_line_indent = None
    add_source_image(doc, technical, 235, 5257800)
    cap = doc.add_paragraph("图4.4  六条支链速度、加速度及其边界", style="Normal")
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.first_line_indent = None
    add_body(
        doc,
        "图4.3和图4.4显示，位姿误差在轨迹阶段转换处保持连续，腿速、腿加速度和支链力在本次标称工况下均未越界。LQI反馈力峰值仅4.19 N，而支链总驱动力峰值为1094.59 N，进一步说明“计算力矩承担主载荷、LQI修正剩余误差”的设计分工与仿真表现一致。需要强调的是，这些结论只对应当前模型、轨迹、初始条件和执行器参数，不能由一次标称仿真外推为所有边界工况下的约束保证。",
    )

    add_heading3(doc, "4.2.6 局限性及QP控制的引出")
    add_body(
        doc,
        "计算力矩-LQI显著改善了基础PID的跟踪性能，但仍存在四类结构性局限。第一，计算力矩对质量、惯量、质心和重力方向等参数敏感，模型失配会直接转化为反馈扰动；第二，LQI增益依据单一工作点的线性化模型和固定权重离线求得，不能自动适应Jacobian、惯量映射和控制分配效率随位姿发生的变化；第三，积分泄漏和抗饱和只能改善受限后的恢复过程，逐通道限幅仍可能改变六条支链合成的平台力旋量；第四，腿长、腿速、腿加速度、支链力、碰撞间隙和奇异性裕度并未进入控制律的统一在线决策，因此“标称仿真未越界”不等价于安全集合得到前向不变性保证。",
    )
    add_body(
        doc,
        "基于上述局限，后续控制器不再替换计算力矩-LQI，而是在其名义总力F_nom之后增加QP在线校正层。QP以“尽量保持F_nom”为目标，同时把执行器能力和状态安全条件写成显式约束：当约束远离活动边界时，输出基本保持原有跟踪性能；当某项约束趋于活动时，在线协调六条支链力，避免简单饱和造成的耦合失真。其基本形式可概括为",
    )
    add_text_equation(doc, "F_cmd = arg min_F ‖F − F_nom‖²_W，  s.t.  A(q,q̇)F ≤ b(q,q̇)", "(4-12)")
    add_body(
        doc,
        "因此，本节得到的计算力矩-LQI并非控制研究的终点，而是后续QP约束控制的高质量名义控制器：前馈提供动力学精度，LQI提供局部误差收敛和稳态消差，QP则进一步负责在多类约束之间进行在线协调。",
    )

    settings = doc.settings._element
    update = settings.find(qn("w:updateFields"))
    if update is not None:
        settings.remove(update)
    doc.save(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    main()
