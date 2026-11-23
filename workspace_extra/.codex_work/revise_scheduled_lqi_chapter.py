import os
from copy import deepcopy

from docx import Document
from docx.oxml.ns import qn


DOCX_PATH = r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿.docx"
ERROR_FIGURE = r"D:\NPU\Matlab_code\stewart-simscape\results\reports\lqi_schedule_comparison\fixed_vs_scheduled_lqi_errors.png"
FORCE_FIGURE = r"D:\NPU\Matlab_code\stewart-simscape\results\reports\lqi_schedule_comparison\fixed_vs_scheduled_lqi_force_sigma.png"


def replace_paragraph(paragraph, text):
    p = paragraph._p
    ppr = p.pPr
    for child in list(p):
        if child is not ppr:
            p.remove(child)
    run = paragraph.add_run(text)
    if paragraph.style is not None:
        run.font.name = "宋体"
        rfonts = run._element.get_or_add_rPr().get_or_add_rFonts()
        rfonts.set(qn("w:eastAsia"), "宋体")


def replace_matching_paragraph(doc, prefix, text, required=True):
    matches = [p for p in doc.paragraphs if p.text.strip().startswith(prefix)]
    if not matches and any(p.text == text for p in doc.paragraphs):
        return
    if required and len(matches) != 1:
        raise RuntimeError(f"Expected one paragraph beginning with {prefix!r}, found {len(matches)}")
    if matches:
        replace_paragraph(matches[0], text)


def set_cell_text(cell, text):
    paragraph = cell.paragraphs[0]
    replace_paragraph(paragraph, text)
    for extra in cell.paragraphs[1:]:
        extra._element.getparent().remove(extra._element)


def replace_inline_image(doc, image_index, image_path):
    if not os.path.isfile(image_path):
        raise FileNotFoundError(image_path)
    shape = doc.inline_shapes[image_index]
    relationship_id, image_part = doc.part.get_or_add_image(image_path)
    shape._inline.graphic.graphicData.pic.blipFill.blip.embed = relationship_id


def set_inline_alt_text(doc, image_index, description):
    doc.inline_shapes[image_index]._inline.docPr.set("descr", description)
    doc.inline_shapes[image_index]._inline.docPr.set("title", description)


doc = Document(DOCX_PATH)

replacements = {
    "第4章 Stewart平台轨迹跟踪控制": "第4章 Stewart平台轨迹跟踪控制",
    "本章研究Stewart并联机器人的轨迹跟踪与在线约束控制。": (
        "本章研究Stewart并联机器人的轨迹跟踪与在线约束控制。首先以传统PID控制作为基础对照，说明仅依靠误差反馈时的作用与局限；随后引入计算力矩前馈，并依次从局部LQR状态反馈、LQI积分增广以及沿参考轨迹离线调度说明计算力矩-调度LQI基线控制器的形成过程；最后结合固定LQI与调度LQI的对比结果分析其跟踪能力与结构性不足，由此引出在名义控制力之后增加QP在线校正层的必要性。"
    ),
    "因此，PID小节给出的结果只能说明": (
        "因此，PID小节给出的结果只能说明基础误差反馈能够实现轨迹跟随，并不能回答“参考运动本身需要多大驱动力”这一问题。为把主要动力学载荷从反馈环节中分离出来，下一节首先引入计算力矩前馈；随后再以局部LQR/LQI处理模型误差、外部扰动和稳态偏差，并使反馈增益随已知参考轨迹调度，形成“模型前馈承担主载荷、时变最优反馈修正剩余误差”的递进结构。"
    ),
    "4.2 Stewart平台计算力矩LQI控制": "4.2 Stewart平台计算力矩-调度LQI控制",
    "本节在4.1节基础PID控制的基础上": (
        "本节在4.1节基础PID控制的基础上，依次讨论计算力矩前馈的引入动机与动力学原理、计算力矩之后仍需局部最优反馈的原因、LQR向LQI增广及其沿参考轨迹调度的必要性，并结合固定LQI与调度LQI的对比结果分析该基线控制器的能力与局限。最终得到的计算力矩-调度LQI总力将作为后续QP在线约束控制的名义输入。"
    ),
    "当J_v^T条件良好时": (
        "当J_v^T条件良好时可直接求解线性方程；接近奇异位形时采用Moore-Penrose伪逆可避免直接求逆造成的数值放大，但这只改善数值求解，并不能消除机构接近奇异时的物理风险。与基础PID相比，计算力矩前馈能够提前补偿参考加速度、重力和耦合载荷，降低反馈增益需求并减小快速轨迹中的滞后。需要强调的是，本研究的前馈项按参考状态计算C(q_r,q̇_r)q̇_r等标称项，而不是利用实际状态实施严格反馈线性化；质量、惯量、质心、摩擦或执行器动态一旦与模型不一致，剩余误差仍需要反馈环节修正。"
    ),
    "4.2.2 计算力矩之后引入LQR": "4.2.2 计算力矩之后引入局部LQR的必要性与原理",
    "计算力矩只回答了": (
        "计算力矩只回答了“按标称模型完成参考运动需要多大力”，不能保证实际平台始终与参考轨迹重合。初始状态偏差、负载参数误差、未建模摩擦、传感噪声以及执行器滞后都会使实际运动偏离标称轨迹。因此，计算力矩之后仍需闭环反馈；此时反馈的职责不再是承担全部动力学载荷，而是稳定残余误差动态并抑制模型失配和扰动。"
    ),
    "在工作点附近，将“支链反馈力": (
        "在每个参考采样时刻t_k，以已知的(q_r(k),q̇_r(k),F_CT(k))作为局部工作点，对合成刚体非线性状态方程ẋ=f(x,F)作一阶展开。令δx=x-x_r(k)、δF=F-F_CT(k)，并取y=Cδx为可测位姿偏差，则得到第k个局部模型δẋ=A_kδx+B_kδF，y=Cδx。这里A_k采用中心差分计算，B_k的加速度部分由H(q_r,q̇_r)^{-1}J_v(q_r)^T给出；因此该模型描述的是参考轨迹附近的局部误差通道，而不是把真实平台在全工作空间内变为线性系统。"
    ),
    "LQR通过在无限时域内": (
        "对每个局部模型，LQR通过在无限时域内同时惩罚状态偏差和控制输入，求取稳定闭环且综合代价最小的状态反馈增益。其性能指标和控制律仍可写为式（4-5），但增益随参考采样时刻记为K(k)。与逐通道PID调参相比，LQR在同一二次型指标中处理多变量耦合，使六条支链的反馈力分配具有统一的优化依据。"
    ),
    "其中Q⪰0决定": (
        "其中Q⪰0决定不同状态误差的相对权重，R≻0限制反馈力幅值；对每个局部模型，K(k)由连续代数Riccati方程得到。设计成立的前提是相应局部线性化对满足可稳定、可检测条件，且Jacobian与质量矩阵条件良好。LQR仍以瞬时状态误差为调节对象；当系统存在常值负载偏差或模型中未包含的恒定扰动时，闭环仍可能保留稳态误差。"
    ),
    "4.2.3 从LQR到LQI": "4.2.3 从LQR到参考轨迹调度LQI的增广设计",
    "为消除LQR在常值扰动": (
        "为消除局部LQR在常值扰动和静态模型偏差下可能出现的稳态误差，将位姿误差积分作为新增状态。令e_q=q−q_r，积分状态x_I满足ẋ_I=−e_q，并与原状态组成增广状态x_a，如式（4-6）和式（4-7）所示。对第k个局部模型，A、B分别替换为A_k、B_k，从而得到对应的增广系统。"
    ),
    "在增广系统上应用": (
        "在每个增广局部系统上应用与LQR相同的二次型最优控制思想，即得到线性二次型积分控制LQI。增广性能指标与反馈修正力仍遵循式（4-8）和式（4-9），但增益写为K_x(k)、K_I(k)。由此形成沿参考轨迹变化的一组离线控制器，而非单一工作点上的固定增益。"
    ),
    "式中K_x调节": (
        "式中K_x(k)调节第k个参考工作点附近的瞬时动态误差，K_I(k)利用累计误差恢复零稳态偏差。调度LQI不是对计算力矩的替代，而是其误差修正层：计算力矩负责已知模型下的主要载荷，调度LQI补偿两者之间的差异。离线阶段在完整参考轨迹的每个0.01 s采样点预计算A_k、B_k、K_x(k)、K_I(k)及观测器增益L(k)，并检查Jacobian、质量矩阵条件数以及闭环极点；任何一个采样点病态或不稳定即拒绝构建调度表。运行时只按时间索引零阶保持选取已离散化的控制器矩阵，观测器状态和积分状态在相邻采样点之间连续传递，不在线重线性化，也不在线求解Riccati方程。对于不能直接测量的局部状态，可采用相应的Luenberger观测器重构；为避免执行器饱和、死区和速率限制导致积分持续累积，仍需保留积分泄漏和基于实际执行力的抗饱和回灌："
    ),
    "计算力矩前馈与LQI反馈相加": "计算力矩前馈与调度LQI反馈相加，形成完整名义支链力",
    "4.2.4 计算力矩-LQI控制结构": "4.2.4 计算力矩-调度LQI控制结构与实现",
    "图4.2  计算力矩、LQI": "图4.2  计算力矩、参考轨迹调度LQI与非理想力执行器控制结构",
    "如图4.2所示": (
        "如图4.2所示，参考轨迹经连续重建后同时提供q_r、q̇_r和q̈_r。计算力矩模块仅根据参考状态和动力学参数生成F_CT；在仿真开始前，调度器以同一参考轨迹离线建立并验证控制器序列{K_x(k),K_I(k),L(k)}。运行阶段，LQI模块根据实际位姿、观测状态和积分状态生成F_LQI(k)，并在每个0.01 s采样时刻零阶保持对应的离散控制器矩阵；它不以实际位姿重新线性化。二者相加得到F_nom，再经延迟、死区、力变化率限制、一阶滞后和饱和等非理想执行器环节作用于平台。该并联结构保持了前馈与反馈的职责可解释，也为后续QP在不破坏标称性能的前提下修改F_nom提供了清晰接口。"
    ),
    "4.2.5 仿真结果与分析": "4.2.5 固定LQI与调度LQI的对比结果与分析",
    "采用完整两阶段参考轨迹": (
        "为隔离“是否调度”这一变量，采用完整两阶段参考轨迹、0.01 s控制周期和同一合成刚体精确非线性模型，比较“计算力矩+固定LQI”与“计算力矩+参考轨迹调度LQI”。两者使用相同的计算力矩前馈、LQI权重、积分泄漏、反馈力保护、初始位姿偏差以及执行器模型；依次考察标称工况、移动体质量与惯量增加20%、质心偏置[10,−8,5] mm和一阶力执行器滞后30 ms四类工况。主要结果汇总如下。"
    ),
    "表4.1  计算力矩-LQI": "表4.1  计算力矩-调度LQI与固定LQI的主要对比指标",
    "图4.3  计算力矩-LQI": "图4.3  固定LQI与参考轨迹调度LQI在四类工况下的位姿误差对比",
    "图4.4  六条支链速度": "图4.4  固定LQI与参考轨迹调度LQI的反馈力、总驱动力及最小奇异值对比",
    "图4.3和图4.4显示": (
        "图4.3和图4.4显示，在标称工况下，调度LQI的平移RMSE为0.301 mm、姿态RMSE为0.0259°，相对固定LQI的0.664 mm和0.1465°分别降低54.7%和82.3%；两者的反馈力峰值均约为39.6 N，而调度LQI的总驱动力峰值略低，为1097.26 N。这说明计算力矩继续承担主要重力和轨迹运动载荷，调度LQI通过匹配不同参考位姿附近的局部动力学改善残余误差收敛。质心偏置和30 ms执行器滞后下，调度LQI也显著降低位姿误差，且未新增腿长、腿速、腿加速度或力越界。需要强调的是，质量与惯量增加20%时，虽然调度LQI的误差低于固定LQI，但两者均出现大量腿长、腿速和腿加速度越界；因此，调度只改善工作点变化造成的局部模型失配，不能由该对比外推为对参数大失配的鲁棒性保证或约束安全保证。"
    ),
    "4.2.6 局限性及QP": "4.2.6 调度LQI的局限性及QP控制的引出",
    "计算力矩-LQI显著改善": (
        "计算力矩-调度LQI显著改善了固定工作点LQI的跟踪性能，但仍存在四类结构性局限。第一，计算力矩仍对质量、惯量、质心和重力方向等参数敏感，模型失配会直接转化为反馈扰动；第二，调度增益依赖预先已知且平滑的参考轨迹以及标称模型，只随参考状态变化而不随实际测量状态在线重线性化，因而不能自动补偿大幅参数失配或未建模动力学；第三，积分泄漏和抗饱和只能改善受限后的恢复过程，逐通道限幅仍可能改变六条支链合成的平台力旋量；第四，腿长、腿速、腿加速度、支链力、碰撞间隙和奇异性裕度并未进入控制律的统一在线决策，因此“某次仿真未越界”不等价于安全集合得到前向不变性保证。"
    ),
    "基于上述局限": (
        "基于上述局限，后续控制器不再替换计算力矩-调度LQI，而是在其名义总力F_nom之后增加QP在线校正层。QP以“尽量保持F_nom”为目标，同时把执行器能力和状态安全条件写成显式约束：当约束远离活动边界时，输出基本保持原有跟踪性能；当某项约束趋于活动时，在线协调六条支链力，避免简单饱和造成的耦合失真。其基本形式可概括为"
    ),
    "因此，本节得到的计算力矩-LQI": (
        "因此，本节得到的计算力矩-调度LQI并非控制研究的终点，而是后续QP约束控制的高质量名义控制器：前馈提供标称动力学精度，参考轨迹调度LQI提供随局部工作点变化的误差收敛与稳态消差，QP则进一步负责在多类约束之间进行在线协调。"
    ),
}

for prefix, replacement in replacements.items():
    replace_matching_paragraph(doc, prefix, replacement)

table = doc.tables[0]
rows = [
    ["评价指标", "调度LQI仿真结果", "固定LQI对照", "结果说明"],
    ["标称平移RMSE", "0.301 mm", "0.664 mm", "调度下降54.7%"],
    ["标称姿态RMSE", "0.0259°", "0.1465°", "调度下降82.3%"],
    ["标称最大平移误差", "0.539 mm", "1.051 mm", "局部峰值减小"],
    ["标称最大姿态误差", "0.0467°", "0.2407°", "局部峰值减小"],
    ["LQI反馈力峰值", "39.61 N", "39.61 N", "反馈保护120 N以内"],
    ["支链总驱动力峰值", "1097.26 N", "1104.08 N", "均低于±2000 N"],
    ["质心偏置平移RMSE", "4.918 mm", "819.703 mm", "调度显著改善，且无新增腿部越界"],
    ["质量/惯量+20%", "30.42 m；5.619 rad", "38.48 m；21.351 rad", "两者均发生腿部边界越界，不构成鲁棒保证"],
]
for row, values in zip(table.rows, rows):
    for cell, value in zip(row.cells, values):
        set_cell_text(cell, value)

replace_inline_image(doc, 2, ERROR_FIGURE)
replace_inline_image(doc, 3, FORCE_FIGURE)
alt_text = [
    "Stewart平台PID闭环控制原理图。",
    "计算力矩前馈、参考轨迹调度LQI反馈和非理想力执行器控制结构图。",
    "固定LQI与参考轨迹调度LQI在标称、质量惯量失配、质心偏置和执行器滞后工况下的平移与姿态误差对比图。",
    "固定LQI与参考轨迹调度LQI在四类工况下的反馈力、总驱动力和Jacobian最小奇异值对比图。",
]
for image_index, description in enumerate(alt_text):
    set_inline_alt_text(doc, image_index, description)

doc.core_properties.title = "第四章 Stewart平台轨迹跟踪控制（计算力矩-调度LQI修订稿）"
doc.core_properties.subject = "参考轨迹调度LQI原理与固定LQI对比"
doc.save(DOCX_PATH)
print(DOCX_PATH)
