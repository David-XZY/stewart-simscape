from copy import deepcopy
from pathlib import Path
from shutil import copy2

from docx import Document
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

SRC = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_文本内公式OMML统一版.docx")
OUT = SRC.with_name("第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版.docx")


def word_run(text):
    run = OxmlElement("w:r")
    value = OxmlElement("w:t")
    if text.startswith(" ") or text.endswith(" "):
        value.set(qn("xml:space"), "preserve")
    value.text = text
    run.append(value)
    return run


def word_tab_run():
    run = OxmlElement("w:r")
    run.append(OxmlElement("w:tab"))
    return run


def omath(text):
    node = OxmlElement("m:oMath")
    run = OxmlElement("m:r")
    value = OxmlElement("m:t")
    if text.startswith(" ") or text.endswith(" "):
        value.set(qn("xml:space"), "preserve")
    value.text = text
    run.append(value)
    node.append(run)
    return node


def add_segments(paragraph, segments):
    for kind, value in segments:
        if kind == "text":
            paragraph.add_run(value)
        elif kind == "math":
            paragraph._p.append(omath(value))
        else:
            raise ValueError(f"Unknown segment kind: {kind}")


def add_body(doc, anchor, style, segments):
    paragraph = doc.add_paragraph(style=style)
    add_segments(paragraph, segments)
    anchor._p.addnext(paragraph._p)
    return paragraph


def add_formula(doc, anchor, formula_ppr, formula, label=None):
    paragraph = doc.add_paragraph()
    existing_ppr = paragraph._p.pPr
    if existing_ppr is not None:
        paragraph._p.remove(existing_ppr)
    paragraph._p.insert(0, deepcopy(formula_ppr))
    paragraph._p.append(word_tab_run())
    paragraph._p.append(omath(formula))
    if label:
        paragraph._p.append(word_tab_run())
        paragraph._p.append(word_run(label))
    anchor._p.addnext(paragraph._p)
    return paragraph


def body(text):
    return [("text", text)]


def write_revision():
    copy2(SRC, OUT)
    doc = Document(OUT)

    # Preserve the original local body and equation formatting as the source authority.
    heading = doc.paragraphs[17]
    normal_style = doc.paragraphs[18].style
    formula_ppr = deepcopy(doc.paragraphs[20]._p.pPr)

    # Replace only the old, abbreviated LQR block (paragraphs 18--23).
    for index in range(23, 17, -1):
        element = doc.paragraphs[index]._p
        element.getparent().remove(element)

    anchor = heading
    anchor = add_body(doc, anchor, normal_style, body(
        "计算力矩前馈给出了在标称动力学和标称参考轨迹下完成运动所需的支链力，"
        "但它本身不构成误差闭环。初始位姿偏差、质量或质心误差、摩擦、传感噪声以及执行器滞后都会使实际状态偏离参考状态。"
        "因此，在计算力矩之后还需要反馈校正；其职责不是重复承担全部重力和惯性载荷，"
        "而是在前馈已经消除主要标称载荷的基础上，使剩余误差动态稳定并尽可能快速衰减。"
    ))
    anchor = add_body(doc, anchor, normal_style, body(
        "为使该反馈问题能够系统设计，先将合成刚体动力学写成以平台位姿和广义速度为状态、"
        "以六条支链轴向驱动力为输入的非线性状态方程。状态、输入与状态导数的定义为"
    ))
    anchor = add_formula(doc, anchor, formula_ppr,
        "x=[qᵀ  q̇ᵀ]ᵀ，q=[pₓ  pᵧ  p_z  φ  θ  ψ]ᵀ，ẋ=f(x,F)")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "其中，"), ("math", "x∈R¹²"), ("text", "为12维状态；"),
        ("math", "q∈R⁶"), ("text", "依次包含三个平移坐标"),
        ("math", "pₓ,pᵧ,p_z"), ("text", "与绕三个坐标轴的姿态角"),
        ("math", "φ,θ,ψ"), ("text", "；"),
        ("math", "q̇"), ("text", "为对应的广义速度；"),
        ("math", "F∈R⁶"), ("text", "为六条支链的轴向力向量。上标"),
        ("math", "T"), ("text", "表示转置，"),
        ("math", "R"), ("text", "表示实数向量空间。该状态选择与仿真程序中的"),
        ("math", "x=[qᵀ,q̇ᵀ]ᵀ"), ("text", "完全一致，因此后续设计的反馈力可直接叠加到六条支链的计算力矩前馈上。")
    ])
    anchor = add_formula(doc, anchor, formula_ppr,
        "ẋ=[q̇ᵀ  (H(q)⁻¹[Jᵥ(q)ᵀF−Wᵦ(q,q̇)])ᵀ]ᵀ")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "式中，"), ("math", "H(q)∈R⁶ˣ⁶"), ("text", "为合成刚体质量矩阵；"),
        ("math", "Jᵥ(q)∈R⁶ˣ⁶"), ("text", "为平台扭量Jacobian；"),
        ("math", "Jᵥ(q)ᵀF"), ("text", "把支链力映射为作用于动平台的广义力；"),
        ("math", "Wᵦ(q,q̇)"), ("text", "为零加速度时由重力、速度耦合和质心偏置等形成的偏置广义力。"
            "因此，括号内的差表示可用于产生加速度的净广义力，左乘"),
        ("math", "H(q)⁻¹"), ("text", "后得到广义加速度。该式也说明了为什么接近质量矩阵或力映射病态时，"
            "不能仅依靠增大反馈增益解决问题。")
    ])
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "LQR只能直接用于线性模型，故在每个参考采样时刻"), ("math", "tₖ"),
        ("text", "，以参考状态"), ("math", "xᵣ,ₖ=[qᵣ,ₖᵀ  q̇ᵣ,ₖᵀ]ᵀ"),
        ("text", "和计算力矩前馈"), ("math", "FCT,ₖ"), ("text", "构成局部工作点。定义状态增量和输入增量为"),
        ("math", "δx=x−xᵣ,ₖ"), ("text", "与"), ("math", "δF=F−FCT,ₖ"), ("text", "。这里的"),
        ("math", "δF"), ("text", "是反馈修正力，而不是总支链力；总力仍等于计算力矩前馈与该修正力之和。")
    ])
    anchor = add_formula(doc, anchor, formula_ppr,
        "f(xᵣ,ₖ+δx,FCT,ₖ+δF)=f(xᵣ,ₖ,FCT,ₖ)+Aₖδx+BₖδF+O(‖δx‖²+‖δF‖²)")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "上式是多变量Taylor一阶展开。因为在标称模型内，"),
        ("math", "FCT,ₖ"), ("text", "由参考位姿、速度和加速度反求，满足"),
        ("math", "f(xᵣ,ₖ,FCT,ₖ)=ẋᵣ,ₖ"), ("text", "；"
            "将这一等式从实际状态方程中相减，并忽略关于"),
        ("math", "δx"), ("text", "和"), ("math", "δF"), ("text", "的二阶及更高阶小量，"
            "便得到只描述参考轨迹邻域内残余误差的线性模型，而非把全工作空间的真实平台误认为线性系统。")
    ])
    anchor = add_formula(doc, anchor, formula_ppr,
        "δẋ=Aₖδx+BₖδF，y=Cδx", "(4-4)")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "其中，"), ("math", "Aₖ=∂f/∂x|₍xᵣ,ₖ,FCT,ₖ₎∈R¹²ˣ¹²"),
        ("text", "是状态Jacobian，刻画小位姿、速度偏差如何影响状态导数；"),
        ("math", "Bₖ=∂f/∂F|₍xᵣ,ₖ,FCT,ₖ₎∈R¹²ˣ⁶"),
        ("text", "是输入Jacobian，刻画各支链力增量如何改变状态导数；"),
        ("math", "C=[I₆  0₆ˣ⁶]∈R⁶ˣ¹²"), ("text", "选择可测的六维位姿偏差，"
            "所以"), ("math", "y=δq"), ("text", "。下标"), ("math", "k"), ("text", "强调这些矩阵随参考工作点改变。")
    ])
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "本研究对"), ("math", "Aₖ"), ("text", "采用中心差分而不是单侧差分。令"),
        ("math", "eᵢ"), ("text", "为第"), ("math", "i"), ("text", "个标准基向量，"),
        ("math", "hᵢ"), ("text", "为相应扰动步长，则第"), ("math", "i"), ("text", "列由下式取得：")
    ])
    anchor = add_formula(doc, anchor, formula_ppr,
        "Aₖ(:,i)=[f(xᵣ,ₖ+hᵢeᵢ,FCT,ₖ)−f(xᵣ,ₖ−hᵢeᵢ,FCT,ₖ)]/(2hᵢ)，i=1,…,12")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "这样正、负扰动的一阶截断误差相互抵消。程序中对前三个平移和后三个姿态状态分量均取"),
        ("math", "hᵢ=10⁻⁶"), ("text", "，对六个速度分量取"), ("math", "hᵢ=10⁻⁵"),
        ("text", "；并以半步长重新计算"), ("math", "Aₖ"), ("text", "，用两次结果的Frobenius范数相对差检查数值收敛。"
            "输入矩阵则利用驱动力在线性加速度方程中显式出现这一结构直接获得：")
    ])
    anchor = add_formula(doc, anchor, formula_ppr,
        "Bₖ=[0₆ˣ⁶；H(qᵣ,ₖ)⁻¹Jᵥ(qᵣ,ₖ)ᵀ]")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "也就是说，支链力不直接改变位姿导数，故上半块为零；它先经"),
        ("math", "Jᵥᵀ"), ("text", "变为广义力，再经"), ("math", "H⁻¹"), ("text", "变为广义加速度，故下半块如上。"
            "程序还以1 N的正、负输入扰动计算数值"), ("math", "Bₖ"), ("text", "，仅用于核对该解析力映射的相对误差，"
            "并不以数值差分结果替代设计用的"), ("math", "Bₖ"), ("text", "。每个工作点同时要求"),
        ("math", "rcond(H)>10⁻¹⁰"), ("text", "和"), ("math", "rcond(Jᵥᵀ)>10⁻¹⁰"),
        ("text", "；任一条件不满足即拒绝该采样点，避免在病态模型上求取不可信增益。")
    ])
    anchor = add_body(doc, anchor, normal_style, body(
        "至此，非线性轨迹跟踪问题在时刻tₖ被转换为局部线性误差系统。LQR的任务是在该局部系统上，"
        "在“误差衰减要快”和“反馈修正力不能过大”之间给出可重复的定量折中，而不是依靠六个独立PID通道反复试调。"
    ))
    anchor = add_body(doc, anchor, normal_style, body(
        "对第k个局部模型，定义无限时域二次型性能指标，并以状态反馈形式施加修正力："
    ))
    anchor = add_formula(doc, anchor, formula_ppr,
        "J_LQR,ₖ=∫₀^∞(δxᵀQδx+δFᵀRδF)dt，δF_LQR=−Kₖδx", "(4-5)")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "式中，"), ("math", "J_LQR,ₖ"), ("text", "为从当前时刻累积到无穷远的综合代价；第一项"),
        ("math", "δxᵀQδx"), ("text", "衡量状态偏差，第二项"), ("math", "δFᵀRδF"),
        ("text", "衡量修正力的使用量。"), ("math", "Q=Qᵀ⪰0"), ("text", "为半正定状态权重，"),
        ("math", "R=Rᵀ≻0"), ("text", "为正定输入权重；前者越大，控制器越重视压低相应误差，"
            "后者越大，控制器越保守、越不愿使用反馈力。控制律中的负号使反馈与误差方向相反；"),
        ("math", "Kₖ∈R⁶ˣ¹²"), ("text", "把12维状态误差映射为6维支链力修正。性能指标前是否写"),
        ("math", "1/2"), ("text", "只会整体缩放代价，不改变最优反馈增益，故本章采用式（4-5）的写法。")
    ])
    anchor = add_body(doc, anchor, normal_style, body(
        "下面说明式（4-5）如何得到增益，而非直接给出结论。设局部最优代价函数为"
        "V(δx)=δxᵀPₖδx，其中Pₖ=Pₖᵀ⪰0。将局部状态方程代入最优性条件，"
        "并令对δF的一阶导数为零，可得到连续代数Riccati方程及反馈增益："
    ))
    anchor = add_formula(doc, anchor, formula_ppr,
        "AₖᵀPₖ+PₖAₖ−PₖBₖR⁻¹BₖᵀPₖ+Q=0，Kₖ=R⁻¹BₖᵀPₖ")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "其中，"), ("math", "Pₖ∈R¹²ˣ¹²"), ("text", "是代价函数的系数矩阵，反映从当前误差状态出发的最小未来代价。"
            "求解Riccati方程时选择使"), ("math", "Aₖ−BₖKₖ"), ("text", "所有特征值实部均为负的稳定解；"
            "只有在"), ("math", "(Aₖ,Bₖ)"), ("text", "可稳定且"), ("math", "(Q¹ᐟ²,Aₖ)"),
        ("text", "可检测时，该稳定解才具有上述意义。因而，“求得K”之后还必须检查闭环极点，"
            "不能把数值求解器返回矩阵直接视为有效控制器。")
    ])
    anchor = add_body(doc, anchor, normal_style, body(
        "权重不按矩阵元素随意试取，而按可接受误差尺度进行归一化。本章先以位姿尺度构造LQR的状态权重，"
        "再在下一小节加入积分状态权重。用于位姿部分的实际设置为"
    ))
    anchor = add_formula(doc, anchor, formula_ppr,
        "sₚ=[0.03,0.03,0.03,π/180,π/180,π/180]ᵀ，Qₓ=Cᵀdiag(sₚ⁻²)C+10⁻⁹I₁₂，R=diag((1/5000)²·1₆)")
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "这里，"), ("math", "sₚ"), ("text", "的前三个0.03表示30 mm的平移尺度，后三个"),
        ("math", "π/180"), ("text", "表示1°的姿态尺度；因此每个平移位姿分量的权重约为1111.11，"
            "每个姿态分量的权重约为3282.81。"), ("math", "C"), ("text", "只选择位姿输出，"
            "所以速度不在该状态代价中直接赋大权重，而是通过系统动力学与闭环反馈被间接抑制；"),
        ("math", "10⁻⁹I₁₂"), ("text", "是很小的数值正则项，用于避免纯位姿选择导致的严格半正定数值问题。"
            "输入尺度5000 N给出"), ("math", "R=4×10⁻⁸I₆"), ("text", "，它表达的是设计时对反馈力使用量的相对尺度，"
            "并不把5000 N当作反馈力允许上限。运行实现中反馈修正力另有限制为120 N，"
            "故若发生限幅，实际闭环将不再等同于未受限的LQR线性理论。")
    ])
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "上述"), ("math", "Qₓ"), ("text", "和"), ("math", "R"), ("text", "在每个参考工作点保持同一归一化规则；"
            "随轨迹改变的是"), ("math", "Aₖ、Bₖ、Pₖ"), ("text", "以及最终的"), ("math", "Kₖ"), ("text", "。"
            "离线构建调度表时，对完整参考轨迹每0.01 s重复“工作点—线性化—Riccati求解—稳定性检查”流程。"
            "如果步长收敛、质量矩阵条件数、Jacobian条件数或闭环极点检查中任一项不合格，"
            "该工作点的控制器即不被接受；运行阶段只读取通过检查的预计算增益，不在线重线性化，也不在线求解Riccati方程。")
    ])
    anchor = add_body(doc, anchor, normal_style, [
        ("text", "需要区分的是，配置中的"), ("math", "5×10⁻³"), ("text", "速度尺度用于Luenberger观测器的对偶LQR设计，"
            "不是上式"), ("math", "Qₓ"), ("text", "的速度权重；而积分尺度0.2、积分泄漏频率2 Hz和抗饱和时间常数0.02 s，"
            "只在引入积分状态后生效。单纯LQR只能使局部瞬态误差收敛；面对恒定负载偏差或未建模常值扰动时，"
            "仍可能留下稳态偏差。正因如此，下一小节在不改变上述前馈—局部反馈分工的前提下，"
            "将位姿误差积分为新增状态，并由LQR自然增广为LQI。")
    ])

    doc.save(OUT)
    print(OUT)


if __name__ == "__main__":
    write_revision()
