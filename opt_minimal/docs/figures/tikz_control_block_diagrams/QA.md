# TikZ 控制框图 QA

## 绘图模式

- presentation_mode: research
- diagram_family: feedback / convergence loop
- shared_style: `control_block_styles.tex`
- contact_sheet_version: `contact_sheet_v01.png`

## 控制逻辑审查

- math_logic_review: exact
- exact_scope: 控制器拓扑、求和符号、参考输入、反馈信号、前馈信号及各限制器顺序均依据当前 MATLAB/Simulink 构建代码绘制。
- schematic_scope: `G_{F->q}` 与 `G_v(z)` 只表示模型中的平台和理想执行器响应结构，不表示已辨识出的具体传递函数参数。
- Run02: `F_cmd = F_FF + J_v(q_0)^(-T) K_x(s) (q_ref - q_act)`；固定映射在初始位姿 `q_0` 计算。
- Run03: 腿长位置 P 外环生成腿速修正，与 `rLd` 前馈相加；经速度/加速度指令限制后进入腿速 PIDF 内环和伺服加速度限制。
- Run04: 在 Run03 串级控制前增加位姿误差低通、时变 `J_q(q_ref)`、位姿反馈增益和腿长修正限幅。
- Run03/04 顶层: 将参考整形明确拆分为腿速限幅与加速度限制；将 PIDF 输出后的伺服加速度限制及 tracking 抗饱和回送显式绘出。
- Length-Servo 层级: 顶层控制图将其保留为黑箱理想运动执行器，不把内部一阶腿速模型、积分器和移动副混入控制器主图。
- Length-Servo 内部模型: 一阶腿速模型仅用于理想执行器内部，连续形式为 `K/(tau*s+1)`，由 ZOH 离散实现；默认 `K=1`、`tau=0.03 s`、`Ts=0.01 s`。
- 求和节点: 使用公共 `markSumPM` / `markSumPP` 宏统一定位圆内符号，分别表达负反馈求差与前馈叠加。

## 视觉与复杂度审查

| 图 | visual_QA | complexity_review | 结论 |
|---|---|---|---|
| Run02 | pass | keep | 单一位姿反馈力控制语法，主链和前馈清晰。 |
| Run03 | pass | keep | 保持顶层控制器与 Length-Servo 子系统的真实层级，未将执行器内部实现混入主图。 |
| Run04 | pass | keep | 位姿外环与 Run03 顶层串级控制保持一致，反馈线路无穿框。 |

## 验证

- 三个 `.tex` 文件通过 `check_tikz_safety.py`。
- 三个 PDF 均使用 XeLaTeX 编译成功。
- 三个 PNG 均通过 `compile_render.py --visual-check --visual-mode research`。
- 已生成并人工检查 `contact_sheet_v01.png`。
- 本轮逻辑优化最终人工检查版本为 `*_v04.png`。
