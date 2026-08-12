# Safety-Critical CLF-CBF-QP Tracking Control

## 为什么不继续把 LQI 调参作为核心创新
现有 LQI 和计算力矩控制属于固定反馈律加前馈补偿路线，主要创新空间集中在参数整定和线性化模型质量。SC-QP 将控制层改为在线约束优化，在每个控制周期显式协调跟踪收敛、驱动力能力、支腿运动边界、奇异性裕度和碰撞裕度。

## 基本思想
控制器先生成名义力 `F_nom`，可来自 IHSID、计算力矩或简单 PD；随后求解小规模 QP 得到最终六支链力 `F_cmd`。QP 目标是在接近名义力和限制力变化率的同时，使用 CLF 约束推动误差下降，并用 CBF/barrier-like 约束保护安全边界。

## CLF 约束
定义 `e=q-q_ref`、`ed=qd-qd_ref`、`z=[e;ed]`，Lyapunov 函数为 `V=z'Pz`。当前实现采用等效加速度形式 `qdd_des=qdd_ref-Kd*ed-Kp*e`，并把 `ed'*(qdd(F)-qdd_des)<=-c_clf*V+delta_clf` 写成关于支链力的线性不等式。

## CBF 约束设计
CBF 层覆盖驱动力上下限、驱动力变化率、支腿长度、支腿速度、支腿加速度、归一化 Jacobian 最小奇异值和碰撞距离裕度。难以精确输入仿射化的奇异性/碰撞项先作为 barrier-like 监督器，输出风险裕度并在完整 CBF 模式下加入高惩罚松弛约束。

## QP 目标函数
目标函数为 `||F_cmd-F_nom||_W^2 + rho_clf*delta_clf^2 + rho_cbf*||delta_cbf||^2 + rho_rate*||F_cmd-F_prev||^2`。其中 CBF 松弛权重大于 CLF 松弛权重，使安全裕度优先于跟踪性能。

## 不可行处理策略
若 `quadprog` 不可用或求解失败，控制器进入 fallback：缩放名义力、限制力变化率、强制驱动力限幅，并记录 `status`、`exitflag`、`usedFallback` 和 `safetyBrakeCount`，保证仿真不中断。

## 与 LQI 和计算力矩控制的区别
LQI/计算力矩输出固定反馈或前馈力；SC-QP 输出的是满足约束协调后的优化力。二者可作为 `F_nom` 或 baseline，但不再决定最终控制律。

## 消融实验设计
表格比较 Nominal only、Nominal + CLF、Nominal + force/length CBF、Nominal + singularity CBF、Nominal + collision CBF、Full CLF-CBF-QP 六类组合。

## 鲁棒性实验设计
扰动场景包括负载质量变化、质心偏移、执行器增益误差、力滞后增加、编码器偏置、IMU 姿态噪声和组合扰动。评价跟踪误差、约束违反次数、最小 CBF 裕度和完成状态。

## 可写入硕士论文的创新点表述
本文提出一种面向 Stewart 并联平台的安全关键 CLF-CBF-QP 轨迹跟踪控制方法，将传统固定反馈控制转化为显式安全约束的在线优化控制，在控制层统一处理轨迹收敛、支链力限幅、执行器变化率、奇异性和碰撞安全裕度。

## 推荐图表清单
建议纳入控制结构图、单步 QP 几何图、位姿/姿态误差对比、六支链力与力变化率、支腿约束验证、Lyapunov 函数、CBF 裕度、active constraints、消融柱状图、鲁棒性热力图、奇异值/碰撞距离对比和 QP 求解时间统计。
