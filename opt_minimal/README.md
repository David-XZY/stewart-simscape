# opt_minimal：standard IHSID 轨迹生成主线

本目录只维护一条活动轨迹生成链：**standard IHSID + IPOPT/MA27 + 40x20 网格 + limited-memory Hessian**。  
主入口为 `run_01_ihsid_trajectory.m`，仅当求解器成功且工程后验通过时，才发布 Simscape 参考轨迹。
Simscape 验证入口为 `run_02_simscape_length_control.m`，采用 IHSID 逆动力学力前馈加
笛卡尔位姿动态反馈，最终执行器输入始终为六腿驱动力。
纯长度验证入口为 `run_03_simscape_length_cascade_control.m`，仅使用 `t/q/qd`，
关闭重力并采用位置 P + 速度 PIDF 串级控制。
位姿反馈纯腿长入口为 `run_04_simscape_pose_length_control.m`，在 Run03 基础上
将平台位姿误差转换为腿长参考修正，最终执行器仍为理想腿长输入。

## 快速运行

在 MATLAB 中将当前目录切换到仓库根目录，运行：

```matlab
run('opt_minimal/run_01_ihsid_trajectory.m')
run('opt_minimal/run_02_simscape_length_control.m')
run('opt_minimal/run_03_simscape_length_cascade_control.m')
run('opt_minimal/run_04_simscape_pose_length_control.m')
```

运行期产物写入 `opt_minimal/results/`，该目录除 `.gitkeep` 外均被 Git 忽略。

`run_02` 默认读取标准样例并自动完成仿真。也可指定 `run_01` 发布的结果文件：

```matlab
simscapeTrajectoryFile = 'opt_minimal/results/result_....mat';
run('opt_minimal/run_02_simscape_length_control.m')
```

若希望在 Simulink 中手动点击运行：

```matlab
simscapeRunMode = 'manual';
run('opt_minimal/run_02_simscape_length_control.m')
```

脚本会准备基础工作区变量、配置重力与默认 `15 Hz` 位姿力控制器、设置停止时间并打开模型。
用户可调整 `references`、`K` 等变量后点击运行。关闭模型时不要保存运行期配置。

默认整定参数为：

```matlab
poseForceConfigOverrides = struct( ...
    'bandwidthHz', 15, ...
    'gainScale', 0.45, ...
    'rotationGainScale', 1.4);
run('opt_minimal/run_02_simscape_length_control.m')
```

`run_03` 默认以 `10 ms` 采样周期运行，使用 `pidtune` 在仿真前逐腿整定 `10 Hz`
速度 PIDF 和 `2 Hz` 位置 P，并固定使用位置环 `1.0`、速度环 `0.7` 增益缩放。
它只读取轨迹中的 `refs.t/q/qd`，先由节点 `q/qd` 分段三次 Hermite 重建
`10 ms` 规则参考，再由 IK 和 Jacobian 同步生成 `references.rL/rLd`，
不读取 `refs.L/Fleg` 或 `references.uFF`。手动模式为：

```matlab
lengthCascadeRunMode = 'manual';
run('opt_minimal/run_03_simscape_length_cascade_control.m')
```

如需进行非默认参数实验，可在保留 `10 Hz / 2 Hz` 自动整定带宽的同时覆盖增益缩放：

```matlab
lengthCascadeConfigOverrides = struct( ...
    'positionGainScale', 0.8, ...
    'velocityGainScale', 0.6);
run('opt_minimal/run_03_simscape_length_cascade_control.m')
```

## 输出与验收

主入口固定生成 40x20 IHSID NLP，并依次执行：

1. CasADi/IPOPT/MA27 环境预检。
2. IHSID 初值、NLP 构建、求解和数值轨迹重建。
3. dense 后验、第二阶段连续间隙和工程约束验收。
4. 仅在 `solverSuccess=1` 且 `engineeringPassed=1` 时生成：
   - `refs.q/qd/Fleg/L` 原始节点数组；
   - `references.r=q-q0`、`references.rL=L-L0` 和 `references.uFF=Fleg`，均为节点时间上的 `N×6` `timeseries`。

标准可复用样例保存在 `examples/ihsid_40x20_limited_memory/`。

## 目录职责

- `core/`：场景、动力学、碰撞、运动学、离散配置和 IPOPT 公共配置。
- `ihsid/`：IHSID 初值、NLP、决策变量打包/解包和轨迹重建。
- `validation/`：dense 后验、目标函数分解和工程验收。
- `integration/`：Simscape 相对参考导出、统一仿真准备、参数映射、重力配置、线性化整定和结果验收。
- `tools/`：维护中的绘图、动画和装配图工具。
- `tests/`：仅覆盖活动 IHSID 主线、目录结构和 Simscape 契约。
- `unused/`：保留的历史比较、FATROP、旧 HS 和报告代码；活动代码不得调用。

全部源码位置与状态见 `FILE_CATALOG.md`。

## 测试

```matlab
run('opt_minimal/tests/run_all_active_tests.m')
```

`test_04_simscape_model_contract` 加载并检查 `matlab/stewart_platform_model.slx` 的输入和日志契约。

## Simscape 力输入位姿轨迹跟踪

`run_02_simscape_length_control.m` 保留原入口名称，但控制结构已替换为
`Reference-Tracking-X`。`references.uFF=refs.Fleg` 提供逆动力学与重力补偿，
实际位姿 `Xr` 与 `references.r` 的误差经动态控制器 `K` 转换为六腿反馈修正力：

```text
u = uFF + uFeedback = Fleg + K * poseError
```

线性化时使用现有 `Relative Motion Sensor` 的 `Xr`，并通过当前位姿雅可比的
`Jv^-T` 将笛卡尔控制量映射为支链驱动力。控制器不读取腿长误差作为反馈量。
位姿参考不再直接线性连接 `0.125 s` 节点，而是使用节点 `q/qd` 分段三次 Hermite
重建为 `10 ms` 规则网格；IHSID 前馈力因没有节点导数，仍按时间线性插值到同一网格。

脚本将优化几何、50 kg 移动平台、100 kg 圆柱负载、合成质心、惯量和重力映射到
Simscape；腿刚度与阻尼按优化假设设为零，每段杆件保留 `1e-3 kg` 数值正则质量。
圆柱负载按轨迹优化场景中的真实几何横卧于平台：轴线沿平台 `X` 轴，长度 `1.20 m`、
半径 `0.15 m`、质心位于平台原点上方 `0.15 m`。Simscape Solid 的局部 `Z` 轴经
刚性变换对齐到平台 `X` 轴，惯量同步转换到 Solid 局部坐标，避免仅旋转外观而改变动力学。
该实现直接复用现有 SLX 的位姿参考、相对位姿传感器、`Reference-Tracking-X` 和顶层
力求和链，不需要继续修改 SLX。线性化整定时关闭重力并临时清零 `uFF`；正式仿真恢复
IHSID 前馈并开启重力。完整跟踪硬验收要求：峰值腿长误差不超过 `5 mm`、峰值平移误差
不超过 `10 mm`、峰值转角误差不超过 `1 deg`，腿长/腿速/腿加速度满足优化模型约束，
总驱动力保持在 `±2000 N`。腿运动指标与 run03 一样在 `10 ms` 规则网格上计算。

修正横卧圆柱实体坐标后，默认 run02 相对 run03 默认基准的位姿综合分约为
`0.0226`，平移峰值比约为 `0.0284`，转角峰值比约为 `0.0330`；
最大总力约 `1097.5 N`，最大腿加速度约 `0.1680 m/s²`，全部硬验收通过。

运行 `exportRun02Run03MeetingFigures` 会导出 Run02/Run03 总览、完整轨迹对比和纹波诊断图。
纹波诊断自动选择 Run03 高频残差最明显的支链与局部时间窗，并标注参考节点频率、
速度内环带宽和检测到的主纹波频率。改用 Hermite 重建后，Run02/Run03 的 `8 Hz`
频带误差幅值分别由 `0.0485/0.0459 mm` 降至 `0.00045/0.00137 mm`，证明原波纹主要
来自节点线性参考。Run03 平移和转角峰值约增加 `3%/2%`，但仍远低于硬阈值；
两种控制的腿加速度峰值均明显下降。

Simscape 尚未包含真实腿刚度、阻尼、传感器噪声、延迟、执行器饱和和障碍物。
因此被控对象验证不能替代现有 IHSID 碰撞与工程后验。

## Simscape 纯长度串级控制

`controller.type=8` 选择 `Length-Cascade`，`stewart.actuators.type=5` 选择
`Length-Servo`。长度执行器使用 Motion Provided by Input 的 Prismatic Joint，
由一阶速度对象积分生成带导数的运动轮廓；速度 PIDF 输出同时应用速度饱和、
tracking anti-windup 和加速度斜率限制。长度模式关闭重力，顶层前馈为零，
不输入、不读取、不验收驱动力。为避免改动公共总线尺寸，长度执行器仅在
`type=5` 分支中复用旧 `Taum` 总线槽传递关节速度；该信号在长度模式中不是力。

标准全轨迹硬验收要求信号有限、自动整定闭环稳定、腿长/腿速/腿加速度满足约束，
且峰值腿长误差不超过 `5 mm`、平移误差不超过 `10 mm`、转角误差不超过 `1 deg`。
终点腿长误差 `0.1 mm` 仅作为诊断目标，不决定硬通过。

## Simscape 位姿反馈纯腿长输入控制

`run_04` 保留 Run03 的理想腿长执行器、位置 P 与速度 PIDF，只在长度误差计算前加入
平台位姿反馈：

```text
filteredPoseError = lowpass(r - Xr, 2 Hz)
deltaLpose = Jq(qReference) * 1.0 * filteredPoseError
effectiveLengthError = rL + deltaLpose - dLm
```

默认腿长修正限幅为 `2 mm`，重力关闭。参考雅可比在 `10 ms` Hermite 参考网格上
逐点计算，位姿误差先经过 `2 Hz` 一阶离散低通，再映射为腿长修正。该结构用于分离
“增加平台位姿反馈”本身的效果，
并与 Run02 的力输入位姿反馈及 Run03 的纯腿长输入进行公平对比。运行
`exportRun02Run03Run04Comparison` 可导出三种控制结构的共同误差时序和位姿指标图。

采用时变参考雅可比和低通后，默认完整轨迹的平移峰值、转角峰值和腿长峰值误差约为
`0.486 mm / 0.0606 deg / 0.896 mm`。相对旧固定初始雅可比 Run04，三项峰值分别降低
约 `25.5% / 27.1% / 35.9%`，`5 s` 后等效位姿高频纹波 RMS 从约 `0.0482 mm`
降至 `0.00937 mm`，降低约 `80.6%`。
