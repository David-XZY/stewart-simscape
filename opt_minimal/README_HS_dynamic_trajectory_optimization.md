# Stewart 两阶段隐式 Hermite-Simpson 轨迹优化

`opt_minimal/run_01_hs_dynamic_opt.m` 是当前默认入口。默认链路使用 CasADi MX、IPOPT 和 MA27，为 6-UCU Stewart 平台生成携带运动圆柱体接近并送入固定长方体下方的两阶段轨迹。Simscape 不进入 NLP，只在求解后通过 `exportTrajectoryToSimscape.m` 导出离线验证参考量。

## 当前默认流程

1. `buildOptModelCustom.m` 构建 Stewart 几何、执行器硬约束、合成刚体动力学和目标函数权重。
2. `buildCylinderBoxTransferScene.m` 构建圆柱体-长方体场景、两阶段时间网格、`q0`、`qWaypoint` 和 `qGoal`。
3. `buildInitialGuessTwoPhaseHSImplicit.m` 生成两阶段五次时间律初值，并把第一阶段初值保存为标称参考轨迹。
4. `buildCasadiImplicitHSNLP.m` 构建隐式 HS NLP 并调用 IPOPT/MA27 求解。
5. 求解后重建轨迹，运行 dense 后验验证，保存 MAT、summary、console log、PNG 和 MP4。

## 场景参数

当前默认间隙设置为：

```matlab
scene.collision.safeDistance = 0.010;  % 10 mm
scene.collision.finalGap = 0.005;      % 5 mm
scene.collision.stage1ConstraintDistance = scene.collision.safeDistance;
scene.collision.waypointGap = scene.collision.safeDistance;
```

第一阶段有限圆柱体-有向长方体凸体分离约束使用 `safeDistance`。第二阶段从 10 mm 途径点沿长方体局部 x 轴送入到 5 mm 最终间隙，姿态保持与长方体一致。

## 隐式 HS 决策变量

默认链路使用隐式变量布局，首尾端点状态由 `scene` 固定，不进入决策变量：

```matlab
z = [
    Xinternal(:);
    Anode(:);
    Amid(:);
    Fnode(:);
    Fmid(:);
    separator(:)
]
```

- `Xinternal`：不含首尾端点的节点状态 `X=[q;qd]`。
- `Anode` / `Amid`：节点和中点平台广义加速度 `qdd`。
- `Fnode` / `Fmid`：节点和中点六条支链轴向驱动力。
- `separator`：第一阶段节点和中点的凸体分离证书变量 `[n;eta;zeta;rho]`。

## 约束与目标函数

NLP 等式约束包括 Hermite-Simpson 状态积分一致性、节点/中点隐式动力学平衡、途径点位姿约束、第二阶段送入直线/姿态/单向运动约束，以及第一阶段分离证书单位法向约束。

NLP 不等式约束包括支链长度、速度、加速度硬约束，第一阶段分离证书安全间隙，以及第二阶段单向送入速度。驱动力仅保留硬上下界：

```matlab
model.actuator.forceMin = -2000 * ones(6, 1);
model.actuator.forceMax =  2000 * ones(6, 1);
```

当前目标函数为五项加权和：

```matlab
J = 0.10 * J_nominalStage1 ...
  + 0.10 * J_forceRate ...
  + 0.40 * J_legAccel ...
  + 0.40 * J_singularity ...
  + 0.10 * J_power;
```

- `J_nominalStage1`：仅第一阶段，约束圆柱中心和姿态偏离初值标称轨迹。
- `J_forceRate`：全阶段，惩罚相邻节点/中点驱动力变化率。
- `J_legAccel`：全阶段，保留支链加速度平滑项。
- `J_singularity`：全阶段，保留奇异性软惩罚。
- `J_power`：全阶段，使用支链机械功率 `P_i=F_i*Ld_i` 的平方归一化代价，默认 `powerScale=1000 W`。

已移除驱动力幅值平方代价和接近驱动力上限的软惩罚。summary 会输出五项加权代价与占比，同时保留 `max|F|` 和 `forcePassed` 作为硬约束诊断。

## 如何运行

在 MATLAB 中进入 `stewart-simscape` 工程根目录，运行：

```matlab
run('opt_minimal/run_01_hs_dynamic_opt.m')
```

结果保存到：

```text
opt_minimal/results/
```

主要输出文件包括：

- `result_cylinder_box_two_phase_ipopt_ma27_yyyymmdd_HHMMSS.mat`
- `summary_cylinder_box_two_phase_ipopt_ma27_yyyymmdd_HHMMSS.txt`
- `console_cylinder_box_two_phase_ipopt_ma27_yyyymmdd_HHMMSS.txt`
- 轨迹诊断 PNG 和动画 MP4

## 目录说明

- 顶层 `.m` 文件：默认入口当前调用链所需的主文件。
- `solver_comparison/`：历史求解器对比脚本目录，不属于默认入口。
- `unused/`：默认入口不再调用的旧版、压缩版、预对准或对比实验文件。
- `results/`：优化运行产生的 MAT、日志、图片和动画。
