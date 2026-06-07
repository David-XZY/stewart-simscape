# Stewart 两阶段隐式 Hermite-Simpson 轨迹优化

`opt_minimal/run_01_hs_dynamic_opt.m` 是当前默认入口。默认主链路使用 CasADi MX、IPOPT 和 MA27，为 6-UCU Stewart 平台生成携带圆柱体接近并送入固定长方体下方的两阶段轨迹。Simscape 不进入 NLP，只在求解后通过离线导出和后验检查使用。

当前工程自带 CasADi 3.7.2 MATLAB 包，并可发现 FATROP 插件；但主优化链路仍默认使用 IPOPT/MA27。FATROP 只通过 `run_05_compare_IHSID_FATROP.m` 做 IHSID manual 接入自检，不作为默认求解器。

## 当前默认流程

1. `buildOptModelCustom.m` 构建 Stewart 几何、执行器硬约束、合成刚体动力学和目标函数权重。
2. `buildCylinderBoxTransferScene.m` 构建圆柱体-长方体场景、两阶段时间网格、`q0`、`qWaypoint` 和 `qGoal`。
3. `buildInitialGuessTwoPhaseHSImplicit.m` / `buildInitialGuessTwoPhaseIHSImplicit.m` 生成两阶段隐式初值，并保留第一阶段标称轨迹。
4. `buildCasadiImplicitHSNLP.m` 或 `buildCasadiImplicitIHSNLP.m` 构建隐式 HS/IHS NLP。
5. 求解后重建轨迹，运行 dense 后验验证，输出 MAT、summary、console log、PNG 和 MP4。

## 场景参数

当前默认间隙设置为：

```matlab
scene.collision.safeDistance = 0.010;  % 10 mm
scene.collision.finalGap = 0.005;      % 5 mm
scene.collision.stage1ConstraintDistance = scene.collision.safeDistance;
scene.collision.waypointGap = scene.collision.safeDistance;
```

第一阶段有限圆柱体和罩体/裙板的分离证书使用 `safeDistance`。第二阶段从 10 mm 路径点沿长方体局部 x 轴送入到 5 mm 最终间隙，姿态保持与长方体一致。

## IHSID 与 FATROP manual 自检

本轮新增的 FATROP manual 链路只服务于公平接入验证，不改变物理模型、碰撞模型、目标函数权重、初值生成逻辑或后验阈值。

manual 阶段变量采用：

```text
Y_k = [X_k; A_k; F_k], nx = 24
U_k = [Xmid_k; Amid_k; Fmid_k; collision certificates; Yright_k]
G_k = Y_{k+1} - Yright_k
```

前 `N` 个区间使用 96 维 `U_k`。终端阶段不再附加完整 96 维 `U_N`，只保留终端节点碰撞分离证书所需的最小占位变量；20x10 默认网格下 `nuVec=[96*ones(30,1);16]`。这样既避免旧的完整终端 `U_N`，也不删除 standard IHSID 里的终端碰撞约束。

相关文件：

- `packIHSFatropManualDecision.m` / `unpackIHSFatropManualDecision.m`：standard IHSID 与 manual 阶段顺序互转。
- `buildCasadiImplicitIHSFatropManualNLP.m`：构建 IHSID manual NLP，可用 IPOPT 或 FATROP 后端。
- `checkFatropManualStructure.m`：求解前检查 `N/nx/nu/ng/equalityMask/numZ/numG` 和初值 gap。
- `checkIHSManualEquivalence.m`：检查 standard 与 manual 在同一初值处的目标函数、等式残差、不等式违反量和 gap。
- `probeFatropOptions.m`：逐项测试 FATROP manual 结构选项和 solver 选项是否被当前 CasADi 接口接受。
- `run_05_compare_IHSID_FATROP.m`：只运行 20x10 的 `IHSID-STANDARD-IPOPT`、`IHSID-MANUAL-IPOPT`、`IHSID-MANUAL-FATROP` 检查。

2026-05-31 的 20x10 检查结果显示：

- `IHSID-STANDARD-IPOPT`：`Solved_To_Acceptable_Level`，工程后验通过。
- `IHSID-MANUAL-IPOPT`：`Solved_To_Acceptable_Level`，工程后验通过；初值处 `objectiveStdAtX0 == objectiveManualAtX0`，manual gap 为 0。
- `IHSID-MANUAL-FATROP`：未进入正式求解，因为当前 CasADi FATROP MATLAB 接口不接受 `hessian_approximation=limited-memory`；因此本轮 FATROP 对比被标记为无效，而不是判定 FATROP 不适合该系统。

结果目录示例：

```text
opt_minimal/results/compare_IHSID_FATROP_manual_20260531_173230/
```

该目录包含 `comparison_summary.csv`、`comparison_report.txt`、`comparison_results.mat`、`README_IHSID_manual_FATROP_fixed.md`、`fatrop_solver_log.txt`、`manual_structure_check.txt` 和 `manual_equivalence_check.txt`。

## 约束与目标函数

NLP 等式约束包括 Hermite-Simpson 状态积分一致性、节点/中点隐式动力学平衡、路径点位姿约束、第二阶段送入直线/姿态/单向运动约束，以及碰撞分离证书单位法向约束。

NLP 不等式约束包括支链长度、速度、加速度硬约束，碰撞安全距离约束，以及第二阶段单向送入速度。驱动力只保留硬上下界：

```matlab
model.actuator.forceMin = -2000 * ones(6, 1);
model.actuator.forceMax =  2000 * ones(6, 1);
```

当前目标函数为五项加权和：

```matlab
J = 0.10 * J_nominalStage1 ...
  + 0.10 * J_forceRate ...
  + 0.40 * J_legAccel ...
  + 0.40 * J_singularity;
```

`summary` 会输出四项加权代价与占比，同时保留 `max|F|` 和 `forcePassed` 作为硬约束诊断。

## 如何运行

默认优化：

```matlab
run('opt_minimal/run_01_hs_dynamic_opt.m')
```

IHSID manual FATROP 接入自检：

```matlab
result = run_05_compare_IHSID_FATROP();
```

IHSID 主线 Hessian/网格对比实验：

```matlab
result = run_08_compare_IHSID_hessian_grid( ...
    'gridList', [10 5; 20 10], ...
    'hessianModes', {'exact', 'limited-memory'});
```

该入口只运行 standard IHSID 主线，用同一个初值、目标和约束结构分别对比 IPOPT exact Hessian 与 limited-memory Hessian，并输出 `comparison_summary.csv`、`comparison_report.txt`、`comparison_results.mat` 和 `README_IHSID_hessian_grid.md`。快速检查程序编排时可加 `'dryRun', true`；小规模真实 smoke 可用 `[2 1]` 或 `[2 1; 4 2]` 网格配合较小 `maxIter`。

本轮只允许自动运行 20x10，不运行 40x20 或 60x30。

中期报告轨迹优化补图：

```matlab
result = writeMidtermReportFigures(comparisonMatFile, outputDir);
```

该入口使用已有 `comparison_results.mat` 生成图4方法比较图和图7工程约束验证图，不重新运行轨迹优化，同时保留可见的 MATLAB figure 窗口并输出 300 dpi PNG 和矢量 PDF。正文方法只展示 CHSED、CHSID、IHSID；CHSED 求解结果数值来自《表4.docx》表1，其不等式约束数量按当前 20+10 离散结构计算，CHSID/IHSID 与工程约束数据来自当前代码结果。

## 2026-05-31 exact Hessian 诊断结果

在 `run_05_compare_IHSID_FATROP.m` 中，IPOPT 对比组已经切换为 exact Hessian；FATROP 不再因为 limited-memory option 不支持而跳过，仍使用 20x10 和 `maxIter=30` smoke。

最新结果目录：

```text
opt_minimal/results/compare_IHSID_FATROP_manual_20260531_175946/
```

关键结论：

- `IHSID-STANDARD-IPOPT` 与 `IHSID-MANUAL-IPOPT` 都返回 `Solve_Succeeded`，但 dense 工程后验未通过，主要表现为 `maxDenseDynResidual` 过大。
- `IHSID-MANUAL-FATROP` 已进入真实求解，30 次 smoke 每次都出现 `degenerate Jacobian`，`degenerateJacobianCount=30`，`n_eval_hess_l=30`。
- 因此 exact Hessian 诊断下，当前 FATROP 问题不再只是 limited-memory 选项不支持，而是持续退化 Jacobian 与工程后验不通过。

## 目录说明

- 顶层 `.m` 文件：当前默认入口和 IHSID/FATROP 接入检查所需的主文件。
- `solver_comparison/`：历史求解器对比脚本目录，不属于默认入口。
- `unused/`：默认入口不再调用的旧版、压缩版、预对准或对比实验文件。
- `results/`：优化和对比运行产生的 MAT、日志、图像、动画与报告。
## FATROP-native HS 诊断路线

`run_06_compare_HS_FATROP_NATIVE.m` 是面向 FATROP 的独立诊断路线，不替代默认
`run_01_hs_dynamic_opt.m`。该路线使用 stage 状态 `X_k=[q_k;qd_k]`、区间控制
`U_k=[F_k;F_c;F_{k+1}]`、固定方向碰撞 gap 约束和 FATROP manual 结构，不再引入自由
collision-certificate 变量。终端 stage 只保留 1 维固定 dummy 控制，因为当前 CasADi
3.7.2 的 FATROP manual MATLAB 接口不接受 `nu(end)=0`。

原隐式 HS defect 含有非线性的 `f(X_{k+1})` 项，不符合 FATROP 期望的
`X_{k+1}-Phi(X_k,U_k)` 依赖形式。FATROP-native 路线因此改用带三点力插值的显式
shooting map，同时继续沿用 dense 工程后验检查。

2026-05-31 的约束分层诊断显示，`dynamics`、`actuator`、`actuator_collision` 和
`no_insertion` 分层均不触发 `degenerate Jacobian`；退化集中在 stage2 插入段的硬几何
等式。默认 `full` profile 因此把插入段直线/姿态约束改为强二次罚，并保留单向插入
不等式；`full_hard` profile 保留旧硬等式，用于复现和对照。可用
`run_07_diagnose_FATROP_NATIVE_profiles.m` 做小网格约束分层排查。

后续诊断还表明，`full` 中剩余的最大不等式违反主要来自固定法向侧壁 gap。该 gap
会把有限裙板近似成无限侧向平面，导致固定初始节点也可能被判成不可行。为定位该问题，
`run_07_diagnose_FATROP_NATIVE_profiles.m` 的输出表新增了
`maxPathViolation`、`maxRoofViolation`、`maxLeftSideViolation`、
`maxRightSideViolation`、`maxInsertionMonotonicViolation` 等分组列。当前保留
`full_soft_side`、`full_smooth_collision`、`full_stage2_side` 和 `full_masked_side`
作为失败分支/对照分支；它们在小网格诊断中没有改善 FATROP 收敛质量，因此不作为默认
推荐配置。
