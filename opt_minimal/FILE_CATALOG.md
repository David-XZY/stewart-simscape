# opt_minimal 文件目录

状态说明：`活动` 表示当前主线直接或间接使用；`测试` 表示活动契约测试；`工具` 表示主线可调用的维护工具；`归档` 表示仅供历史追溯，活动代码不得调用。

## 根目录

| 文件 | 用途 | 状态 | 调用方/入口 | 处理原因 |
|---|---|---|---|---|
| `run_01_ihsid_trajectory.m` | standard IHSID 40x20 limited-memory 唯一主入口 | 活动 | 用户直接运行 | 由旧主入口改名并收敛为 IHSID |
| `run_02_simscape_length_control.m` | 自动运行或手动准备 IHSID 轨迹的 Simscape 稳定跟踪 | 活动 | 用户直接运行 | 自动/手动控制接入与完整硬验收入口 |
| `README.md` | 运行、验收和 Simscape 接入说明 | 活动 | 维护者 | 替代旧 HS 总说明 |
| `FILE_CATALOG.md` | 逐文件分类目录 | 活动 | 维护者 | 便于交接与后续更新 |

## 活动源码

| 文件 | 用途 | 状态 | 主要调用方 | 处理原因 |
|---|---|---|---|---|
| `core/buildCylinderBoxCollisionModel.m` | 构建碰撞模型 | 活动 | 场景构建 | IHSID 场景依赖 |
| `core/buildCylinderBoxTransferScene.m` | 构建两阶段搬运场景 | 活动 | 主入口、测试 | IHSID 场景入口 |
| `core/buildOptModelCustom.m` | 构建 Stewart 优化模型参数 | 活动 | 主入口、测试 | IHSID 模型入口 |
| `core/buildTwoPhaseIHSDiscretization.m` | 构建默认 40x20 或测试网格 | 活动 | 主入口、测试 | 提取并统一离散规则 |
| `core/computeAnchorPoints66.m` | 计算铰点位置 | 活动 | 运动学/绘图 | 公共几何依赖 |
| `core/computeCompositeRequiredWrench.m` | 计算合成刚体需求力旋量 | 活动 | 动力学 | 公共动力学依赖 |
| `core/computeCylinderWaypointAndGoal.m` | 计算途径点和目标位姿 | 活动 | 场景构建 | 两阶段场景依赖 |
| `core/computeHSMidpointStateCompressed.m` | 计算基础初值中点状态 | 活动 | IHSID 基础初值 | 保留配点公共公式 |
| `core/computeLegKinematics.m` | 计算支链运动学 | 活动 | 路径约束/验证 | 公共运动学依赖 |
| `core/evaluateCylinderBoxClearance.m` | 计算圆柱与箱体间隙 | 活动 | 验证/场景 | 工程验收依赖 |
| `core/evaluateCylinderBoxDistanceNumeric.m` | 数值碰撞距离 | 活动 | 路径约束 | NLP/验证依赖 |
| `core/evaluatePathConstraintsAtPoint.m` | 统一点评价路径约束 | 活动 | NLP 数值重建/验证 | 主线约束入口 |
| `core/inverseDynamicsCompositeRigidBody.m` | 数值逆动力学 | 活动 | 初值生成 | IHSID 初值依赖 |
| `core/makeCommonIpoptOptions.m` | 固定 IPOPT/MA27/limited-memory 配置 | 活动 | IHSID NLP | 主线求解器配置 |
| `core/rpy2rotmZYX.m` | ZYX 姿态转换 | 活动 | 场景/动力学/验证 | 公共姿态依赖 |
| `core/rpyRateMapZYX.m` | ZYX 角速度映射 | 活动 | 动力学/验证 | 公共姿态依赖 |
| `core/setupCasadiIpoptMa27.m` | CasADi/IPOPT/MA27 预检 | 活动 | 主入口、测试 | 删除 FATROP 预检分支 |
| `core/sgpIK.m` | Stewart 逆运动学 | 活动 | 运动学/绘图 | 公共运动学依赖 |
| `core/sgpJacobian.m` | Stewart 雅可比 | 活动 | 动力学/目标/验证 | 公共运动学依赖 |
| `core/stateDynamicsCompositeRigidBody.m` | 合成刚体状态动力学 | 活动 | 验证 | 主线动力学依赖 |
| `ihsid/buildCasadiImplicitIHSNLP.m` | 构建 standard IHSID NLP | 活动 | 主入口、测试 | 删除通用 FATROP 工厂依赖 |
| `ihsid/buildInitialGuessTwoPhaseHSImplicit.m` | 生成 IHSID 共用基础初值 | 活动 | IHSID 初值器 | 移回活动目录并停止旧决策向量打包 |
| `ihsid/buildInitialGuessTwoPhaseIHSImplicit.m` | 生成 IHSID 独立中点初值 | 活动 | 主入口、测试 | IHSID 专用初值入口 |
| `ihsid/evaluateImplicitIHSTrajectoryNumeric.m` | 数值重建 IHSID 解 | 活动 | 主入口 | IHSID 后处理入口 |
| `ihsid/packIHSDecisionImplicit.m` | 打包 IHSID 决策变量 | 活动 | 初值器、测试 | IHSID 契约 |
| `ihsid/rebuildComparisonTrajectory.m` | 转换为绘图/验证轨迹结构 | 活动 | 主入口 | 保留现有稳定轨迹结构 |
| `ihsid/unpackIHSDecisionImplicit.m` | 解包 IHSID 决策变量 | 活动 | 数值重建、测试 | IHSID 契约 |
| `validation/computeObjectiveBreakdownImplicit.m` | 分解四项目标函数 | 活动 | 主入口 | IHSID 结果解释 |
| `validation/validateInsertionPhaseContinuousClearance.m` | 验证插入段连续间隙 | 活动 | dense 后验 | 工程验收依赖 |
| `validation/validateTrajectoryDenseIHSID.m` | IHSID 专用 dense 工程验收 | 活动 | 主入口 | 替代多方法验证分支 |
| `validation/validateTrajectoryDenseImplicit.m` | 密集采样后验 | 活动 | IHSID 专用验证器 | 保留稳定 dense 计算 |
| `integration/exportTrajectoryToSimscape.m` | 导出原始数组和 `references` timeseries | 活动 | 主入口、测试 | Simscape 接口 |
| `integration/prepareSimscapeLengthControl.m` | 统一加载轨迹、配置参数、整定控制器并准备模型 | 活动 | `run_02`、测试 | 保证自动与手动模式配置一致 |
| `integration/buildSimscapeLengthControlData.m` | 将优化参数映射为 Simscape 被控对象数据 | 活动 | 控制入口、测试 | 参数单一来源 |
| `integration/configureSimscapeGravity.m` | 运行期启用或关闭 Simscape 重力 | 活动 | 控制入口、测试 | 保持模型文件通用 |
| `integration/designSimscapeLengthController.m` | 线性化并整定六路对角 PIDF | 活动 | 控制入口、测试 | 长度误差反馈控制器 |
| `integration/evaluateSimscapeLengthControl.m` | 解析闭环结果并执行硬验收 | 活动 | 控制入口、测试 | 控制结果验收 |
| `tools/animateStewartTrajectory.m` | 离线动画 | 工具 | 主入口 | 保留维护价值 |
| `tools/exportStewartMountingDiagram.m` | 装配图导出 | 工具 | 绘图流程 | 保留维护价值 |
| `tools/plotOptResult.m` | 轨迹与约束检查图 | 工具 | 主入口 | 标准样例图来源 |

## 活动测试

| 文件 | 用途 | 状态 | 调用方 | 处理原因 |
|---|---|---|---|---|
| `tests/run_all_active_tests.m` | 运行全部活动契约测试 | 测试 | 用户/CI | 唯一测试入口 |
| `tests/test_01_ihsid_contract.m` | 默认配置、打包解包和 NLP 尺寸 | 测试 | 测试入口 | 主线结构契约 |
| `tests/test_02_simscape_export_contract.m` | `references.r/rL/uFF` 维度与时间契约 | 测试 | 测试入口 | Simscape 导出契约 |
| `tests/test_03_directory_contract.m` | 根目录、unused 依赖和残留扫描 | 测试 | 测试入口 | 防止主线再次发散 |
| `tests/test_04_simscape_model_contract.m` | 加载 `.slx` 并检查输入和控制力日志 | 测试 | 测试入口 | 模型接口契约 |
| `tests/test_05_simscape_parameter_mapping.m` | 验证优化参数到 Simscape 的精确映射 | 测试 | 测试入口 | 参数映射契约 |
| `tests/test_06_simscape_length_control_smoke.m` | 短时验证线性化、整定、仿真和解析 | 测试 | 测试入口 | 控制闭环冒烟测试 |
| `tests/test_07_simscape_preparation_contract.m` | 验证统一准备接口、轨迹校验和手动模式 | 测试 | 测试入口 | 自动/手动模式契约 |
| `tests/test_08_simscape_full_tracking.m` | 验证默认 10 Hz 完整轨迹跟踪硬验收 | 测试 | 测试入口 | 防止完整轨迹再次失控 |

## 归档源码

以下每个文件状态均为 `归档`、活动调用方均为 `无`。同组文件共享归档原因。

### `unused/comparisons/`

原因：历史 CHSID/DMSID/ED、网格/Hessian 与求解器比较，不属于当前唯一 IHSID 主线。

```text
buildCasadiImplicitDMSNLP.m
buildCasadiImplicitHSNLP.m
evaluateImplicitDMSTrajectoryNumeric.m
evaluateImplicitTrajectoryNumeric.m
packHSDecisionImplicit.m
run_04_compare_CHSID_IHSID_DMSID.m
run_08_compare_IHSID_hessian_grid.m
unpackHSDecisionImplicit.m
validateTrajectoryDenseComparison.m
solver_comparison/buildImplicitHSVariableScale.m
solver_comparison/rebuildImplicitTrajectoryFromDecision.m
solver_comparison/run_02_compare_ipopt_sqp.m
solver_comparison/solveImplicitHSSQP.m
solver_comparison/test_03_fmincon_adapter_consistency.m
ED_archive/assembleReducedTrajectoryData.m
ED_archive/buildCasadiEliminatedAccelHSNLP.m
ED_archive/buildCasadiImplicitSolvedDynamics.m
ED_archive/buildCasadiMultipleShootingNLP.m
ED_archive/buildCasadiReducedOCPCommon.m
ED_archive/buildInitialGuessTwoPhaseReduced.m
ED_archive/evaluateDMSTrajectoryNumeric.m
ED_archive/evaluateReducedHSTrajectoryNumeric.m
ED_archive/packReducedDecisionTwoPhase.m
ED_archive/run_03_compare_CHSID_CHSED_DMSID_DMSED.m
ED_archive/test_04_implicit_solve_consistency.m
ED_archive/test_05_hse_dms_common_constraint_consistency.m
ED_archive/test_06_discretization_compare_smoke.m
ED_archive/test_07_four_method_framework_contract.m
ED_archive/unpackReducedDecisionTwoPhase.m
```

### `unused/fatrop/`

原因：FATROP、manual 结构与诊断实验已退出活动求解链。

```text
buildCasadiFatropNativeHSNLP.m
buildCasadiImplicitIHSFatropManualNLP.m
buildCasadiImplicitSolvedDynamics.m
buildInitialGuessFatropNativeHS.m
checkFatropManualStructure.m
checkIHSManualEquivalence.m
createCasadiNlpSolver.m
diagnoseFatropNativeConstraintGroups.m
evaluateFatropNativeHSTrajectoryNumeric.m
evaluateIHSFatropManualTrajectoryNumeric.m
getStatSafe.m
inspectIHSIDManualStructureDimensions.m
packFatropNativeHSDecision.m
packIHSFatropManualDecision.m
probeFatropOptions.m
run_05_compare_IHSID_FATROP.m
run_06_compare_HS_FATROP_NATIVE.m
run_07_diagnose_FATROP_NATIVE_profiles.m
unpackFatropNativeHSDecision.m
unpackIHSFatropManualDecision.m
```

### `unused/legacy_hs/`

原因：旧压缩 HS、预对准、fmincon/SQP 和旧测试实现，仅供历史追溯。

```text
analyzeHSResult.m
buildCollisionGeometry.m
buildCylinderBoxGeometry.m
buildFminconSQPAdapterFromCasadi.m
buildInitialGuessQuinticHS.m
buildInitialGuessQuinticHSCompressed.m
buildInitialGuessQuinticHSImplicit.m
buildOptModelFromSimscape.m
buildPreAlignmentScene.m
compareSolverResults.m
computePreAlignmentTarget.m
costHSDynamic.m
costHSDynamicCompressed.m
evaluateCollisionClearance.m
evaluateCompressedTrajectory.m
nonlconHSDynamic.m
nonlconHSDynamicCompressed.m
packHSDecision.m
packHSDecisionCompressed.m
run_03_compare_hs_vs_dms_ipopt.m
signedDistanceSphereToOBB.m
test_00_casadi_ipopt_ma27.m
test_01_symbolic_numeric_consistency.m
test_02_dense_validation_corrected.m
unpackHSDecision.m
unpackHSDecisionCompressed.m
validateTrajectoryDense.m
```

### `unused/reporting/`

原因：历史比较报告与论文图脚本，不参与轨迹生成或 Simscape 接口。

```text
writeDiscretizationComparisonFigures.m
writeMidtermReportFigures.m
```

## 删除内容

重复顶层/历史测试、FATROP 调试矩阵、失败或重复 `results/`、过时副本已删除。需要追溯时使用远端备份分支 `backup/pre-opt-minimal-ihsid-20260607`。
