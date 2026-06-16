# opt_minimal 文件目录

状态说明：`活动` 表示当前主线直接或间接使用；`测试` 表示活动契约测试；`工具` 表示主线可调用的维护工具；`保留` 表示不再由活动入口调用但暂留复查；`归档` 表示仅供历史追溯，活动代码不得调用。

## 根目录

| 文件 | 用途 | 状态 | 调用方/入口 | 处理原因 |
|---|---|---|---|---|
| `run_01_ihsid_trajectory.m` | standard IHSID 40x20 limited-memory 唯一主入口 | 活动 | 用户直接运行 | 由旧主入口改名并收敛为 IHSID |
| `run_02_simscape_length_control.m` | 自动运行或手动准备力输入位姿轨迹跟踪 | 活动 | 用户直接运行 | 重力开启的力驱动控制、Run03 对比与完整硬验收入口 |
| `run_03_simscape_length_cascade_control.m` | 自动运行或手动准备纯长度串级控制 | 活动 | 用户直接运行 | 默认 1.0/0.7 增益缩放的长度伺服与完整硬验收入口 |
| `run_04_simscape_pose_length_control.m` | 自动运行或手动准备位姿反馈纯腿长输入控制 | 活动 | 用户直接运行 | 在 Run03 上增加平台位姿误差反馈，方便与 Run02 对比 |
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
| `integration/reconstructHermitePoseReference.m` | 由节点 q/qd 重建规则网格位姿参考 | 活动 | Run02、Run03、测试 | 保证参考位姿与速度导数一致 |
| `integration/makeSimscapePoseForceConfig.m` | 构建力输入位姿控制统一配置 | 活动 | `run_02`、测试 | 默认整定参数与约束单一来源 |
| `integration/designSimscapePoseForceController.m` | 线性化并整定笛卡尔位姿动态反馈控制器 | 活动 | `run_02`、测试 | 复用 `Reference-Tracking-X` 与 `Jv^-T` 力映射 |
| `integration/prepareSimscapePoseForceControl.m` | 加载轨迹、整定控制器并准备位姿力控制模型 | 活动 | `run_02`、测试 | 保证自动与手动模式配置一致 |
| `integration/evaluateSimscapePoseForceControl.m` | 解析位姿力控制结果并执行硬验收 | 活动 | `run_02`、测试 | 统一位姿、腿运动和总力验收 |
| `integration/comparePoseTrackingPerformance.m` | 计算 Run02/Run03 位姿综合分 | 活动 | `run_02`、测试 | 固化位姿优先比较口径 |
| `integration/prepareSimscapeLengthControl.m` | 旧长度反馈准备实现 | 保留 | 无活动入口 | 仅供历史结果复查 |
| `integration/buildSimscapeLengthControlData.m` | 将优化参数映射为 Simscape 被控对象数据 | 活动 | 控制入口、测试 | 参数单一来源 |
| `integration/installSimscapePayloadGeometry.m` | 将刚性载荷实体配置为横卧真实尺寸圆柱 | 活动 | 模型维护 | 统一中期报告、优化模型和 Simscape 外形与惯量坐标 |
| `integration/configureSimscapeGravity.m` | 运行期启用或关闭 Simscape 重力 | 活动 | 控制入口、测试 | 保持模型文件通用 |
| `integration/designSimscapeLengthController.m` | 旧长度反馈 PIDF 整定实现 | 保留 | 无活动入口 | 仅供历史结果复查 |
| `integration/evaluateSimscapeLengthControl.m` | 旧长度反馈验收实现 | 保留 | 无活动入口 | 仅供历史结果复查 |
| `integration/generateSimscapeLengthCascadeReferences.m` | 由 q/qd 生成腿长与腿速参考 | 活动 | `run_03`、测试 | 移除长度模式的力轨迹依赖 |
| `integration/makeSimscapeLengthCascadeConfig.m` | 构建纯长度串级统一配置 | 活动 | `run_03`、测试 | 采样、对象与约束参数单一来源 |
| `integration/designSimscapeLengthCascadeController.m` | 逐腿自动整定位置 P 与速度 PIDF | 活动 | `run_03`、测试 | 仿真前自整定 |
| `integration/prepareSimscapeLengthCascadeControl.m` | 准备纯长度 Variant、参考与基础工作区 | 活动 | `run_03`、测试 | 自动/手动模式统一入口 |
| `integration/evaluateSimscapeLengthCascadeControl.m` | 解析纯长度日志并执行无力指标硬验收 | 活动 | `run_03`、测试 | 纯长度控制结果验收 |
| `integration/makeSimscapePoseLengthConfig.m` | 构建位姿反馈纯腿长配置 | 活动 | `run_04`、测试 | 固化位姿反馈增益与腿长修正限幅 |
| `integration/designSimscapePoseLengthController.m` | 构造低通位姿误差到腿长修正的增益调度 | 活动 | `run_04`、测试 | 复用 Run03 串级控制与时变参考雅可比 |
| `integration/prepareSimscapePoseLengthControl.m` | 准备位姿反馈纯腿长 Variant 与基础工作区 | 活动 | `run_04`、测试 | 自动/手动模式统一入口 |
| `integration/evaluateSimscapePoseLengthControl.m` | 解析并验收位姿反馈纯腿长结果 | 活动 | `run_04`、测试 | 复用 Run03 硬验收并增加修正量诊断 |
| `integration/installSimscapeLengthCascadeVariants.m` | 在现有 SLX 中安装长度控制与执行器 Variant | 活动 | 模型维护 | 保持 SLX 增量修改可复现 |
| `tools/animateStewartTrajectory.m` | 离线动画 | 工具 | 主入口 | 保留维护价值 |
| `tools/exportStewartMountingDiagram.m` | 装配图导出 | 工具 | 绘图流程 | 保留维护价值 |
| `tools/exportRun02Run03MeetingFigures.m` | 导出 Run02/Run03 总览、对比与纹波诊断 FIG/PNG | 工具 | 用户直接调用 | 汇报图与纹波来源诊断可复现导出 |
| `tools/exportRun02Run03Run04Comparison.m` | 导出 Run02/Run03/Run04 共同误差与位姿指标对比 | 工具 | 用户直接调用 | 对比执行器输入和位姿反馈结构 |
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
| `tests/test_06_simscape_length_control_smoke.m` | 短时验证力输入位姿控制不会起步坠落 | 测试 | 测试入口 | 控制闭环冒烟测试 |
| `tests/test_07_simscape_preparation_contract.m` | 验证位姿力控制统一准备接口 | 测试 | 测试入口 | 自动/手动模式契约 |
| `tests/test_08_simscape_full_tracking.m` | 验证默认位姿力控制完整轨迹硬验收 | 测试 | 测试入口 | 防止完整轨迹再次失控 |
| `tests/test_09_length_cascade_reference_contract.m` 至 `test_13_length_cascade_evaluation_contract.m` | 验证纯 q/qd 参考、整定、准备、SLX 与验收契约 | 测试 | 测试入口 | 纯长度控制结构契约 |
| `tests/test_14_length_cascade_smoke.m` | 短时验证纯长度 Variant 可运行 | 测试 | 测试入口 | 纯长度冒烟测试 |
| `tests/test_15_length_cascade_full_tracking.m` | 验证标准全轨迹纯长度硬验收 | 测试 | 测试入口 | 防止纯长度轨迹回归 |
| `tests/test_16_meeting_figure_export_contract.m` | 验证四组 FIG/PNG 与纹波诊断字段 | 测试 | 测试入口 | 汇报图和纹波诊断导出契约 |
| `tests/test_17_pose_force_control_contract.m` 至 `test_19_pose_force_full_tracking.m` | 验证位姿力配置、整定、准备、综合分和完整轨迹 | 测试 | 测试入口 | 新 Run02 控制结构契约 |
| `tests/test_20_hermite_reference_contract.m` | 验证 Hermite 重建精确通过节点 q/qd | 测试 | 测试入口 | 防止参考重建退化为线性插值 |
| `tests/test_21_pose_length_control_contract.m` 至 `test_23_pose_length_full_tracking.m` | 验证 Run04 配置、准备链路和完整轨迹 | 测试 | 测试入口 | 防止位姿反馈纯腿长控制回归 |

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
## PWM 执行器辨识与控制扩展

| 文件/目录 | 用途 | 状态 |
|---|---|---|
| `run_05_generate_pwm_identification_data.m` | 生成含 PWM、编码器量化、位姿观测和隐藏真值的数据集 | 活动 |
| `run_06_train_pwm_force_identifier.m` | 训练并验证灰箱加残差 NARX 力估计器 | 活动 |
| `run_07_compare_pwm_pose_force_control.m` | 比较 oracle、辨识真值位姿反馈和辨识 UKF 反馈三条闭环线 | 活动 |
| `run_09_export_four_control_group_meeting_comparison.m` | 导出纯力、高保真、真实位姿辨识和 UKF 辨识四方案中文组会图 | 活动 |
| `run_08_export_pwm_feedback_source_comparison.m` | 导出反馈源消融实验的组会结果包 | 活动 |
| `actuator_identification/` | 高保真物理教师、辨识器、PWM 力内环、双线仿真与验收 | 活动 |
| `integration/installSimscapePwmActuatorVariant.m` | 在现有支链 SLX 中增量安装 `PWM-Physical` Variant | 活动 |
| `tools/exportPwmIdentificationEvidence.m` | 导出 PWM 物理模型、辨识与双线控制全链路图表和数值证据 | 工具 |
| `tools/exportIdealPwmMacroMotionComparison.m` | 对比纯理想执行器与 PWM 真值/辨识反馈线的宏观运动表现 | 工具 |
| `tools/exportPwmFeedbackSourceComparison.m` | 导出三线反馈源消融的数值表、误差归因和八张组会图片 | 工具 |
| `tools/exportPwmUkfOuterLoopTuningEvaluation.m` | 导出 UKF 与外环联合整定前后的完整轨迹多种子对比 | 工具 |
| `tests/test_24...test_30` | 覆盖物理模型、辨识、控制、SLX、开关校验和完整轨迹 | 测试 |

当前 PWM 真实传感器和 UKF 闭环完整轨迹验收采用每个平移轴峰值误差 `<= 10 mm`、收敛后估计器平移轴峰值误差 `<= 4 mm`、旋转峰值误差 `<= 1.5 deg`。
## 相对编码器与 IMU UKF

| 文件 | 用途 | 状态 |
|---|---|---|
| `actuator_identification/makePoseImuUkfConfig.m` | 构造回零锚定 UKF 配置 | 活动 |
| `actuator_identification/initializePoseImuUkf.m` | 初始化公共 18 状态 UKF | 活动 |
| `actuator_identification/stepPoseImuUkf.m` | 使用 IMU 预测、相对腿长和姿态校正 | 活动 |
| `actuator_identification/simulatePoseImuSensors.m` | 生成不含运行时位置测量的传感器数据 | 活动 |
| `actuator_identification/estimatePoseImuUkfSeries.m` | 批量运行世界系/原始比力 UKF | 活动 |
| `tests/test_32...test_35` | 公共接口、传感器隔离、双模式恢复和极限精度测试 | 测试 |
