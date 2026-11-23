# SC-FG-MHE 参数估计主线设计

## 目标

在 `controller/` 下新增一条独立于现有 UKF 的 Stewart-Constrained Factor Graph Moving Horizon Estimation 主线。该方法以滑动窗口非线性最小二乘为核心，把窗口内位姿、速度、传感器偏置、负载参数和执行器参数统一放入优化变量，并用 Stewart 几何闭环、腿长、IMU、动力学、执行器一致性和参数慢变化先验构造 residual。

现有 `controller/ukf_pose_estimation/` 不重写、不删除，只作为对比基线复用。

## 选定路线

采用路线 C：先建立合成数据 + 现有 UKF 对比 + 可开关 residual 的最小研究闭环。

该路线的目的不是用占位图表伪装完整结果，而是在一个可重复的合成 benchmark 中证明以下问题：

1. FG-MHE 相对运动学和 UKF 是否降低位姿误差。
2. 腿长偏置是否可由窗口优化稳定恢复。
3. 负载质量修正量和质心偏移是否可被动力学 residual 约束。
4. 动力学 residual、鲁棒核、执行器参数估计分别贡献多少。
5. 计算时间是否适合离线论文实验和后续在线化探索。

## 目录和文件

新增目录：

- `controller/factor_graph_estimation/`

新增主函数：

- `makeStewartFgMheConfig.m`：配置窗口长度、权重、鲁棒核、参数开关、求解器选项。
- `initializeStewartFgMheWindow.m`：从轨迹、测量和上一窗口结果生成窗口初值。
- `buildStewartFgMheVariables.m`：在结构体变量和优化向量之间双向打包。
- `buildStewartFgMheResiduals.m`：构建 residual 向量和分解统计。
- `solveStewartFgMheWindow.m`：调用 `lsqnonlin`，失败时回退上一窗口。
- `rollStewartFgMheEstimator.m`：在线滑窗滚动估计。
- `evaluateStewartFgMheSeries.m`：生成合成 benchmark，运行多个 FG-MHE variant。
- `compareUkfAndFgMheEstimation.m`：统一比较运动学、UKF 和 FG-MHE。
- `exportFgMheEvidence.m`：导出 12 张图、汇总表和消融表。
- `run_10_factor_graph_parameter_estimation.m`：一键运行主线。

新增测试：

- `controller/tests/test_50_fg_mhe_config_contract.m`
- `controller/tests/test_51_fg_mhe_residual_dimension.m`
- `controller/tests/test_52_fg_mhe_synthetic_recovery.m`
- `controller/tests/test_53_fg_mhe_compare_with_ukf_smoke.m`
- `controller/tests/test_54_fg_mhe_export_evidence.m`

新增论文方法文档：

- `docs/controller_workflow/factor_graph_parameter_estimation_method.md`

## 复用现有代码

必须复用已有 Stewart 工具，避免另造运动学和动力学链路：

- `opt_minimal/core/sgpIK.m` 用于腿长预测。
- `opt_minimal/core/sgpJacobian.m` 用于速度 Jacobian 和奇异性约束。
- `opt_minimal/core/rpy2rotmZYX.m`、`rpyRateMapZYX.m` 用于 IMU 近似积分。
- `opt_minimal/core/computeCompositeRequiredWrench.m` 用于动力学广义力需求。
- `controller/ukf_pose_estimation/simulatePoseImuSensors.m` 用于生成传感器测量。
- `controller/pwm_identification/benchmarkPwmPoseEstimators.m` 用于复用运动学和 UKF 基线。

## 变量设计

每个窗口长度为 `W`。优化变量包含：

- `q`: `6 x W` 位姿。
- `qd`: `6 x W` 速度。
- `qdd`: `6 x W` 加速度，可由速度差分初始化并可选择参与优化。
- `bL`: `6 x 1` 腿长偏置。
- `bAtt`: `3 x 1` 姿态偏置。
- `ba`: `3 x 1` 加速度计零偏。
- `bg`: `3 x 1` 陀螺仪零偏。
- `dm`: `1 x 1` 负载质量修正。
- `dc`: `3 x 1` 负载质心偏移。
- `kF`: `6 x 1` 执行器力增益修正。
- `tauF`: `1 x 1` 一阶滞后时间常数，可固定。
- `cF`: `6 x 1` 等效黏性阻尼，可固定。

状态变量随时间变化，偏置和物理参数在窗口内保持常值。窗口之间通过 `theta_window - theta_previous` 形成慢变化先验。

## residual 设计

`buildStewartFgMheResiduals.m` 返回一个统一 residual 向量，并同时返回分组统计 `breakdown`。

必须支持以下 residual 分组：

- `leg`：`L_meas - L_pred - bL`。
- `attitude`：`wrapAngle(att_meas - q(4:6) - bAtt)`。
- `imu`：轻量 preintegration-like factor，用 IMU 加速度和角速度推进 `q`、`qd`。
- `geometry`：腿长上下界 violation 和归一化 Jacobian 最小奇异值软约束。
- `dynamics`：`Jv(q)' * F_leg - W_required(q, qd, qdd, theta_payload)`。
- `actuator`：用 PWM/等效输入和 `theta_act` 解释支链力。
- `parameterPrior`：偏置和参数相对上一窗口慢变化。

鲁棒核第一版支持 Huber，对腿长、IMU 和动力学 residual 生效。普通最小二乘模式保持原始 residual。

## 求解策略

第一版使用 MATLAB `lsqnonlin`。如果 Optimization Toolbox 不可用，函数返回明确错误并由测试覆盖 smoke 路径；不强行引入 CasADi 或新依赖。

求解失败时：

1. 不抛出导致主流程中断的错误。
2. 返回上一窗口估计或初值。
3. 在结果中记录 `success=false`、`exitflag`、`message` 和 `fallbackUsed=true`。

## 比较和消融

`compareUkfAndFgMheEstimation.m` 至少输出这些方法：

- `kinematic`
- `ukf_stable`
- `ukf_leg_bias`
- `fg_mhe_no_dynamics`
- `fg_mhe_with_dynamics`
- `fg_mhe_robust`
- `fg_mhe_actuator`

`evaluateStewartFgMheSeries.m` 负责构建合成真值、传感器测量、真实参数和异常点扰动；`compareUkfAndFgMheEstimation.m` 负责统一指标表。

## 输出

结果目录：

- `results/reports/factor_graph_estimation/`

必须输出：

- `fg_mhe_results.mat`
- `table_estimation_comparison.csv`
- `table_ablation_factor_graph.csv`
- 至少 12 张 `.fig` 和对应 `.png`

图名固定为：

1. `fig_01_method_structure`
2. `fig_02_sliding_window`
3. `fig_03_position_error_comparison`
4. `fig_04_attitude_error_comparison`
5. `fig_05_leg_bias_convergence`
6. `fig_06_imu_bias_convergence`
7. `fig_07_payload_parameter_convergence`
8. `fig_08_actuator_gain_convergence`
9. `fig_09_residual_breakdown`
10. `fig_10_ablation_bar`
11. `fig_11_solve_time_comparison`
12. `fig_12_robustness_heatmap`

## 验证

新增测试先行：

- `test_50` 验证配置字段、默认开关、权重和求解器选项。
- `test_51` 验证 residual 分组和维度。
- `test_52` 验证合成数据下腿长偏置和负载质量修正量收敛。
- `test_53` 验证运动学、UKF、FG-MHE 比较 smoke 路径。
- `test_54` 验证图表和表格导出。

完成标准：

1. `controller/runs/run_10_factor_graph_parameter_estimation.m` 可运行。
2. 新增 `test_50` 到 `test_54` 通过。
3. 不破坏已有 UKF 和 active test 入口。
4. 结果表能明确回答目标文件列出的七个研究问题。
