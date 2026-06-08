# standard IHSID 40x20 limited-memory 标准样例

本目录只保存通过求解器与工程后验双重验收的标准样例：

- `simscape_references.mat`：包含 `refs`、`references`、`solverResult`、`denseReport` 和 `result`。
- `summary.txt`：求解与工程后验摘要。
- `trajectory_check.png`：三维轨迹检查图。

`references.r=q-q0`、`references.rL=L-L0` 与 `references.uFF=Fleg` 均为节点时间上的
`N×6` `timeseries`；`r/rL` 首个样本为零，`uFF` 首个样本为优化初始支撑力。
这些信号可供 `matlab/stewart_platform_model.slx` 使用。

Simscape 闭环由 `opt_minimal/run_02_simscape_length_control.m` 运行和诊断，当前控制结构为
IHSID 力前馈加长度误差反馈：`u=uFF+uFeedback`。短时静态支撑测试已覆盖初始重力补偿；
完整 7.5 秒轨迹仍以硬验收结果为准。
