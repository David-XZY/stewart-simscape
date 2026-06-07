# standard IHSID 40x20 limited-memory 标准样例

本目录只保存通过求解器与工程后验双重验收的标准样例：

- `simscape_references.mat`：包含 `refs`、`references`、`solverResult`、`denseReport` 和 `result`。
- `summary.txt`：求解与工程后验摘要。
- `trajectory_check.png`：三维轨迹检查图。

`references.r=q-q0` 与 `references.rL=L-L0` 均为节点时间上的 `N×6`
`timeseries`，首个样本为零，可供 `matlab/stewart_platform_model.slx` 使用。

纯长度反馈完整闭环通过 `opt_minimal/run_02_simscape_length_control.m` 验证。当前在
开启重力且不使用优化力前馈时，标准轨迹尚未通过腿长与控制力双重硬验收。
