# standard IHSID 40x20 limited-memory 标准样例

本目录只保存通过求解器与工程后验双重验收的标准样例：

- `simscape_references.mat`：包含 `refs`、`references`、`solverResult`、`denseReport` 和 `result`。
- `summary.txt`：求解与工程后验摘要。
- `trajectory_check.png`：三维轨迹检查图。

`references.r` 与 `references.rL` 均为节点时间上的 `N×6` `timeseries`，可供 `matlab/stewart_platform_model.slx` 使用。
