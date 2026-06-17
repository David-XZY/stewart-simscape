# results：统一结果目录

本目录统一保存轨迹规划、控制验证、汇报审查和历史备份结果。

## 分类

- `trajectory_planning/`：`opt_minimal/run_01_ihsid_trajectory.m` 新生成的轨迹规划结果。
- `controller/`：Simscape 控制、PWM 辨识、UKF 评估和控制对比结果。
- `reports/`：组会、审查、检查、技术报告和过程分析导出。
- `archive/opt_minimal_results_legacy/`：旧 `opt_minimal/results/` 的完整历史备份。

运行期结果默认不进入 Git；需要长期保留的精选结果应显式加入版本控制。
