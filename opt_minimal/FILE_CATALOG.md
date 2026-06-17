# opt_minimal 文件目录

本目录只维护轨迹规划与优化主线。

## 活动入口

| 路径 | 职责 |
|---|---|
| `run_01_ihsid_trajectory.m` | standard IHSID 轨迹生成、dense 后验、结果发布 |

## 活动源码

| 目录 | 职责 |
|---|---|
| `core/` | Stewart 几何、运动学、动力学、碰撞、约束、IPOPT 公共配置 |
| `ihsid/` | IHSID NLP、初值、决策变量打包/解包、轨迹重建 |
| `validation/` | dense 后验、工程约束验收、目标函数分解 |
| `tools/` | 轨迹规划结果图、动画、装配图导出 |
| `examples/` | 标准样例轨迹和 Simscape 参考数据 |

## 归档源码

| 目录 | 职责 |
|---|---|
| `unused/comparisons/` | 历史 CHSID/DMSID/ED、网格 Hessian 和求解器比较 |
| `unused/fatrop/` | FATROP/manual 结构和诊断实验 |
| `unused/legacy_hs/` | 旧 HS、fmincon/SQP、预对准和旧测试实现 |
| `unused/reporting/` | 历史报告与论文图脚本 |

活动源码不得依赖 `unused/`。

## 外移内容

| 新路径 | 内容 |
|---|---|
| `controller/` | Simscape 控制、PWM、UKF、控制入口、控制导出工具、活动测试 |
| `docs/controller_workflow/` | 完整仿真/控制流程报告和图表源文件 |
| `results/` | 新的统一结果目录和旧结果备份 |
| `matlab/simscape_subsystems/` | 顶层模型引用的 Simscape 子系统模型 |
