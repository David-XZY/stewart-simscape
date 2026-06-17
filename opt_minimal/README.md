# opt_minimal：轨迹规划与优化主线

`opt_minimal` 现在只保留 Stewart 平台轨迹规划、IHSID 优化和轨迹后验验证相关代码。控制、Simscape 闭环、PWM、UKF 和汇报导出入口已移到根目录 `controller/`。

## 快速运行

在仓库根目录下运行：

```matlab
run('opt_minimal/run_01_ihsid_trajectory.m')
```

轨迹规划结果默认写入：

```text
results/trajectory_planning/
```

历史结果完整备份在：

```text
results/archive/opt_minimal_results_legacy/
```

## 目录职责

- `run_01_ihsid_trajectory.m`：standard IHSID 40x20 limited-memory 默认轨迹生成入口。
- `core/`：几何、运动学、动力学、碰撞、约束和 IPOPT 公共配置。
- `ihsid/`：IHSID NLP 构建、初值、打包/解包和轨迹重建。
- `validation/`：dense 后验、工程约束验收和目标函数分解。
- `tools/`：轨迹规划结果绘图、动画和装配图导出工具。
- `examples/`：可复用示例轨迹和参考数据。
- `unused/`：历史比较、FATROP、旧 HS 和报告脚本归档，活动代码不得调用。

## 与控制主线的关系

`run_01` 只负责生成通过求解器和工程后验验收的轨迹参考。控制主线从 `controller/runs/` 读取这些参考，或读取 `opt_minimal/examples/` 下的标准样例。

常用控制入口：

```matlab
run('controller/runs/run_02_simscape_length_control.m')
run('controller/runs/run_03_simscape_length_cascade_control.m')
run('controller/runs/run_04_simscape_pose_length_control.m')
```

活动测试入口：

```matlab
run('controller/tests/run_all_active_tests.m')
```
