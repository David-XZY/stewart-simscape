# controller：控制与闭环验证主线

`controller` 存放 Simscape 控制、PWM 执行器辨识、UKF 位姿估计、控制结果导出和活动测试。轨迹规划仍由 `opt_minimal/run_01_ihsid_trajectory.m` 负责。

## 目录职责

- `runs/`：控制和汇报导出入口，包含原 `run_02` 到 `run_09`。
- `simscape_tracking/`：Simscape 参考重建、控制器设计、模型变体安装、控制结果评估。
- `pwm_identification/`：PWM 物理执行器、灰箱/NARX 力辨识、PWM 闭环控制。
- `ukf_pose_estimation/`：相对编码器和 IMU 位姿 UKF。
- `tools/`：控制结构图、PWM/UKF 证据、组会和技术报告导出工具。
- `tests/`：活动契约测试入口和测试用例。

## 常用入口

```matlab
run('controller/runs/run_02_simscape_length_control.m')
run('controller/runs/run_03_simscape_length_cascade_control.m')
run('controller/runs/run_04_simscape_pose_length_control.m')
run('controller/runs/run_05_generate_pwm_identification_data.m')
run('controller/runs/run_06_train_pwm_force_identifier.m')
run('controller/runs/run_07_compare_pwm_pose_force_control.m')
```

控制结果默认写入 `results/controller/`，汇报和审查类导出默认写入 `results/reports/`。

## 测试

```matlab
run('controller/tests/run_all_active_tests.m')
```
