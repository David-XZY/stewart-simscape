# opt_minimal：standard IHSID 轨迹生成主线

本目录只维护一条活动轨迹生成链：**standard IHSID + IPOPT/MA27 + 40x20 网格 + limited-memory Hessian**。  
主入口为 `run_01_ihsid_trajectory.m`，仅当求解器成功且工程后验通过时，才发布 Simscape 参考轨迹。
纯长度反馈 Simscape 验证入口为 `run_02_simscape_length_control.m`。

## 快速运行

在 MATLAB 中将当前目录切换到仓库根目录，运行：

```matlab
run('opt_minimal/run_01_ihsid_trajectory.m')
run('opt_minimal/run_02_simscape_length_control.m')
```

运行期产物写入 `opt_minimal/results/`，该目录除 `.gitkeep` 外均被 Git 忽略。

## 输出与验收

主入口固定生成 40x20 IHSID NLP，并依次执行：

1. CasADi/IPOPT/MA27 环境预检。
2. IHSID 初值、NLP 构建、求解和数值轨迹重建。
3. dense 后验、第二阶段连续间隙和工程约束验收。
4. 仅在 `solverSuccess=1` 且 `engineeringPassed=1` 时生成：
   - `refs.q/qd/Fleg/L` 原始节点数组；
   - `references.r=q-q0` 和 `references.rL=L-L0`，均为节点时间上的 `N×6` `timeseries`。

标准可复用样例保存在 `examples/ihsid_40x20_limited_memory/`。

## 目录职责

- `core/`：场景、动力学、碰撞、运动学、离散配置和 IPOPT 公共配置。
- `ihsid/`：IHSID 初值、NLP、决策变量打包/解包和轨迹重建。
- `validation/`：dense 后验、目标函数分解和工程验收。
- `integration/`：Simscape 相对参考导出、参数映射、重力配置、线性化整定和结果验收。
- `tools/`：维护中的绘图、动画和装配图工具。
- `tests/`：仅覆盖活动 IHSID 主线、目录结构和 Simscape 契约。
- `unused/`：保留的历史比较、FATROP、旧 HS 和报告代码；活动代码不得调用。

全部源码位置与状态见 `FILE_CATALOG.md`。

## 测试

```matlab
run('opt_minimal/tests/run_all_active_tests.m')
```

`test_04_simscape_model_contract` 加载并检查 `matlab/stewart_platform_model.slx` 的输入和日志契约。

## Simscape 纯长度反馈

`run_02_simscape_length_control.m` 只使用 `references.rL-dLm`，不使用优化 `Fleg`
前馈或位姿反馈。脚本将优化几何、50 kg 移动平台、100 kg 圆柱负载、合成质心、
惯量和重力映射到 Simscape；腿刚度与阻尼按优化假设设为零，每段杆件保留
`1e-3 kg` 数值正则质量。模型文件只新增 Controller 输出力 `u` 的日志，不改变控制
或物理连接。

当前自动整定候选为 `0.5/0.25/0.125 Hz`。开启重力后的完整 7.5 秒标准轨迹已完成
验证，但现阶段三档纯长度反馈均无法同时满足腿长范围和 `±2000 N` 控制力硬约束；
脚本会保存最后一次完整诊断并明确报错。下一阶段若要求完整轨迹通过，需要放宽
“不使用力前馈”的约束、重新规划更慢轨迹，或补充真实执行器/被动柔性参数。

Simscape 尚未包含真实腿刚度、阻尼、传感器噪声、延迟、执行器饱和和障碍物。
因此被控对象验证不能替代现有 IHSID 碰撞与工程后验。
