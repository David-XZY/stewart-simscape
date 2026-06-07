# opt_minimal：standard IHSID 轨迹生成主线

本目录只维护一条活动轨迹生成链：**standard IHSID + IPOPT/MA27 + 40x20 网格 + limited-memory Hessian**。  
主入口为 `run_01_ihsid_trajectory.m`，仅当求解器成功且工程后验通过时，才发布 Simscape 参考轨迹。

## 快速运行

在 MATLAB 中将当前目录切换到仓库根目录，运行：

```matlab
run('opt_minimal/run_01_ihsid_trajectory.m')
```

运行期产物写入 `opt_minimal/results/`，该目录除 `.gitkeep` 外均被 Git 忽略。

## 输出与验收

主入口固定生成 40x20 IHSID NLP，并依次执行：

1. CasADi/IPOPT/MA27 环境预检。
2. IHSID 初值、NLP 构建、求解和数值轨迹重建。
3. dense 后验、第二阶段连续间隙和工程约束验收。
4. 仅在 `solverSuccess=1` 且 `engineeringPassed=1` 时生成：
   - `refs.q/qd/Fleg/L` 原始节点数组；
   - `references.r` 和 `references.rL`，均为节点时间上的 `N×6` `timeseries`。

标准可复用样例保存在 `examples/ihsid_40x20_limited_memory/`。

## 目录职责

- `core/`：场景、动力学、碰撞、运动学、离散配置和 IPOPT 公共配置。
- `ihsid/`：IHSID 初值、NLP、决策变量打包/解包和轨迹重建。
- `validation/`：dense 后验、目标函数分解和工程验收。
- `integration/`：Simscape 参考轨迹导出。
- `tools/`：维护中的绘图、动画和装配图工具。
- `tests/`：仅覆盖活动 IHSID 主线、目录结构和 Simscape 契约。
- `unused/`：保留的历史比较、FATROP、旧 HS 和报告代码；活动代码不得调用。

全部源码位置与状态见 `FILE_CATALOG.md`。

## 测试

```matlab
run('opt_minimal/tests/run_all_active_tests.m')
```

`test_04_simscape_model_contract` 只加载并检查 `matlab/stewart_platform_model.slx`，不会保存或修改模型。

## Simscape 下一阶段接入

本轮不修改控制器和 `.slx`。下一阶段可加载标准样例 MAT 文件，将其中 `references` 放入模型工作区；现有模型继续消费 `references.r` 与 `references.rL`。
