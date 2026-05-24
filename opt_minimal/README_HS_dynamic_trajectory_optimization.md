# Stewart Hermite-Simpson 完整动力学轨迹优化

## 本模块解决什么问题

本模块使用 Hermite-Simpson 直接配点法，为自定义 6-UCU Stewart 平台生成从 `q0` 到 `qf` 的完整动力学约束轨迹。优化器在 MATLAB 中运行，Simscape 只保留为可选参考和后续验证接口。

## 为什么从有限差分节点法改为 Hermite-Simpson

旧方法只优化位姿节点，并用有限差分估计速度和加速度。这样端点速度、加速度和积分一致性都比较间接。Hermite-Simpson 方法把位姿 `Q`、速度 `V`、加速度 `A` 和中点加速度 `Ac` 都作为优化变量，再用配点等式保证积分一致性，更适合轨迹优化。

## 优化变量

决策变量统一为：

```matlab
z = [Q(:); V(:); A(:); Ac(:)]
```

- `Q`：`6 x N` 节点平台位姿 `[x;y;z;roll;pitch;yaw]`
- `V`：`6 x N` 节点平台广义速度
- `A`：`6 x N` 节点平台广义加速度
- `Ac`：`6 x (N-1)` 每个区间中点平台广义加速度

端点位姿和端点速度通过等式约束固定，端点加速度不固定。

## Hermite-Simpson 公式

区间中点状态为：

```matlab
qc = 0.5*(qk + qk1) + h/8*(vk - vk1)
vc = 0.5*(vk + vk1) + h/8*(ak - ak1)
```

配点等式为：

```matlab
qk1 - qk = h/6*(vk + 4*vc + vk1)
vk1 - vk = h/6*(ak + 4*ac + ak1)
```

第一条表示位姿由速度积分得到，第二条表示速度由加速度积分得到。

## 当前路径约束

节点和中点同时检查：

- 支链长度：`0.40 m <= L <= 0.80 m`
- 支链速度：`abs(Ld) <= ldotMax`
- 支链加速度：`abs(Ldd) <= lddotMax`
- 奇异性：`sigmaMin >= sigmaMinSafe`
- 条件数：`condJ <= condJMax`
- 完整逆动力学驱动力：`forceMin <= Fleg <= forceMax`

当前不包含平台位姿边界、虎克铰摆角、功率约束、优化变量上下界和 Simscape 联合优化。

## 如何运行

在 MATLAB 中进入 `stewart-simscape` 工程根目录，运行：

```matlab
run('opt_minimal/run_01_hs_dynamic_opt.m')
```

结果保存到：

```text
opt_minimal/results/hs_dynamic_result_yyyymmdd_HHMMSS.mat
```

动画保存到 `opt_minimal/results/`。

## 文件作用

- `run_01_hs_dynamic_opt.m`：HS 完整动力学主脚本。
- `buildOptModelCustom.m`：默认自定义 Stewart 几何、动力学和约束参数。
- `buildOptModelFromSimscape.m`：Simscape 参数参考入口，不作为默认模型。
- `computeAnchorPoints66.m`：生成 6-6 平台铰点。
- `packHSDecision.m` / `unpackHSDecision.m`：打包和解包 HS 决策变量。
- `buildInitialGuessQuinticHS.m`：五次多项式初值。
- `computeHSMidpoint.m`：HS 中点状态公式。
- `evaluatePathConstraintsAtPoint.m`：单点路径约束和物理量计算。
- `costHSDynamic.m`：Simpson 积分目标函数。
- `nonlconHSDynamic.m`：HS 等式约束和路径约束。
- `sgpIK.m`、`sgpJacobian.m`、`computeLegKinematics.m`：运动学计算。
- `inverseDynamicsFullUCU.m`、`computePlatformWrenchFull.m`、`computeLegInertiaWrenchFull.m`：完整逆动力学近似。
- `analyzeHSResult.m`：优化结果分析。
- `plotOptResult.m`：中文结果图。
- `animateStewartTrajectory.m`：三维 Stewart 动画。
- `exportTrajectoryToSimscape.m`：导出节点参考轨迹。

## 后续扩展

可将 `N=21` 改为 25 或 30 增加离散精度；也可进一步提供解析梯度、稀疏雅可比或切换到更适合大规模直接配点的优化器。
