# UKF 与外环稳健联合整定结果

- 辨识模型：`D:\NPU\Matlab_code\stewart-simscape\opt_minimal\results\pwm_force_identifier_20260613_232220.mat`
- 参考轨迹：`D:\NPU\Matlab_code\stewart-simscape\opt_minimal\examples\ihsid_40x20_limited_memory\simscape_references.mat`
- 实际传感器噪声、偏置和回零残差保持不变。
- 使用完整轨迹与五个随机种子进行稳健性复核。

## 汇总

| 配置 | 平移 RMS 均值 (mm) | 平移峰值最差值 (mm) | 旋转峰值最差值 (deg) | 力跟踪 NRMSE 均值 | PWM 最差峰值 |
|---|---:|---:|---:|---:|---:|
| 整定前 | 2.0189 | 11.1325 | 2.0219 | 0.02851 | 4198.0 |
| 稳健联合整定后 | 1.1344 | 9.0871 | 1.2803 | 0.01500 | 3933.6 |

## 固化参数

- UKF 加速度过程噪声标准差：`0.02 m/s^2`
- 平移位置/速度/积分增益比例：`0.70 / 0.30 / 0.25`
- 旋转位置/速度增益比例：`0.75 / 1.25`
- 外环单腿修正力限幅：`750 N`

原始逐种子结果见 `multiseed_metrics.csv`。
