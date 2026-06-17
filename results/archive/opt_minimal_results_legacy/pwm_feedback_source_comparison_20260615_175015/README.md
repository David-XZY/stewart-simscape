# PWM 辨识闭环反馈源消融实验

- 辨识模型：`D:\NPU\Matlab_code\stewart-simscape\opt_minimal\results\pwm_force_identifier_20260613_232220.mat`
- 参考轨迹：`D:\NPU\Matlab_code\stewart-simscape\opt_minimal\examples\ihsid_40x20_limited_memory\simscape_references.mat`
- 总体验收通过：`0`

## 核心结果

| 控制线 | 平移 RMS (mm) | 平移峰值 (mm) | 旋转 RMS (deg) | 旋转峰值 (deg) | 力跟踪 NRMSE | 对齐力估计 RMS (N) |
|---|---:|---:|---:|---:|---:|---:|
| oracle | 0.076385 | 0.276182 | 0.068163 | 0.214321 | 0.015708 | 0.000 |
| identifiedTruthFeedback | 0.141282 | 0.351012 | 0.264346 | 0.558731 | 0.019548 | 59.063 |
| identified | 3.067115 | 8.720525 | 0.374471 | 1.136161 | 0.027579 | 62.736 |

## 误差归因

- UKF 相对真值位姿反馈的平移 RMS 增量：`2.925833 mm`
- 辨识模型/力环相对 oracle 的平移 RMS 增量：`0.064897 mm`
- UKF 相对真值位姿反馈的旋转 RMS 增量：`0.110125 deg`
- 真值位姿反馈与 UKF 反馈的对齐力估计 RMS：`59.063 N / 62.736 N`

## 组会讲述建议

1. 原双线实验把力辨识误差与 UKF 反馈误差混在一起，本实验新增真值位姿反馈线进行单变量消融。
2. 真值位姿反馈线仍使用灰箱+NARX辨识力，因此不是完整 oracle。
3. 先展示三线总体指标，再使用误差来源分解图判断后续优化优先级。
4. 若真值位姿反馈显著改善力估计，说明 UKF 速度误差通过腿速输入污染了 NARX。
