# IHSID-FATROP-manual 20x10 自检

本目录由 `run_05_compare_IHSID_FATROP.m` 生成，只自动运行 20x10。

实验组：
- IHSID-IPOPT
- IHSID-FATROP-manual

未运行 DMSID-FATROP-auto，未运行 40x20 或 60x30，未生成动画或图片。

## 结论

- FATROP 插件可发现，toy problem 可 solve。
- 旧 IHSID-FATROP-auto 失败是变量/约束结构不满足 FATROP 要求，不是物理模型错误。
- IHSID-FATROP-manual 已成功初始化并进入求解。
- IHSID-FATROP-manual 返回 solverSuccess=1，但未通过工程后验。
- 相比 IHSID-IPOPT，本次 FATROP manual 没有减少 solveTime、iter_count、Jg 调用次数或 Jg 总时间。

## 关键指标

```text
IHSID-IPOPT:
  solverStatus        = Solved_To_Acceptable_Level
  engineeringPassed   = 1
  solveTime_s         = 13.7314
  iter_count          = 185
  n_eval_jac_g        = 187
  jac_g_total_s       = 8.776

IHSID-FATROP-manual:
  solverStatus        = 0
  solverSuccess       = 1
  engineeringPassed   = 0
  solveTime_s         = 334.9761
  iter_count          = 340
  n_eval_jac_g        = 353
  jac_g_total_s       = 51.791
  maxDenseDynResidual = 157.307563
```
