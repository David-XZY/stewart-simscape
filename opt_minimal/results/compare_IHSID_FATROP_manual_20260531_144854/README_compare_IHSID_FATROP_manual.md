# IHSID-FATROP-manual 20x10 自检

本目录由 `run_05_compare_IHSID_FATROP.m` 生成，只自动运行 20x10。

实验组：IHSID-IPOPT、IHSID-FATROP-manual。未运行 DMSID-FATROP-auto，未生成动画或图片。

- fatropPluginAvailable: 是
- fatropToySolvePassed: 是

```text
    method           caseName           solverBackend    fatropStructure    N1    N2    numVariables    numZ    numEq    numIneq            solverStatus            solverSuccess    engineeringPassed    buildTime_s    solveTime_s    totalTime_s    objective    iter_count    n_eval_f    n_eval_g    n_eval_grad_f    n_eval_jac_g    time_nlp_f    time_nlp_g    time_grad_f    time_jac_g    jac_g_total_s    jac_g_avg_s    jac_g_pct_solve    casadi_eval_pct_solve    time_other_upper_bound    maxEqResidual    maxIneqViolation    minStage1Clearance_m    finalGap_m    maxDenseDynResidual    maxHSDefectResidual    maxMidConsistencyResidual    maxStage2LateralError    maxStage2HeightError    maxStage2AttitudeError    stage2Passed    fatrop_iterations_count    fatrop_eval_jac_count    fatrop_eval_obj_count    fatrop_eval_grad_count    fatrop_eval_cv_count    fatrop_return_flag                                                           failureReason                                                       
    _______    _____________________    _____________    _______________    __    __    ____________    ____    _____    _______    ____________________________    _____________    _________________    ___________    ___________    ___________    _________    __________    ________    ________    _____________    ____________    __________    __________    ___________    __________    _____________    ___________    _______________    _____________________    ______________________    _____________    ________________    ____________________    __________    ___________________    ___________________    _________________________    _____________________    ____________________    ______________________    ____________    _______________________    _____________________    _____________________    ______________________    ____________________    __________________    ___________________________________________________________________________________________________________________________

    "IHSID"    "IHSID-IPOPT"              "ipopt"           ""              20    10        2744        2744    1360      3847      "Solved_To_Acceptable_Level"        true               true              15.32         16.816         32.136        0.27536        185           308        308            187             187           0.856         1.434          1.435         11.055         11.055         0.059118          65.741                87.893                    2.0359             9.1362e-07                  0             0.035038            0.005             3.2557               9.7779e-13                5.3833e-11                 4.9988e-10                7.137e-10               3.0981e-10             true                   NaN                       NaN                      NaN                      NaN                      NaN                    ""             ""                                                                                                                         
    "IHSID"    "IHSID-FATROP-manual"      "fatrop"          "manual"        20    10        3720        3720    2080      3847      "0"                                 true               false            49.743         435.65         485.39         12.222        340          1563        955            322             353           7.137          4.75           3.99         69.428         69.428          0.19668          15.937                19.581                    350.35              9.886e-09         9.9999e-09             0.013576            0.005             157.31               6.4993e-09                3.2497e-09                 4.6953e-10               5.9136e-10               5.0555e-11             true                   340                       322                     1536                      322                      901                    "0"            "求解完成但未通过统一工程后验验证；stage2Passed=1, maxDenseDynResidual=157.308, minStage1Clearance_m=0.0135755, finalGap_m=0.005"


```

## 问题回答

1. 当前 FATROP 插件是否可用，toy problem 是否能 solve？插件可发现=是，toy solve=是。
2. 旧 IHSID-FATROP-auto 为什么失败？原因是旧 z/g 按类型集中堆叠，且 HS/IHS gap 对下一阶段状态不是 FATROP 要求的 identity 形式；这是变量/约束结构问题，不是物理模型错误。
3. IHSID-FATROP-manual 是否成功初始化？是。
4. IHSID-FATROP-manual 是否成功求解并通过工程后验？求解=是，工程后验=否。
5. 与 IHSID-IPOPT 相比是否减少 solveTime、iter_count、Jg 调用次数、Jg 总时间？solveTime=否，iter=否，Jg调用=否，Jg总时间=否。
6. 若仍失败，失败位置：0；failureReason=求解完成但未通过统一工程后验验证；stage2Passed=1, maxDenseDynResidual=157.308, minStage1Clearance_m=0.0135755, finalGap_m=0.005。
