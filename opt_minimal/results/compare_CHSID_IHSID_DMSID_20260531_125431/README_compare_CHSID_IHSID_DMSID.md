# CHSID / IHSID / DMSID 隐式动力学对比实验

本目录由 `run_04_compare_CHSID_IHSID_DMSID.m` 生成。

## 方法命名

当前 active 实验只保留 CHSID、IHSID、DMSID 三种隐式动力学方法。

- CHSID: Compressed Hermite-Simpson + Implicit Dynamics。旧名 HSI。
- IHSID: Implicit / Full Hermite-Simpson + Implicit Dynamics，中点状态为独立变量。
- DMSID: Direct Multiple Shooting + Implicit Dynamics。本轮新增。

## 统一 IPOPT 设置

- linear_solver: `ma27`
- hessian_approximation: `limited-memory`
- max_iter: `300`
- tol: `1e-06`
- constr_viol_tol: `1e-06`
- acceptable_tol: `0.001`
- acceptable_iter: `1`
- bound_push/bound_frac: `1e-08` / `1e-08`
- mu_strategy: `adaptive`

三种方法均通过 `makeCommonIpoptOptions` 生成设置；若 exact Hessian 无法稳定运行，应统一切换，不允许单独调整某一种方法。

IHSID 的 `maxMidConsistencyResidual` 来自新增 g_mid；CHSID/DMSID 对该字段填 NaN。

## 验证口径

验证分为三层：离散 NLP 可行性、工程可行性、轨迹质量诊断。
CHS 的 dense 非配点动力学残差只作为轨迹质量诊断，不直接否决 engineeringPassed。
DMS 的 dense 动力学残差由积分器定义，不能直接解释为 DMS 物理一致性优于 CHS。

## 当前自检结果

```text
    method     methodName    N1    N2    gridApproach    gridInsertion     h      numZ    numVariables    numEq    numIneq            solverStatus                    ipoptStatus             solverSuccess    successFlag    engineeringPassed    buildTime_s    solveTime_s    totalTime_s    solveTime    iterations    iterCount    iter_count    n_eval_f    n_eval_g    n_eval_grad_f    n_eval_jac_g    n_eval_h    time_nlp_f    time_nlp_g    time_grad_f    time_jac_g    time_hess_l    time_other_upper_bound    jac_g_total_s    jac_g_avg_s    jac_g_pct_solve    casadi_eval_pct_solve    objectiveTotal    objective    nominalCost    powerCost    legAccelCost    singularityCost    forceRateCost    objectiveForce    objectiveLegAccel    objectiveSingularity    maxEqResidual    maxIneqViolation    minStage1Clearance_m    minStage1Clearance    minStage1Gap    minDenseGap    finalGap_m    maxDynResidual    maxDefectResidual    minSigmaMin    maxCondJ    maxAbsLegSpeed    maxAbsLegAccel    maxAbsForce    maxLegSpeedViolation    maxLegAccelViolation    maxPathViolation    maxForceViolation    minSigma    maxStage2LateralError    maxStage2HeightError    maxStage2AttitudeError    maxInsertionBackwardSpeedViolation    stage2Passed    denseSampleCount    dynamicsEvalCountEstimate    maxDenseDynResidual    maxHSDefectResidual    maxMidConsistencyResidual    ipoptLinearSolver    ipoptHessianApproximation    ipoptTol    ipoptConstrViolTol    ipoptMaxIter           failureReason        
    _______    __________    __    __    ____________    _____________    ____    ____    ____________    _____    _______    ____________________________    ____________________________    _____________    ___________    _________________    ___________    ___________    ___________    _________    __________    _________    __________    ________    ________    _____________    ____________    ________    __________    __________    ___________    __________    ___________    ______________________    _____________    ___________    _______________    _____________________    ______________    _________    ___________    _________    ____________    _______________    _____________    ______________    _________________    ____________________    _____________    ________________    ____________________    __________________    ____________    ___________    __________    ______________    _________________    ___________    ________    ______________    ______________    ___________    ____________________    ____________________    ________________    _________________    ________    _____________________    ____________________    ______________________    __________________________________    ____________    ________________    _________________________    ___________________    ___________________    _________________________    _________________    _________________________    ________    __________________    ____________    ____________________________

    "CHSID"     "CHSID"      20    10         20              10          0.25    2384        2384        1000      3847      "Solved_To_Acceptable_Level"    "Solved_To_Acceptable_Level"        true            true              true             22.774         27.515         50.289        27.515         207           207          207          380         380            192             210           NaN          0.98         1.627          1.204         21.467          NaN                2.2372               21.467          0.10222          78.019                91.869               0.27544         0.27544      0.0091687     0.074197       0.032375          0.08011          0.079593          0.15379            0.032375                0.08011           9.3268e-07             0                   0.03512                0.03512            0.03512         0.005         0.005            3.3871          2.9163e-12          0.28455       8.3307         0.2772           0.19828          1103.1                0                       0                     0                    0            0.28455          1.7156e-09               1.0371e-10               1.3218e-10                          0                        true               601                       61                      3.3871              2.9163e-12                       NaN                 "ma27"              "limited-memory"          1e-06            1e-06               300         ""                          
    "IHSID"     "IHSID"      20    10         20              10          0.25    2744        2744        1360      3847      "Solved_To_Acceptable_Level"    "Solved_To_Acceptable_Level"        true            true              true             15.602         14.881         30.484        14.881         185           185          185          308         308            187             187           NaN         0.827          1.46          1.266          9.576          NaN                1.7525                9.576         0.051209          64.348                88.224               0.27536         0.27536      0.0089899     0.074324       0.032303         0.080121          0.079622          0.15395            0.032303               0.080121           9.1362e-07             0                  0.035038               0.035038           0.035038         0.005         0.005            3.2557          9.7779e-13          0.28455       8.3307         0.2779           0.19625            1103                0                       0                     0                    0            0.28455          4.9988e-10                7.137e-10               3.0981e-10                          0                        true               601                       61                      3.2557              9.7779e-13                5.3833e-11                 "ma27"              "limited-memory"          1e-06            1e-06               300         ""                          
    "DMSID"     "DMSID"      20    10         20              10          0.25    2384        2384        1000      3847      "Solved_To_Acceptable_Level"    "Solved_To_Acceptable_Level"        true            true              false            17.754         30.392         48.147        30.392         195           195          195          341         341            197             197           NaN         0.968          1.57          1.266          24.43          NaN                2.1583                24.43          0.12401          80.382                92.898                0.2729          0.2729       0.010971     0.072784       0.030641         0.079905          0.078599          0.15138            0.030641               0.079905           5.0716e-07             0                   0.03503                0.03503            0.03503         0.005         0.005          0.069798          2.6999e-13          0.28467       8.3275        0.27527           0.16316          1103.6                0                       0                     0                    0            0.28467            1.22e-05               6.7852e-05               0.00010519                          0                        false              601                      301                    0.069798              2.6999e-13                       NaN                 "ma27"              "limited-memory"          1e-06            1e-06               300         "求解完成但未通过统一工程后验验证"


```
