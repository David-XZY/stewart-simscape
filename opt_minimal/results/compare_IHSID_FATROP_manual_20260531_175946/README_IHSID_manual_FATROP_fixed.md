# IHSID manual FATROP exact Hessian diagnostic

本目录记录 20x10 exact Hessian 对比实验。IPOPT 对比组已切到 exact Hessian；FATROP 不再因 limited-memory option 不支持而跳过，实际运行 30 次 smoke。

- IHSID-STANDARD-IPOPT: solverSuccess=1, engineeringPassed=0, objective=15.3976570749151
- IHSID-MANUAL-IPOPT: solverSuccess=1, engineeringPassed=0, objective=16.1535208982347
- IHSID-MANUAL-FATROP: solverSuccess=0, engineeringPassed=0, objective=93.7819873891232
- FATROP degenerateJacobianCount=30, n_eval_hess_l=30, time_hess_l=32.563

结论：FATROP 已进入真实求解，但 30 次迭代每次都出现 degenerate Jacobian，且 dense 工程后验不通过。
