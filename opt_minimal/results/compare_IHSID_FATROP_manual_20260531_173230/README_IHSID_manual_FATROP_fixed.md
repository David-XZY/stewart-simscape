# IHSID manual FATROP fixed check

本目录由 `run_05_compare_IHSID_FATROP.m` 的 20x10 流程生成，并只覆盖 IHSID-MANUAL-FATROP 接入检查。未运行 DMS、ED、CHS 或自适应网格路线。

## 结论

- IHSID-STANDARD-IPOPT：solverSuccess=1，engineeringPassed=1，objective=0.275360058177391。
- IHSID-MANUAL-IPOPT：solverSuccess=1，engineeringPassed=1，objective=0.277783416056183。
- manual 等价性初值检查：objectiveDiffAtX0=0，maxManualGapResidualAtX0=0。
- IHSID-MANUAL-FATROP：solverStatus=FATROP_LIMITED_MEMORY_UNAVAILABLE，solverSuccess=0，engineeringPassed=0。
- FATROP 对比有效性：fatropComparisonValid=0。失败原因：当前 CasADi FATROP 接口不接受 hessian_approximation=limited-memory，按要求停止 FATROP 对比。

## 文件

- `comparison_summary.csv`：三组方法的结构、solver、后验与失败原因汇总。
- `comparison_report.txt`：自动生成的文本报告。
- `comparison_results.mat`：MATLAB 结构化结果。
- `manual_structure_check.txt`：manual 结构自检。
- `manual_equivalence_check.txt`：standard/manual 初值等价性检查。
- `fatrop_solver_log.txt`：FATROP 未进入正式求解的原因记录。
