function methodDoc = writeSamplingMpcMethodDocument(projectRoot)
% writeSamplingMpcMethodDocument - 写出 SC-MPC 控制方法说明文档
docDir = fullfile(projectRoot, 'docs', 'controller_workflow');
if ~isfolder(docDir)
    mkdir(docDir);
end
methodDoc = fullfile(docDir, 'sampling_mpc_control_method.md');
lines = [
"# Sampling Constrained MPC Tracking Control"
""
"## 方法定位"
"SC-MPC 是一条不同于固定反馈律和 LQI/计算力矩的控制路线。它不直接把误差映射为单次反馈力，而是在每个控制周期生成一组候选力序列，沿短预测时域滚动评估跟踪误差、力幅值、力变化率和安全裕度代价，只执行当前时刻代价最低序列的第一拍。"
""
"## 与现有算法的差异"
"IHSID feedforward baseline 主要依赖轨迹优化给出的前馈力；CT-LQI 使用计算力矩前馈和线性动态反馈；Safety QP 类方法强调单步约束优化和安全约束协调；SC-MPC 进一步显式引入未来若干步预测，因此能直接比较短时域误差收敛、控制平滑性和约束裕度。"
""
"## 当前实现"
"当前实现采用采样式候选序列而不是引入新的大型优化依赖。候选力由计算力矩名义力和一组反馈力缩放构成，每个候选都经过力幅值和力变化率限制，再通过轻量 Stewart task-space 预测模型计算代价。"
""
"## 对比指标"
"导出表包含平移和姿态 RMSE、峰值误差、等效位姿误差、力峰值、最大力变化率、控制能量、最小安全裕度、平均求解时间和约束违反次数。对比对象包含 IHSID、CT-LQI、Safety QP surrogate 和 SC-MPC。"
];
fid = fopen(methodDoc, 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid));
for index = 1:numel(lines)
    fprintf(fid, '%s\n', lines(index));
end
end
