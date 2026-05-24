function result = analyzeHSResult(traj, denseReport, c, ceq, solverResult)
% analyzeHSResult - 汇总预对准 HS 完整 NLP 求解结果
%
% 文件用途：
%   统计端点状态误差、HS 等式残差、路径硬约束违反量、碰撞间隙、驱动力范围和
%   求解器退出状态。
%
% 输入参数：
%   traj struct - 主脚本重建的节点/中点轨迹，包含 xStart/xEnd。
%   denseReport struct - validateTrajectoryDense 输出的联合检查报告。
%   c/ceq - nonlconHSDynamic 返回的不等式与等式约束。
%   solverResult struct - fmincon 输出摘要。
%
% 输出参数：
%   result struct - 统一诊断结构；successFlag 仅在硬约束和等式残差均满足容差时为 true。
%
% 核心公式：
%   maxPathViolation=max(max(c),0)，hsEqualityMax=max(abs(ceq))。
%
% 在优化链路中的作用：
%   主脚本保存 result，并据此决定是否可以声明最终轨迹可行。
result = struct();
result.err.startStateNorm = norm(traj.Xnode(:, 1) - traj.xStart);
result.err.endStateNorm = norm(traj.Xnode(:, end) - traj.xEnd);
result.err.hsEqualityMax = max(abs(ceq(:)));
result.constraint.maxPathViolation = max([c(:); 0]);
result.constraint.denseMaxPathViolation = denseReport.maxPathViolation;
result.constraint.minClearance = denseReport.minClearance;
result.constraint.minSigmaMin = denseReport.minSigmaMin;
result.constraint.maxCondJ = denseReport.maxCondJ;
result.constraint.maxAbsForce = denseReport.maxAbsForce;
result.constraint.minLength = denseReport.minLength;
result.constraint.maxLength = denseReport.maxLength;
result.constraint.maxAbsLd = denseReport.maxAbsLd;
result.constraint.maxAbsLdd = denseReport.maxAbsLdd;
result.successFlag = result.err.startStateNorm <= 1e-6 && ...
    result.err.endStateNorm <= 1e-6 && ...
    result.err.hsEqualityMax <= 1e-6 && ...
    result.constraint.maxPathViolation <= 1e-6 && ...
    denseReport.isFeasible;
result.solver = solverResult;

fprintf('\n===== HS 完整 NLP 结果分析 =====\n');
fprintf('起点状态误差：%.3e\n', result.err.startStateNorm);
fprintf('终点状态误差：%.3e\n', result.err.endStateNorm);
fprintf('HS 等式最大残差：%.3e\n', result.err.hsEqualityMax);
fprintf('非线性路径最大正违反量：%.3e\n', result.constraint.maxPathViolation);
fprintf('dense 联合最大路径违反量：%.3e\n', result.constraint.denseMaxPathViolation);
fprintf('最小碰撞间隙 [m]：%.6f\n', result.constraint.minClearance);
fprintf('最小 sigmaMin：%.6f\n', result.constraint.minSigmaMin);
fprintf('最大 condJ：%.6f\n', result.constraint.maxCondJ);
fprintf('最大绝对驱动力 [N]：%.3f\n', result.constraint.maxAbsForce);
fprintf('腿长范围 [m]：%.6f 到 %.6f\n', result.constraint.minLength, result.constraint.maxLength);
fprintf('最大 |腿速| [m/s]：%.6f\n', result.constraint.maxAbsLd);
fprintf('最大 |腿加速度| [m/s^2]：%.6f\n', result.constraint.maxAbsLdd);
fprintf('综合成功标志：%d\n', result.successFlag);
end
