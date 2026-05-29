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
%   result struct - 统一诊断结构，区分 NLP、state 轨迹和工程几何轨迹验收。
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
result.constraint.forceUpperViolationMax = denseReport.forceUpperViolationMax;
result.constraint.forceLowerViolationMax = denseReport.forceLowerViolationMax;
result.constraint.minLength = denseReport.minLength;
result.constraint.maxLength = denseReport.maxLength;
result.constraint.maxAbsLd = denseReport.maxAbsLd;
result.constraint.maxAbsLdd = denseReport.maxAbsLdd;
solverReportedSuccess = ~isstruct(solverResult) || ~isfield(solverResult, 'success') || solverResult.success;
result.nlpPassed = solverReportedSuccess && ...
    result.err.startStateNorm <= 1e-6 && ...
    result.err.endStateNorm <= 1e-6 && ...
    result.err.hsEqualityMax <= 1e-6 && ...
    result.constraint.maxPathViolation <= 1e-6;
result.postCheck.pathPassed = denseReport.pathPassed;
result.postCheck.forcePassed = denseReport.forcePassed;
result.postCheck.singularityPassed = denseReport.singularityPassed;
result.postCheck.kinematicsPassed = denseReport.kinematicsPassed;
result.postCheck.stateDynamicsPassed = denseReport.stateDynamicsPassed;
result.postCheck.geometricDynamicsPassed = denseReport.geometricDynamicsPassed;
result.solverTrajectoryPassed = result.nlpPassed && denseReport.solverTrajectoryPassed;
result.engineeringTrajectoryPassed = result.nlpPassed && denseReport.engineeringTrajectoryPassed;
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
fprintf('驱动力上界最大违反量：%.3e\n', result.constraint.forceUpperViolationMax);
fprintf('驱动力下界最大违反量：%.3e\n', result.constraint.forceLowerViolationMax);
fprintf('腿长范围 [m]：%.6f 到 %.6f\n', result.constraint.minLength, result.constraint.maxLength);
fprintf('最大 |腿速| [m/s]：%.6f\n', result.constraint.maxAbsLd);
fprintf('最大 |腿加速度| [m/s^2]：%.6f\n', result.constraint.maxAbsLdd);
fprintf('NLP 通过：%d\n', result.nlpPassed);
fprintf('state 轨迹通过：%d\n', result.solverTrajectoryPassed);
fprintf('工程几何轨迹通过：%d\n', result.engineeringTrajectoryPassed);
end
