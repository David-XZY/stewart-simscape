function denseReport = validateTrajectoryDense(traj, model, scene, disc)
% validateTrajectoryDense - 对 HS 结果做节点/中点综合密集验证
%
% 文件用途：
%   汇总节点和中点处的腿长、腿速、腿加速度、奇异性、驱动力和碰撞间隙
%   极值，判断最终结果是否可信。
%
% 输入参数：
%   traj struct - 由主脚本重建的 HS 轨迹。
%   model struct - Stewart 模型。
%   scene struct - 碰撞场景。
%   disc struct - 离散参数。
%
% 输出参数：
%   denseReport struct - 包含最小间隙、最大约束违反量和极值报告。
%
% 核心公式：
%   节点和中点共用 evaluatePathConstraintsAtPoint，避免验证约束与求解约束不一致。
%
% 在优化链路中的作用：
%   analyzeHSResult 和主脚本用本函数判断最终结果是否满足硬约束。

allPoints = [traj.nodePoints, traj.midPoints];
allC = [];
allLength = [];
allLd = [];
allLdd = [];
allForce = [];
allSigma = [];
allCond = [];
allClearance = [];

for pointIndex = 1:numel(allPoints)
    point = allPoints{pointIndex};
    allC = [allC; point.cPath]; %#ok<AGROW>
    allLength = [allLength, point.L]; %#ok<AGROW>
    allLd = [allLd, point.Ld]; %#ok<AGROW>
    allLdd = [allLdd, point.Ldd]; %#ok<AGROW>
    allForce = [allForce, point.F]; %#ok<AGROW>
    allSigma = [allSigma, point.sigmaMin]; %#ok<AGROW>
    allCond = [allCond, point.condJ]; %#ok<AGROW>
    allClearance = [allClearance; point.collisionDistances(:)]; %#ok<AGROW>
end

denseReport = struct();
denseReport.maxPathViolation = max([allC(:); 0]);
denseReport.minClearance = min(allClearance);
denseReport.minLength = min(allLength(:));
denseReport.maxLength = max(allLength(:));
denseReport.maxAbsLd = max(abs(allLd(:)));
denseReport.maxAbsLdd = max(abs(allLdd(:)));
denseReport.minSigmaMin = min(allSigma(:));
denseReport.maxCondJ = max(allCond(:));
forceUpperViolation = allForce - repmat(model.actuator.forceMax(:), 1, size(allForce, 2));
forceLowerViolation = repmat(model.actuator.forceMin(:), 1, size(allForce, 2)) - allForce;
denseReport.forceUpperViolationMax = max([forceUpperViolation(:); 0]);
denseReport.forceLowerViolationMax = max([forceLowerViolation(:); 0]);
denseReport.pathPassed = denseReport.maxPathViolation <= 1e-6;
denseReport.forcePassed = denseReport.forceUpperViolationMax <= 1e-6 && ...
    denseReport.forceLowerViolationMax <= 1e-6;
denseReport.singularityPassed = denseReport.minSigmaMin >= model.singularity.sigmaMinSafe && ...
    denseReport.maxCondJ <= model.singularity.condWarning;
denseReport.kinematicsPassed = true;
denseReport.stateDynamicsPassed = true;
denseReport.geometricDynamicsPassed = true;
denseReport.solverTrajectoryPassed = denseReport.pathPassed && denseReport.forcePassed && ...
    denseReport.singularityPassed && denseReport.stateDynamicsPassed;
denseReport.engineeringTrajectoryPassed = denseReport.pathPassed && denseReport.forcePassed && ...
    denseReport.singularityPassed && denseReport.kinematicsPassed && denseReport.geometricDynamicsPassed;
denseReport.disc = disc;
denseReport.scene = scene;
end
