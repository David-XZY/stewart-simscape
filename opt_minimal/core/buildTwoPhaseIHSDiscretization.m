function disc = buildTwoPhaseIHSDiscretization(scene, numIntervalsApproach, numIntervalsInsertion)
% buildTwoPhaseIHSDiscretization - 构建两阶段 standard IHSID 离散配置
%
% 默认采用 40x20 网格。两个阶段必须使用相同步长，以保证统一的
% Hermite-Simpson 配点公式和节点时间序列。
if nargin < 2 || isempty(numIntervalsApproach)
    numIntervalsApproach = 40;
end
if nargin < 3 || isempty(numIntervalsInsertion)
    numIntervalsInsertion = 20;
end

disc = struct();
disc.numIntervalsApproach = numIntervalsApproach;
disc.numIntervalsInsertion = numIntervalsInsertion;
disc.numIntervals = disc.numIntervalsApproach + disc.numIntervalsInsertion;
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.durationApproach = scene.phase.durationApproach;
disc.durationInsertion = scene.phase.durationInsertion;
disc.duration = disc.durationApproach + disc.durationInsertion;
disc.hApproach = disc.durationApproach / disc.numIntervalsApproach;
disc.hInsertion = disc.durationInsertion / disc.numIntervalsInsertion;
if abs(disc.hApproach - disc.hInsertion) > 1e-12
    error('buildTwoPhaseIHSDiscretization:NonUniformStep', '两阶段 IHSID 必须使用统一步长。');
end
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = disc.numIntervalsApproach + 1;
disc.stage2NodeIndices = disc.waypointNodeIndex:disc.numNodes;
disc.stage2MidIndices = (disc.numIntervalsApproach + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (disc.numIntervalsApproach + 1) + disc.numIntervalsApproach;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
