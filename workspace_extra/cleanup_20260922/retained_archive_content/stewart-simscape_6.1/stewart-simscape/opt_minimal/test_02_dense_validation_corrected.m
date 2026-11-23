%% test_02_dense_validation_corrected - 两阶段初值密集验证检查
% 用途：
%   构建圆柱体-长方体两阶段五次初值，不重新优化，验证 dense 后验函数可覆盖
%   r_kin、r_acc、r_dyn,state、r_dyn,geom 以及阶段 2 几何指标。
%
% 输入：
% 输出：
%   控制台打印初值密集验证摘要。

clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildTestDiscretization(scene);
assert(numel(scene.hood.obstacles) == 3, '密集验证必须使用三个罩体构件。');
assert(disc.numCollisionCertificates == 323, '默认离散下分离证书数量应为 323。');
[z0, ~] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
data = evaluateImplicitTrajectoryNumeric(z0, model, scene, disc);
traj = struct('t', disc.tNode, 'tc', disc.tMid, 'Xnode', data.Xnode, 'Xmid', data.Xmid, ...
    'Anode', data.Anode, 'Amid', data.Amid, 'Unode', data.Fnode, 'Umid', data.Fmid, ...
    'Q', data.Xnode(1:6, :), 'V', data.Xnode(7:12, :));
report = validateTrajectoryDenseImplicit(traj, model, scene, disc);
fprintf('\nTEST_02 cylinder-box two-phase dense validation\n');
fprintf('intervals=%d samples=%d\n', disc.numIntervals, report.sampleCount);
fprintf('max r_kin=%.6e at t=%.6f\n', report.maxKinematicResidual, report.maxKinematicResidualTime);
fprintf('max r_acc=%.6e at t=%.6f\n', report.maxAccelConsistencyResidual, report.maxAccelConsistencyResidualTime);
fprintf('max r_dyn_state=%.6e at t=%.6f\n', report.maxDynResidualState, report.maxDynResidualStateTime);
fprintf('max r_dyn_geom=%.6e at t=%.6f\n', report.maxDynResidualGeometric, report.maxDynResidualGeometricTime);
fprintf('min clearance=%.6e, stage1 min clearance=%.6e, final gap=%.6e\n', ...
    report.minClearance, report.minStage1Clearance, report.finalGap);
fprintf('roof/left/right min clearance=[%.6e %.6e %.6e]\n', report.minObstacleClearance);
fprintf('stage2 side min clearance=[%.6e %.6e]\n', report.stage2SideMinClearance);
fprintf('stage2 lateral=%.6e height=%.6e attitude=%.6e min xdot_B=%.6e\n', ...
    report.stage2MaxLateralError, report.stage2MaxHeightError, ...
    report.stage2MaxAttitudeError, report.stage2MinInsertionSpeed);
fprintf('pathPassed=%d stage2Passed=%d\n', report.pathPassed, report.stage2Passed);
assert(abs(report.finalGap - scene.collision.finalGap) <= 1e-10, '终点顶板间隙必须等于 finalGap。');
assert(all(report.stage2SideMinClearance >= scene.collision.safeDistance - 1e-8), ...
    '第二阶段左右裙板密集间隙必须满足 safeDistance。');
fprintf('\nTEST_02_DONE\n');

function disc = buildTestDiscretization(scene)
disc = struct();
disc.numIntervalsApproach = scene.phase.numIntervalsApproach;
disc.numIntervalsInsertion = scene.phase.numIntervalsInsertion;
disc.numIntervals = disc.numIntervalsApproach + disc.numIntervalsInsertion;
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.durationApproach = scene.phase.durationApproach;
disc.durationInsertion = scene.phase.durationInsertion;
disc.duration = disc.durationApproach + disc.durationInsertion;
disc.h = disc.durationApproach / disc.numIntervalsApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = disc.numIntervalsApproach + 1;
disc.stage2NodeIndices = disc.waypointNodeIndex:disc.numNodes;
disc.stage2MidIndices = (disc.numIntervalsApproach + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (disc.numIntervalsApproach + 1) + disc.numIntervalsApproach;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
