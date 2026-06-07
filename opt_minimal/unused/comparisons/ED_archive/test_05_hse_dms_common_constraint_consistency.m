%% test_05_hse_dms_common_constraint_consistency - CHSED/DMSED reduced 结构一致性
% 用途：
%   检查消去加速度后的 CHSED 与 DMSED 使用相同 reduced 决策变量、相同证书数量、
%   相同节点路径约束定义，并能完成初值符号求值。
clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene);
disc = resizeDiscForTest(disc, scene, 4, 2);
[z0, initialGuess] = buildInitialGuessTwoPhaseReduced(model, scene, disc);

hse = buildCasadiEliminatedAccelHSNLP(model, scene, disc, initialGuess);
dms = buildCasadiMultipleShootingNLP(model, scene, disc, initialGuess);
[~, gEqHSE, cHSE] = hse.eval(z0);
[~, gEqDMS, cDMS] = dms.eval(z0);

fprintf('TEST_05 CHSED/DMSED common reduced structure\n');
fprintf('CHSED sizes z=%d eq=%d ineq=%d\n', hse.sizes.numZ, hse.sizes.numEq, hse.sizes.numIneq);
fprintf('DMSED sizes z=%d eq=%d ineq=%d\n', dms.sizes.numZ, dms.sizes.numEq, dms.sizes.numIneq);
fprintf('initial maxEq CHSED/DMSED = %.3e / %.3e\n', max(abs(full(gEqHSE))), max(abs(full(gEqDMS))));
fprintf('initial maxIneq CHSED/DMSED = %.3e / %.3e\n', max([full(cHSE);0]), max([full(cDMS);0]));

expectedZ = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates;
assert(hse.sizes.numZ == expectedZ && dms.sizes.numZ == expectedZ, 'reduced 决策变量规模不一致。');
assert(hse.sizes.numIneq == dms.sizes.numIneq, 'CHSED 与 DMSED 不等式数量必须一致。');
assert(hse.sizes.numEq == dms.sizes.numEq, 'CHSED 与 DMSED 射击/缺陷等式数量必须一致。');
fprintf('TEST_05_OK\n');

function disc = buildLocalDisc(scene)
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

function disc = resizeDiscForTest(disc, scene, n1, n2)
disc.numIntervalsApproach = n1;
disc.numIntervalsInsertion = n2;
disc.numIntervals = n1 + n2;
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.durationApproach = scene.phase.durationApproach;
disc.durationInsertion = scene.phase.durationInsertion;
disc.duration = disc.durationApproach + disc.durationInsertion;
disc.h = disc.durationApproach / n1;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage2NodeIndices = disc.waypointNodeIndex:disc.numNodes;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
