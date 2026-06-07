%% test_04_implicit_solve_consistency - 消去加速度动力学一致性测试
% 用途：
%   验证 CasADi 显式求值动力学 fSolved(X,F) 仍满足原隐式平衡方程
%   Jv(q).'*F - Wreq(q,qd,A)=0，并与数值 stateDynamicsCompositeRigidBody 一致。
clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
dynFun = buildCasadiImplicitSolvedDynamics(model);
disc = buildLocalDisc(scene);
[z0, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc); %#ok<ASGLU>
data = evaluateImplicitTrajectoryNumeric(z0, model, scene, disc);

sampleIndices = round(linspace(1, disc.numNodes, 20));
maxDynResidual = 0;
maxQddError = 0;
for i = 1:numel(sampleIndices)
    nodeIndex = sampleIndices(i);
    X = data.Xnode(:, nodeIndex);
    F = data.Fnode(:, nodeIndex);
    [xdotCas, ACas, rDynCas] = dynFun(X, F);
    [xdotNum, auxNum] = stateDynamicsCompositeRigidBody(X, F, model);
    maxDynResidual = max(maxDynResidual, max(abs(full(rDynCas))));
    maxQddError = max(maxQddError, max(abs(full(ACas) - auxNum.qdd)));
    maxQddError = max(maxQddError, max(abs(full(xdotCas) - xdotNum)));
end

fprintf('TEST_04 implicit solved dynamics consistency\n');
fprintf('max rDyn = %.3e, max qdd/xdot error = %.3e\n', maxDynResidual, maxQddError);
assert(maxDynResidual <= 1e-9, '消去加速度后的 CasADi 动力学残差过大。');
assert(maxQddError <= 1e-9, 'CasADi 显式动力学与数值动力学不一致。');
fprintf('TEST_04_OK\n');

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
