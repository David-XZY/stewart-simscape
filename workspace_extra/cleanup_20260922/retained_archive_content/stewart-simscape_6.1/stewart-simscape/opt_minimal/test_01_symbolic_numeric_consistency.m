%% test_01_symbolic_numeric_consistency - 两阶段符号公式与数值公式一致性检查
% 文件用途：
%   构建圆柱体-长方体两阶段隐式 HS NLP，在五次初值上比较 CasADi MX 生成的
%   目标、等式、不等式与 MATLAB 数值重建结果。
%
% 输入：无。
% 输出：控制台打印误差摘要，超限则报错。
clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildTestDiscretization(scene);
assert(max(abs(scene.box.rpy - deg2rad([5; 20; 45]))) <= 1e-12, '长方体姿态未更新为 [5,20,45] deg。');
assert(isfield(scene, 'hood') && isfield(scene.hood, 'obstacles') && numel(scene.hood.obstacles) == 3, ...
    '场景必须包含 roof/leftSkirt/rightSkirt 三个罩体 OBB。');
assert(disc.numCollisionCertificates == 323, '默认离散下分离证书数量应为 323。');

[z0, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
nlpData = buildCasadiImplicitHSNLP(model, scene, disc, initialGuess);
[Jcas, gEqCas, cCas] = nlpData.eval(z0);
Jcas = full(Jcas);
gEqCas = full(gEqCas);
cCas = full(cCas);

data = evaluateImplicitTrajectoryNumeric(z0, model, scene, disc);
[Jnum, gEqNum, cNum] = numericObjectiveAndConstraints(data, model, scene, disc, initialGuess);

errObjective = abs(Jcas - Jnum);
errEq = max(abs(gEqCas - gEqNum));
errPath = max(abs(cCas - cNum));
dynamicStart = 12*disc.numIntervals + 1;
dynamicEnd = dynamicStart + 6*(disc.numNodes + disc.numMidpoints) - 1;
errDyn = max(abs(gEqCas(dynamicStart:dynamicEnd) - gEqNum(dynamicStart:dynamicEnd)));

fprintf('TEST_01 cylinder-box two-phase symbolic/numeric consistency\n');
fprintf('objective error = %.3e\n', errObjective);
fprintf('HS + dynamics + stage equality max error = %.3e\n', errEq);
fprintf('dynamic Wreq/Jv residual block max error = %.3e\n', errDyn);
fprintf('path block max error (L/Ld/Ldd/clearance/monotonic) = %.3e\n', errPath);
fprintf('sizes z=%d eq=%d ineq=%d\n', nlpData.sizes.numZ, nlpData.sizes.numEq, nlpData.sizes.numIneq);
fprintf('collision certificates=%d separator variables=%d\n', ...
    disc.numCollisionCertificates, 8*disc.numCollisionCertificates);

assert(errObjective <= 1e-8, '目标函数符号/数值不一致。');
assert(errEq <= 1e-8, '等式约束符号/数值不一致。');
assert(errPath <= 1e-8, '路径约束符号/数值不一致。');
assert(nlpData.sizes.numZ == 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + ...
    6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates, 'NLP 决策变量规模未由三构件证书驱动。');
fprintf('TEST_01_OK\n');

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

function [J, gEq, c] = numericObjectiveAndConstraints(data, model, scene, disc, initialGuess)
breakdown = computeObjectiveBreakdownImplicit(data, model, scene, disc, initialGuess);
J = breakdown.total;
gHS = zeros(12*disc.numIntervals, 1);
for intervalIndex = 1:disc.numIntervals
    block = (intervalIndex - 1)*12 + (1:12);
    gHS(block) = data.Xnode(:, intervalIndex+1) - data.Xnode(:, intervalIndex) - ...
        disc.h/6*(data.fNode(:, intervalIndex) + 4*data.fMid(:, intervalIndex) + data.fNode(:, intervalIndex+1));
end

gDyn = [];
gWaypoint = data.Xnode(1:6, disc.waypointNodeIndex) - scene.qWaypoint;
gStage2 = [];
c = [];
for nodeIndex = 1:disc.numNodes
    gDyn = [gDyn; data.nodeDynResidual(:, nodeIndex)]; %#ok<AGROW>
    c = [c; data.nodePoint{nodeIndex}.cPath(1:36)]; %#ok<AGROW>
    if nodeIndex <= disc.numIntervalsApproach + 1
        sepIndex = nodeIndex;
        [gSep, cSep] = separatorNumeric(data.Xnode(1:6, nodeIndex), data.separator(:, sepIndex), ...
            scene.hood.roof, scene.collision.stage1ConstraintDistance, scene);
        gDyn = [gDyn; gSep]; %#ok<AGROW>
        c = [c; cSep]; %#ok<AGROW>
    end
    allPointIndex = nodeIndex;
    leftIndex = disc.numStage1CollisionPoints + allPointIndex;
    rightIndex = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + allPointIndex;
    [gLeft, cLeft] = separatorNumeric(data.Xnode(1:6, nodeIndex), data.separator(:, leftIndex), ...
        scene.hood.leftSkirt, scene.collision.safeDistance, scene);
    [gRight, cRight] = separatorNumeric(data.Xnode(1:6, nodeIndex), data.separator(:, rightIndex), ...
        scene.hood.rightSkirt, scene.collision.safeDistance, scene);
    gDyn = [gDyn; gLeft; gRight]; %#ok<AGROW>
    c = [c; cLeft; cRight]; %#ok<AGROW>
    if nodeIndex >= disc.waypointNodeIndex
        [gLine, cMono] = insertionNumeric(data.Xnode(:, nodeIndex), scene);
        gStage2 = [gStage2; gLine]; %#ok<AGROW>
        c = [c; cMono]; %#ok<AGROW>
    end
end
for midIndex = 1:disc.numMidpoints
    gDyn = [gDyn; data.midDynResidual(:, midIndex)]; %#ok<AGROW>
    c = [c; data.midPoint{midIndex}.cPath(1:36)]; %#ok<AGROW>
    if midIndex <= disc.numIntervalsApproach
        sepIndex = disc.numIntervalsApproach + 1 + midIndex;
        [gSep, cSep] = separatorNumeric(data.Xmid(1:6, midIndex), data.separator(:, sepIndex), ...
            scene.hood.roof, scene.collision.stage1ConstraintDistance, scene);
        gDyn = [gDyn; gSep]; %#ok<AGROW>
        c = [c; cSep]; %#ok<AGROW>
    end
    allPointIndex = disc.numNodes + midIndex;
    leftIndex = disc.numStage1CollisionPoints + allPointIndex;
    rightIndex = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + allPointIndex;
    [gLeft, cLeft] = separatorNumeric(data.Xmid(1:6, midIndex), data.separator(:, leftIndex), ...
        scene.hood.leftSkirt, scene.collision.safeDistance, scene);
    [gRight, cRight] = separatorNumeric(data.Xmid(1:6, midIndex), data.separator(:, rightIndex), ...
        scene.hood.rightSkirt, scene.collision.safeDistance, scene);
    gDyn = [gDyn; gLeft; gRight]; %#ok<AGROW>
    c = [c; cLeft; cRight]; %#ok<AGROW>
    if midIndex >= disc.numIntervalsApproach + 1
        [gLine, cMono] = insertionNumeric(data.Xmid(:, midIndex), scene);
        gStage2 = [gStage2; gLine]; %#ok<AGROW>
        c = [c; cMono]; %#ok<AGROW>
    end
end
gEq = [gHS; gDyn; gWaypoint; gStage2];
end

function [gSep, cSep] = separatorNumeric(q, sep, obstacle, requiredDistance, scene)
n = sep(1:3);
eta = sep(4:6);
zeta = sep(7);
rho = sep(8);
R = rpy2rotmZYX(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
axisS = R * scene.objectCylinder.axis_P;
PperpN = n - axisS * (axisS.' * n);
gap = n.' * (obstacle.center_S - centerS) - obstacle.halfSize.' * eta - ...
    0.5 * scene.objectCylinder.length * zeta - scene.objectCylinder.radius * rho;
gSep = n.' * n - 1;
cSep = [n.'*obstacle.R_S(:,1)-eta(1);
        -n.'*obstacle.R_S(:,1)-eta(1);
        n.'*obstacle.R_S(:,2)-eta(2);
        -n.'*obstacle.R_S(:,2)-eta(2);
        n.'*obstacle.R_S(:,3)-eta(3);
        -n.'*obstacle.R_S(:,3)-eta(3);
        n.'*axisS-zeta;
        -n.'*axisS-zeta;
        sqrt(PperpN.'*PperpN + scene.collision.smoothingEps^2)-rho;
        requiredDistance-gap];
end

function [gLine, cMono] = insertionNumeric(X, scene)
q = X(1:6);
qd = X(7:12);
clearance = evaluateCylinderBoxClearance(q, scene);
lineHeight = insertionLineHeightNumeric(clearance.pCylinder_B(1), scene);
gLine = [clearance.pCylinder_B(2);
         clearance.pCylinder_B(3) - lineHeight;
         q(4:6) - scene.box.rpy];
R = rpy2rotmZYX(q(4:6));
omega = rpyRateMapZYX(q(4:6)) * qd(4:6);
pCd_S = qd(1:3) + cross(omega, R * scene.objectCylinder.center_P);
pCd_B = scene.box.R_S.' * pCd_S;
cMono = -pCd_B(1);
end

function z = insertionLineHeightNumeric(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end
