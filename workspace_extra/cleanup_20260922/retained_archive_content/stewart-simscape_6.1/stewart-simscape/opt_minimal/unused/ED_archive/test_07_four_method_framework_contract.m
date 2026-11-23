%% test_07_four_method_framework_contract - 四组离散/动力学表达实验框架契约测试
% 用途：
%   在实现四方法入口前先定义应满足的最小契约：
%   1) 统一 IPOPT 选项函数存在，且四组方法共享关键设置；
%   2) 新入口支持 CHSID、CHSED、DMSID、DMSED 四个规范方法名；
%   3) DMSID 能以隐式动力学变量布局构建直接多重射击 NLP；
%   4) 四方法在同一测试网格上可以给出一致的结果表字段。
clear; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);

solverOptions = struct('maxIter', 3, 'printLevel', 0, ...
    'hessianApproximation', 'limited-memory', 'acceptableTol', 1e-3, 'acceptableIter', 1);
opts = makeCommonIpoptOptions(solverOptions);
assert(strcmp(opts.ipopt.linear_solver, 'ma27'), '统一 IPOPT 线性求解器必须为 ma27。');
assert(strcmp(opts.ipopt.hessian_approximation, 'limited-memory'), '四组方法必须共享 Hessian 近似设置。');
assert(opts.ipopt.max_iter == 3, '统一 IPOPT max_iter 未正确透传。');

[zImplicit, implicitGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc); %#ok<ASGLU>
[zReduced, reducedGuess] = buildInitialGuessTwoPhaseReduced(model, scene, disc); %#ok<ASGLU>

chsid = buildCasadiImplicitHSNLP(model, scene, disc, implicitGuess, solverOptions);
chsed = buildCasadiEliminatedAccelHSNLP(model, scene, disc, reducedGuess, solverOptions);
dmsid = buildCasadiImplicitDMSNLP(model, scene, disc, implicitGuess, solverOptions);
dmsed = buildCasadiMultipleShootingNLP(model, scene, disc, reducedGuess, solverOptions);

assert(strcmp(chsid.method, 'CHSID'), 'CHSID 构建器必须返回规范方法名。');
assert(strcmp(chsed.method, 'CHSED'), 'CHSED 构建器必须返回规范方法名。');
assert(strcmp(dmsid.method, 'DMSID'), 'DMSID 构建器必须返回规范方法名。');
assert(strcmp(dmsed.method, 'DMSED'), 'DMSED 构建器必须返回规范方法名。');
assert(dmsid.sizes.numZ == chsid.sizes.numZ, 'DMSID 与 CHSID 都是 ID 表达，变量布局规模应一致。');
assert(dmsid.sizes.numIneq == chsid.sizes.numIneq, 'DMSID 与 CHSID 路径/碰撞不等式数量应一致。');

result = run_03_compare_CHSID_CHSED_DMSID_DMSED('gridList', [2 1], ...
    'methods', {'CHSID','CHSED','DMSID','DMSED'}, ...
    'makePlots', false, 'makeAnimation', false, 'maxIter', 3, 'printLevel', 0);
methodNames = string(result.summaryTable.methodName);
assert(isequal(sort(methodNames), sort(["CHSID";"CHSED";"DMSID";"DMSED"])), ...
    '四方法入口必须输出 CHSID、CHSED、DMSID、DMSED 四行。');
requiredColumns = ["methodName","gridApproach","gridInsertion","numVariables","numEq","numIneq", ...
    "ipoptStatus","successFlag","solveTime","iterCount","objective","maxEqResidual", ...
    "maxIneqViolation","minStage1Gap","minDenseGap","maxLegSpeedViolation", ...
    "maxLegAccelViolation","maxForceViolation","minSigma","maxStage2LateralError", ...
    "maxStage2HeightError","maxStage2AttitudeError","engineeringPassed"];
assert(all(ismember(requiredColumns, string(result.summaryTable.Properties.VariableNames))), ...
    '主汇总表缺少四方法实验要求的字段。');
fprintf('TEST_07_OK\n');

function disc = buildLocalDisc(scene, n1, n2)
disc = struct();
disc.numIntervalsApproach = n1;
disc.numIntervalsInsertion = n2;
disc.numIntervals = n1 + n2;
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.durationApproach = scene.phase.durationApproach;
disc.durationInsertion = scene.phase.durationInsertion;
disc.duration = disc.durationApproach + disc.durationInsertion;
disc.hApproach = disc.durationApproach / n1;
disc.hInsertion = disc.durationInsertion / n2;
assert(abs(disc.hApproach - disc.hInsertion) < 1e-12, '测试网格必须保持统一步长。');
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage2NodeIndices = disc.waypointNodeIndex:disc.numNodes;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
