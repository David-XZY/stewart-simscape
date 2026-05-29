%% test_03_fmincon_adapter_consistency - fmincon 适配器一致性测试
% 文件用途：
%   验证 buildFminconSQPAdapterFromCasadi 直接求解的确是当前 CasADi 隐式 HS NLP：
%   目标、等式、不等式与 nlpData.eval 完全一致，并用中心差分检查目标梯度和约束雅可比。
%
% 输入：
%   可选环境变量 HS_NUM_INTERVALS_TEST03；未设置时使用 10 区间以缩短单元测试时间。
%
% 输出：
%   控制台打印 unscaled/scaled 两种模式下的函数值误差和方向导数相对误差。
%
% 求解链路位置：
%   本测试不调用 fmincon 求解，只验证 SQP adapter 与 IPOPT 使用同一个 J/gEq/cIneq。

clear; clc;
solverComparisonRoot = fileparts(mfilename('fullpath'));
optRoot = fileparts(solverComparisonRoot);
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(solverComparisonRoot);
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildPreAlignmentScene(model);
disc = buildTestDiscretization(scene);
[z0, ~] = buildInitialGuessQuinticHSImplicit(model, scene, disc);
nlpData = buildCasadiImplicitHSNLP(model, scene, disc);
[Jref, gEqRef, cRef] = nlpData.eval(z0);
Jref = full(Jref);
gEqRef = full(gEqRef);
cRef = full(cRef);

[scale, scaleSummary] = buildImplicitHSVariableScale(z0, model, scene, disc); %#ok<ASGLU>
runOneAdapterCase('unscaled', nlpData, z0, Jref, gEqRef, cRef, false, ones(numel(z0), 1));
runOneAdapterCase('scaled', nlpData, z0, Jref, gEqRef, cRef, true, scale);
fprintf('\nTEST_03_OK\n');

function runOneAdapterCase(caseName, nlpData, z0, Jref, gEqRef, cRef, useScaling, scale)
scalingConfig = struct('useVariableScaling', useScaling, 'scale', scale);
adapter = buildFminconSQPAdapterFromCasadi(nlpData, scalingConfig);
y0 = adapter.toSolver(z0);
[fAdapter, gradAdapter] = adapter.objectiveFcn(y0);
[cAdapter, ceqAdapter, GC, GCeq] = adapter.nonlconFcn(y0);

objectiveDiff = abs(fAdapter - Jref);
eqDiff = max(abs(ceqAdapter - gEqRef));
ineqDiff = max(abs(cAdapter - cRef));
fprintf('\nTEST_03 %s function consistency\n', caseName);
fprintf('objective difference = %.3e\n', objectiveDiff);
fprintf('equality max difference = %.3e\n', eqDiff);
fprintf('inequality max difference = %.3e\n', ineqDiff);
assert(objectiveDiff <= 1e-10, '%s objective mismatch.', caseName);
assert(eqDiff <= 1e-10, '%s equality mismatch.', caseName);
assert(ineqDiff <= 1e-10, '%s inequality mismatch.', caseName);

rng(7);
numDirections = 3;
stepSize = 1e-6;
eqIndices = unique(round(linspace(1, nlpData.sizes.numEq, min(8, nlpData.sizes.numEq))));
ineqIndices = 1:min(36, nlpData.sizes.numIneq);
maxObjRelErr = 0;
maxEqRelErr = 0;
maxIneqRelErr = 0;
for directionIndex = 1:numDirections
    direction = randn(numel(y0), 1);
    direction = direction / norm(direction);
    yPlus = y0 + stepSize * direction;
    yMinus = y0 - stepSize * direction;

    fPlus = adapter.objectiveFcn(yPlus);
    fMinus = adapter.objectiveFcn(yMinus);
    fdObj = (fPlus - fMinus) / (2*stepSize);
    adObj = gradAdapter(:).' * direction;
    maxObjRelErr = max(maxObjRelErr, relativeError(fdObj, adObj));

    [cPlus, ceqPlus] = adapter.nonlconFcn(yPlus);
    [cMinus, ceqMinus] = adapter.nonlconFcn(yMinus);
    fdEq = (ceqPlus(eqIndices) - ceqMinus(eqIndices)) / (2*stepSize);
    adEq = GCeq(:, eqIndices).' * direction;
    fdIneq = (cPlus(ineqIndices) - cMinus(ineqIndices)) / (2*stepSize);
    adIneq = GC(:, ineqIndices).' * direction;
    maxEqRelErr = max(maxEqRelErr, max(relativeError(fdEq, adEq)));
    maxIneqRelErr = max(maxIneqRelErr, max(relativeError(fdIneq, adIneq)));
end
fprintf('objective directional derivative max relative error = %.3e\n', maxObjRelErr);
fprintf('equality Jacobian directional max relative error = %.3e\n', maxEqRelErr);
fprintf('inequality Jacobian directional max relative error = %.3e\n', maxIneqRelErr);
assert(maxObjRelErr <= 1e-4, '%s objective gradient directional check failed.', caseName);
assert(maxEqRelErr <= 1e-4, '%s equality Jacobian directional check failed.', caseName);
assert(maxIneqRelErr <= 1e-4, '%s inequality Jacobian directional check failed.', caseName);
end

function disc = buildTestDiscretization(scene)
numIntervals = str2double(getenv('HS_NUM_INTERVALS_TEST03'));
if ~isfinite(numIntervals) || numIntervals <= 0
    numIntervals = 10;
end
disc = struct();
disc.numIntervals = round(numIntervals);
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.duration = scene.preAlign.duration;
disc.h = disc.duration / disc.numIntervals;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
end

function err = relativeError(a, b)
err = abs(a - b) ./ max(1, max(abs(a), abs(b)));
end
