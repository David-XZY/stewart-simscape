%% run_02_compare_ipopt_sqp - 对比 IPOPT/MA27 与 fmincon-SQP 求解同一隐式 HS NLP
% 文件用途：
%   使用同一个 model、scene、disc、z0、J、gEq、cIneq、lbz、ubz，公平比较当前
%   CasADi/IPOPT/MA27 链路与新增 fmincon-SQP 链路。SQP 默认使用 CasADi 自动微分
%   一阶导数和变量尺度处理。
%
% 输入：
%   无直接输入。可用环境变量 HS_NUM_INTERVALS、SQP_MAX_ITER、SQP_MAX_FUN_EVALS
%   临时控制离散规模与 SQP 预算。
%
% 输出：
%   opt_minimal/results 下的 compare_ipopt_sqp_*.mat、summary_compare_ipopt_sqp_*.txt
%   和 solver_* 对比图。
%
% 求解链路位置：
%   本脚本是求解器性能研究入口，不替代 run_01_hs_dynamic_opt.m。
%
% 是否改变数学问题：
%   否。两个求解器使用同一个 CasADi NLP 表达式和边界。

clear; close all; clc;

solverComparisonRoot = fileparts(mfilename('fullpath'));
optRoot = fileparts(solverComparisonRoot);
projectRoot = fileparts(optRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(optRoot);
addpath(solverComparisonRoot);

resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
diaryFile = fullfile(resultDir, ['console_compare_ipopt_sqp_', timestamp, '.txt']);
diary(diaryFile);
diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('\n===== IPOPT/MA27 vs fmincon 多求解器同一 NLP 对比 =====\n');
model = buildOptModelCustom();
scene = buildPreAlignmentScene(model);
disc = buildHSDiscretization(scene);
fprintf('numIntervals=%d\n', disc.numIntervals);

ma27Info = setupCasadiIpoptMa27(projectRoot);
[z0, initialGuess] = buildInitialGuessQuinticHSImplicit(model, scene, disc); %#ok<NASGU>
nlpData = buildCasadiImplicitHSNLP(model, scene, disc);
fprintf('NLP size: z=%d eq=%d ineq=%d\n', nlpData.sizes.numZ, nlpData.sizes.numEq, nlpData.sizes.numIneq);

fprintf('\n===== Solver A: IPOPT + MA27 =====\n');
ipoptEntry = solveWithIpopt(nlpData, z0, model, scene, disc, ma27Info);

fprintf('\n===== Solver B: fmincon + SQP + CasADi gradients + scaling =====\n');
sqpOptions = buildSQPOptions();
sqpRaw = solveImplicitHSSQP(nlpData, z0, model, scene, disc, sqpOptions);
sqpEntry = buildEntryFromSQP(sqpRaw, model, scene, disc);

fprintf('\n===== Solver C: fmincon + interior-point + CasADi gradients/Hessian + scaling =====\n');
interiorPointOptions = buildInteriorPointOptions();
interiorPointRaw = solveImplicitHSSQP(nlpData, z0, model, scene, disc, interiorPointOptions);
interiorPointEntry = buildEntryFromSQP(interiorPointRaw, model, scene, disc);

solverEntries = [ipoptEntry, sqpEntry, interiorPointEntry];
problemInfo = struct('numIntervals', disc.numIntervals, 'numZ', nlpData.sizes.numZ, ...
    'numEq', nlpData.sizes.numEq, 'numIneq', nlpData.sizes.numIneq, ...
    'solverCount', numel(solverEntries));
comparison = compareSolverResults(solverEntries, resultDir, timestamp, problemInfo);

resultFile = fullfile(resultDir, ['compare_ipopt_sqp_', timestamp, '.mat']);
nlpSizes = nlpData.sizes;
save(resultFile, 'model', 'scene', 'disc', 'z0', 'nlpSizes', 'ma27Info', ...
    'ipoptEntry', 'sqpEntry', 'interiorPointEntry', 'comparison', ...
    'sqpOptions', 'interiorPointOptions');

fprintf('\n===== 对比结果已保存 =====\n');
fprintf('MAT: %s\n', resultFile);
fprintf('summary: %s\n', comparison.summaryFile);
fprintf('console: %s\n', diaryFile);

function disc = buildHSDiscretization(scene)
% buildHSDiscretization - 构建对比实验 HS 离散参数，默认 40 区间
numIntervals = str2double(getenv('HS_NUM_INTERVALS'));
if ~isfinite(numIntervals) || numIntervals <= 0
    numIntervals = 40;
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

function sqpOptions = buildSQPOptions()
% buildSQPOptions - 构建默认 exactGradientScaledSQP 配置
sqpOptions = struct();
sqpOptions.useExactGradient = true;
sqpOptions.useVariableScaling = true;
sqpOptions.Display = 'iter';
sqpOptions.ConstraintTolerance = 1e-6;
sqpOptions.OptimalityTolerance = 1e-6;
sqpOptions.StepTolerance = 1e-10;
sqpOptions.MaxIterations = readPositiveEnv('SQP_MAX_ITER', 20);
sqpOptions.MaxFunctionEvaluations = readPositiveEnv('SQP_MAX_FUN_EVALS', 200000);
sqpOptions.rawFiniteDifferenceSQPEnabled = false;
sqpOptions.exactGradientUnscaledSQPEnabled = false;
end

function interiorPointOptions = buildInteriorPointOptions()
% buildInteriorPointOptions - configure Solver C for the shared CasADi NLP.
interiorPointOptions = struct();
interiorPointOptions.Algorithm = 'interior-point';
interiorPointOptions.useExactGradient = true;
interiorPointOptions.useExactHessian = true;
interiorPointOptions.useVariableScaling = true;
interiorPointOptions.Display = 'iter';
interiorPointOptions.ConstraintTolerance = 1e-6;
interiorPointOptions.OptimalityTolerance = 1e-6;
interiorPointOptions.StepTolerance = 1e-10;
interiorPointOptions.MaxIterations = readPositiveEnv('IP_MAX_ITER', 40);
interiorPointOptions.MaxFunctionEvaluations = readPositiveEnv('IP_MAX_FUN_EVALS', 200000);
end

function value = readPositiveEnv(name, defaultValue)
value = str2double(getenv(name));
if ~isfinite(value) || value <= 0
    value = defaultValue;
end
value = round(value);
end

function entry = solveWithIpopt(nlpData, z0, model, scene, disc, ma27Info)
solveTimer = tic;
ipoptConsole = evalc("sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, " + ...
    "'lbg', nlpData.lbg, 'ubg', nlpData.ubg);");
solveTime = toc(solveTimer);
fprintf('%s', ipoptConsole);
stats = nlpData.solver.stats();
zOpt = full(sol.x);
fval = full(sol.f);
gOpt = full(sol.g);
gEq = gOpt(1:nlpData.sizes.numEq);
cIneq = gOpt(nlpData.sizes.numEq+1:end);

traj = rebuildImplicitTrajectoryFromDecision(zOpt, model, scene, disc);
denseReport = validateTrajectoryDenseImplicit(traj, model, scene, disc);
objectiveBreakdown = computeObjectiveBreakdownImplicit(traj, model, disc);
solverResult = struct();
solverResult.solveTime = solveTime;
solverResult.iterations = stats.iter_count;
solverResult.functionCount = readStatsField(stats, 'n_call_nlp_f', NaN);
solverResult.return_status = char(stats.return_status);
solverResult.exitflag = NaN;
solverResult.fval = fval;
solverResult.maxEqResidual = max(abs(gEq(:)));
solverResult.maxIneqViolation = max([cIneq(:); 0]);
solverResult.success = isfield(stats, 'success') && stats.success && ...
    solverResult.maxEqResidual <= 1e-6 && solverResult.maxIneqViolation <= 1e-6;
solverResult.stats = stats;
solverResult.ma27Info = ma27Info;
solverResult.ipoptConsole = ipoptConsole;

result = buildPostResult(traj, denseReport, solverResult, objectiveBreakdown);
entry = struct();
entry.solverName = 'IPOPT + MA27';
entry.solverResult = solverResult;
entry.traj = traj;
entry.denseReport = denseReport;
entry.objectiveBreakdown = objectiveBreakdown;
entry.result = result;
entry.iterationHistory = parseIpoptIterationHistory(ipoptConsole);
entry.usesExactGradient = true;
entry.usesVariableScaling = false;
entry.initialValueDescription = 'shared quintic implicit HS z0';
entry.statusText = solverResult.return_status;
entry.zOpt = zOpt;
end

function history = parseIpoptIterationHistory(ipoptConsole)
% parseIpoptIterationHistory - convert IPOPT console iteration table to plot data.
history = initializeSolverHistory();
lines = regexp(ipoptConsole, '\r\n|\n|\r', 'split');
for lineIndex = 1:numel(lines)
    tokens = regexp(lines{lineIndex}, ...
        '^\s*(\d+)(r?)\s+([+-]?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)\s+([+-]?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)\s+([+-]?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)\s+.*?([+-]?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)\s+([+-]?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)[A-Za-z]?\s+\d+\s*$', ...
        'tokens', 'once');
    if isempty(tokens)
        continue;
    end
    history.iteration(end+1, 1) = str2double(tokens{1}); %#ok<AGROW>
    history.fval(end+1, 1) = str2double(tokens{3}); %#ok<AGROW>
    history.maxEqResidual(end+1, 1) = str2double(tokens{4}); %#ok<AGROW>
    history.maxIneqViolation(end+1, 1) = NaN; %#ok<AGROW>
    history.maxConstraintViolation(end+1, 1) = str2double(tokens{4}); %#ok<AGROW>
    history.firstOrderOpt(end+1, 1) = str2double(tokens{5}); %#ok<AGROW>
    history.stepSize(end+1, 1) = str2double(tokens{7}); %#ok<AGROW>
    if isempty(tokens{2})
        history.state{end+1, 1} = 'iter'; %#ok<AGROW>
    else
        history.state{end+1, 1} = 'restoration'; %#ok<AGROW>
    end
end
end

function history = initializeSolverHistory()
history = struct();
history.iteration = [];
history.fval = [];
history.maxEqResidual = [];
history.maxIneqViolation = [];
history.maxConstraintViolation = [];
history.firstOrderOpt = [];
history.stepSize = [];
history.state = {};
end

function entry = buildEntryFromSQP(sqpRaw, model, scene, disc)
traj = rebuildImplicitTrajectoryFromDecision(sqpRaw.zOpt, model, scene, disc);
denseReport = validateTrajectoryDenseImplicit(traj, model, scene, disc);
objectiveBreakdown = computeObjectiveBreakdownImplicit(traj, model, disc);
result = buildPostResult(traj, denseReport, sqpRaw, objectiveBreakdown);
entry = struct();
entry.solverName = sqpRaw.solverName;
entry.solverResult = sqpRaw;
entry.traj = traj;
entry.denseReport = denseReport;
entry.objectiveBreakdown = objectiveBreakdown;
entry.result = result;
entry.iterationHistory = sqpRaw.iterationHistory;
entry.usesExactGradient = sqpRaw.usesExactGradient;
entry.usesVariableScaling = sqpRaw.usesVariableScaling;
entry.initialValueDescription = 'shared quintic implicit HS z0';
entry.statusText = sprintf('exitflag=%d', sqpRaw.exitflag);
entry.zOpt = sqpRaw.zOpt;
end

function result = buildPostResult(traj, denseReport, solverResult, objectiveBreakdown)
% buildPostResult - 统一区分求解器收敛和连续后验通过
result = struct();
result.err.startStateNorm = norm(traj.Xnode(:, 1) - traj.xStart);
result.err.endStateNorm = norm(traj.Xnode(:, end) - traj.xEnd);
result.err.hsEqualityMax = solverResult.maxEqResidual;
result.constraint.maxPathViolation = solverResult.maxIneqViolation;
result.constraint.denseMaxPathViolation = denseReport.maxPathViolation;
result.constraint.minClearance = denseReport.minClearance;
result.constraint.minSigmaMin = denseReport.minSigmaMin;
result.constraint.maxCondJ = denseReport.maxCondJ;
result.constraint.forceUpperViolationMax = denseReport.forceUpperViolationMax;
result.constraint.forceLowerViolationMax = denseReport.forceLowerViolationMax;
result.nlpPassed = solverResult.success;
result.postCheck.pathPassed = denseReport.pathPassed;
result.postCheck.forcePassed = denseReport.forcePassed;
result.postCheck.singularityPassed = denseReport.singularityPassed;
result.postCheck.kinematicsPassed = denseReport.kinematicsPassed;
result.postCheck.stateDynamicsPassed = denseReport.stateDynamicsPassed;
result.postCheck.geometricDynamicsPassed = denseReport.geometricDynamicsPassed;
result.solverTrajectoryPassed = result.nlpPassed && denseReport.solverTrajectoryPassed;
result.engineeringTrajectoryPassed = result.nlpPassed && denseReport.engineeringTrajectoryPassed;
result.objectiveBreakdown = objectiveBreakdown;
end

function value = readStatsField(stats, fieldName, defaultValue)
if isstruct(stats) && isfield(stats, fieldName)
    value = stats.(fieldName);
else
    value = defaultValue;
end
end
