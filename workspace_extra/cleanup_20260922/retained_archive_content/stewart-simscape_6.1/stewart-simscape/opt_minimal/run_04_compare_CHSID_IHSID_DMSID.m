function result = run_04_compare_CHSID_IHSID_DMSID(varargin)
% 三种 active 方法均为隐式动力学：CHSID、IHSID、DMSID。
%
% 输入（名称-值）：
%   gridList      - 每行 [N1 N2]，默认 [20 10; 40 20; 60 30; 80 40]。
%   makePlots     - 是否输出对比图，默认 false。
%   makeAnimation - 是否输出默认网格最佳方法动画，默认 false。
%
% 输出：
%   result - 保存结果目录、逐项 trial 结果和汇总行。
%
% 在实验链路中的作用：
%   在同一模型、同一场景、同一 IPOPT/MA27 下运行指定方法。
%   默认只运行 20x10 的 CHSID、IHSID、DMSID，完整网格由用户手动开启。
%   运行后生成 MAT、CSV、TXT，并可选生成默认网格对比图。
parser = inputParser();
parser.addParameter('gridList', [20 10; 40 20; 60 30]);
parser.addParameter('runOnlyFirstGrid', true);
parser.addParameter('runAllGrids', true);
parser.addParameter('makePlots', false);
parser.addParameter('makeAnimation', false);
parser.addParameter('maxIter', 300);
parser.addParameter('printLevel', 4);
% active 方法只保留三种隐式动力学实现。
parser.addParameter('methods', {'CHSID', 'IHSID', 'DMSID'});
parser.addParameter('hessianApproximation', 'limited-memory');
parser.addParameter('acceptableTol', 1e-3);
parser.addParameter('acceptableIter', 1);
parser.parse(varargin{:});
gridList = parser.Results.gridList;
if parser.Results.runOnlyFirstGrid && ~parser.Results.runAllGrids
    gridList = gridList(1, :);
end
makePlots = parser.Results.makePlots;
makeAnimation = parser.Results.makeAnimation;
methods = normalizeMethodList(parser.Results.methods);
solverOptions = struct('maxIter', parser.Results.maxIter, 'printLevel', parser.Results.printLevel, ...
    'hessianApproximation', char(string(parser.Results.hessianApproximation)), ...
    'acceptableTol', parser.Results.acceptableTol, ...
    'acceptableIter', parser.Results.acceptableIter);

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
setupCasadiIpoptMa27(projectRoot);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ['compare_CHSID_IHSID_DMSID_', timestamp]);
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

rows = struct([]);
trials = struct([]);
for gridIndex = 1:size(gridList, 1)
    disc = buildComparisonDisc(scene, gridList(gridIndex, 1), gridList(gridIndex, 2));
    for methodIndex = 1:numel(methods)
        method = methods{methodIndex};
        fprintf('\n===== 离散比较 %s N1=%d N2=%d =====\n', method, disc.numIntervalsApproach, disc.numIntervalsInsertion);
        trial = runOneMethod(method, model, scene, disc, solverOptions);
        trials = [trials; trial]; %#ok<AGROW>
        rows = [rows; trial.row]; %#ok<AGROW>
        saveTrialDetail(resultDir, trial);
        savePartialComparison(resultDir, model, scene, trials, rows, gridList);
    end
end

summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results.mat');
save(matFile, 'model', 'scene', 'trials', 'summaryTable', 'gridList');
reportFile = fullfile(resultDir, 'comparison_report.txt');
writeComparisonReport(reportFile, summaryTable, scene);
readmeFile = fullfile(resultDir, 'README_compare_CHSID_IHSID_DMSID.md');
writeThreeMethodReadme(readmeFile, summaryTable, solverOptions);
plotFiles = {};
animationFile = '';
if makePlots
    plotFiles = writeComparisonPlots(summaryTable, trials, resultDir, scene);
end
if makeAnimation
    animationFile = writeBestDefaultAnimation(summaryTable, trials, model, scene, resultDir, timestamp);
end

result = struct();
result.resultDir = resultDir;
result.rows = rows;
result.trials = trials;
result.summaryTable = summaryTable;
result.csvFile = csvFile;
result.matFile = matFile;
result.reportFile = reportFile;
result.readmeFile = readmeFile;
result.plotFiles = plotFiles;
result.animationFile = animationFile;
fprintf('\n比较结果目录：%s\n', resultDir);
end

function methods = normalizeMethodList(methodsInput)
if ischar(methodsInput) || isstring(methodsInput)
    methods = cellstr(methodsInput);
else
    methods = methodsInput;
end
if isempty(methods)
    error('run_03_compare:EmptyMethods', 'methods 不能为空。');
end
validMethods = {'CHSID', 'IHSID', 'DMSID'};
for i = 1:numel(methods)
    methods{i} = char(string(methods{i}));
    methods{i} = mapLegacyMethodName(methods{i});
    if ~ismember(methods{i}, validMethods)
        error('run_03_compare:UnknownMethod', '未知方法 %s。', methods{i});
    end
end
end

function method = mapLegacyMethodName(method)
switch method
    case 'HSI'
        method = 'CHSID';
    case 'IHS'
        method = 'IHSID';
end
end

function savePartialComparison(resultDir, model, scene, trials, rows, gridList)
summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary_partial.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results_partial.mat');
save(matFile, 'model', 'scene', 'trials', 'summaryTable', 'gridList');
end

function saveTrialDetail(resultDir, trial)
% saveTrialDetail - 为每个方法/网格保存独立详细结果
%
% 输出内容包含求解器状态、后验验证、目标分解、轨迹和行汇总，便于失败方法
% 追溯，不需要重新读取总 MAT 文件。
detailDir = fullfile(resultDir, 'details');
if ~exist(detailDir, 'dir')
    mkdir(detailDir);
end
methodName = char(string(trial.method));
fileName = sprintf('%s_%dx%d.mat', methodName, ...
    trial.disc.numIntervalsApproach, trial.disc.numIntervalsInsertion);
detail = trial; %#ok<NASGU>
save(fullfile(detailDir, fileName), 'detail');
end

function trial = runOneMethod(method, model, scene, disc, solverOptions)
trial = struct('method', method, 'disc', disc, 'zOpt', [], 'traj', [], 'denseReport', [], ...
    'solverResult', [], 'objectiveBreakdown', [], 'row', []);
row = baseRow(method, disc);
try
    switch method
        case 'CHSID'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
            nlpData = buildCasadiImplicitHSNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateImplicitTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = disc.numNodes + disc.numMidpoints;
        case 'IHSID'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
            nlpData = buildCasadiImplicitIHSNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateImplicitIHSTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = disc.numNodes + disc.numMidpoints;
        case 'DMSID'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
            nlpData = buildCasadiImplicitDMSNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateImplicitDMSTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = 8*disc.numIntervals + disc.numNodes + disc.numMidpoints;
        otherwise
            error('run_03_compare:UnknownMethod', '未知方法 %s。', method);
    end
    solveTimer = tic;
    sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, ...
        'lbg', nlpData.lbg, 'ubg', nlpData.ubg);
    solveTime = toc(solveTimer);
    stats = nlpData.solver.stats();
    zOpt = full(sol.x);
    fval = full(sol.f);
    gOpt = full(sol.g);
    gEqOpt = gOpt(1:nlpData.sizes.numEq);
    cOpt = gOpt(nlpData.sizes.numEq+1:end);
    data = evalTrajectory(zOpt);
    traj = rebuildComparisonTrajectory(data, scene, disc);
    denseReport = validateTrajectoryDenseComparison(method, traj, data, model, scene, disc);
    objectiveBreakdown = computeObjectiveBreakdownImplicit(data, model, scene, disc, initialGuess);
    solverResult = struct('return_status', char(stats.return_status), ...
        'success', isfield(stats, 'success') && stats.success, ...
        'iterations', stats.iter_count, 'solveTime', solveTime, 'fval', fval, ...
        'maxEqResidual', max(abs(gEqOpt(:))), 'maxIneqViolation', max([cOpt(:); 0]), ...
        'stats', stats);
    row = fillSuccessRow(row, nlpData.sizes, solverResult, denseReport, objectiveBreakdown, ...
        buildTime, dynamicsEvalCountEstimate, nlpData.opts, data);
    trial.zOpt = zOpt;
    trial.traj = traj;
    trial.denseReport = denseReport;
    trial.solverResult = solverResult;
    trial.objectiveBreakdown = objectiveBreakdown;
catch ME
    row.failureReason = string(ME.message);
    row.solverStatus = "FAILED_BEFORE_RESULT";
    row.ipoptStatus = "FAILED_BEFORE_RESULT";
    fprintf('方法 %s 失败：%s\n', method, ME.message);
end
trial.row = row;
end

function row = baseRow(method, disc)
row = struct();
row.method = string(method);
row.methodName = string(method);
row.N1 = disc.numIntervalsApproach;
row.N2 = disc.numIntervalsInsertion;
row.gridApproach = disc.numIntervalsApproach;
row.gridInsertion = disc.numIntervalsInsertion;
row.h = disc.h;
row.numZ = NaN;
row.numVariables = NaN;
row.numEq = NaN;
row.numIneq = NaN;
row.solverStatus = "";
row.ipoptStatus = "";
row.solverSuccess = false;
row.successFlag = false;
row.engineeringPassed = false;
row.buildTime_s = NaN;
row.solveTime_s = NaN;
row.totalTime_s = NaN;
row.solveTime = NaN;
row.iterations = NaN;
row.iterCount = NaN;
row.iter_count = NaN;
row.n_eval_f = NaN;
row.n_eval_g = NaN;
row.n_eval_grad_f = NaN;
row.n_eval_jac_g = NaN;
row.n_eval_h = NaN;
row.time_nlp_f = NaN;
row.time_nlp_g = NaN;
row.time_grad_f = NaN;
row.time_jac_g = NaN;
row.time_hess_l = NaN;
row.time_other_upper_bound = NaN;
row.jac_g_total_s = NaN;
row.jac_g_avg_s = NaN;
row.jac_g_pct_solve = NaN;
row.casadi_eval_pct_solve = NaN;
row.objectiveTotal = NaN;
row.objective = NaN;
row.nominalCost = NaN;
row.legAccelCost = NaN;
row.singularityCost = NaN;
row.forceRateCost = NaN;
row.objectiveLegAccel = NaN;
row.objectiveSingularity = NaN;
row.maxEqResidual = NaN;
row.maxIneqViolation = NaN;
row.minStage1Clearance_m = NaN;
row.minStage1Clearance = NaN;
row.minStage1Gap = NaN;
row.minDenseGap = NaN;
row.finalGap_m = NaN;
row.maxDynResidual = NaN;
row.maxDefectResidual = NaN;
row.minSigmaMin = NaN;
row.maxCondJ = NaN;
row.maxAbsLegSpeed = NaN;
row.maxAbsLegAccel = NaN;
row.maxAbsForce = NaN;
row.maxLegSpeedViolation = NaN;
row.maxLegAccelViolation = NaN;
row.maxPathViolation = NaN;
row.maxForceViolation = NaN;
row.minSigma = NaN;
row.maxStage2LateralError = NaN;
row.maxStage2HeightError = NaN;
row.maxStage2AttitudeError = NaN;
row.maxInsertionBackwardSpeedViolation = NaN;
row.stage2Passed = false;
row.denseSampleCount = NaN;
row.dynamicsEvalCountEstimate = NaN;
row.maxDenseDynResidual = NaN;
row.maxHSDefectResidual = NaN;
row.maxMidConsistencyResidual = NaN;
row.ipoptLinearSolver = "";
row.ipoptHessianApproximation = "";
row.ipoptTol = NaN;
row.ipoptConstrViolTol = NaN;
row.ipoptMaxIter = NaN;
row.failureReason = "";
end

function row = fillSuccessRow(row, sizes, solverResult, denseReport, objectiveBreakdown, buildTime, dynamicsEvalCountEstimate, opts, data)
row.numZ = sizes.numZ;
row.numVariables = sizes.numZ;
row.numEq = sizes.numEq;
row.numIneq = sizes.numIneq;
row.solverStatus = string(solverResult.return_status);
row.ipoptStatus = string(solverResult.return_status);
row.solverSuccess = solverResult.success;
row.successFlag = solverResult.success;
row.engineeringPassed = isfield(denseReport, 'engineeringTrajectoryPassed') && denseReport.engineeringTrajectoryPassed;
row.buildTime_s = buildTime;
row.solveTime_s = solverResult.solveTime;
row.totalTime_s = buildTime + solverResult.solveTime;
row.solveTime = solverResult.solveTime;
row.iterations = solverResult.iterations;
row.iterCount = solverResult.iterations;
row.iter_count = solverResult.iterations;
row.n_eval_f = getStatNumber(solverResult.stats, 'n_call_nlp_f');
row.n_eval_g = getStatNumber(solverResult.stats, 'n_call_nlp_g');
row.n_eval_grad_f = getStatNumber(solverResult.stats, 'n_call_nlp_grad_f');
row.n_eval_jac_g = getStatNumber(solverResult.stats, 'n_call_nlp_jac_g');
row.n_eval_h = getStatNumber(solverResult.stats, 'n_call_nlp_hess_l');
row.time_nlp_f = getStatNumber(solverResult.stats, 't_proc_nlp_f');
row.time_nlp_g = getStatNumber(solverResult.stats, 't_proc_nlp_g');
row.time_grad_f = getStatNumber(solverResult.stats, 't_proc_nlp_grad_f');
row.time_jac_g = getStatNumber(solverResult.stats, 't_proc_nlp_jac_g');
row.time_hess_l = getStatNumber(solverResult.stats, 't_proc_nlp_hess_l');
casadiEvalTime = sum([row.time_nlp_f, row.time_nlp_g, row.time_grad_f, row.time_jac_g, row.time_hess_l], 'omitnan');
row.time_other_upper_bound = max(row.solveTime_s - casadiEvalTime, 0);
row.jac_g_total_s = row.time_jac_g;
if row.n_eval_jac_g > 0
    row.jac_g_avg_s = row.jac_g_total_s / row.n_eval_jac_g;
end
if row.solveTime_s > 0
    row.jac_g_pct_solve = 100 * row.jac_g_total_s / row.solveTime_s;
    row.casadi_eval_pct_solve = 100 * casadiEvalTime / row.solveTime_s;
end
row.objectiveTotal = objectiveBreakdown.total;
row.objective = objectiveBreakdown.total;
row.nominalCost = objectiveBreakdown.nominalStage1;
row.legAccelCost = objectiveBreakdown.legAccel;
row.singularityCost = objectiveBreakdown.singularity;
row.forceRateCost = objectiveBreakdown.forceRate;
row.objectiveLegAccel = objectiveBreakdown.legAccel;
row.objectiveSingularity = objectiveBreakdown.singularity;
row.maxEqResidual = solverResult.maxEqResidual;
row.maxIneqViolation = solverResult.maxIneqViolation;
row.minStage1Clearance_m = denseReport.minStage1Clearance;
row.minStage1Clearance = denseReport.minStage1Clearance;
row.minStage1Gap = denseReport.minStage1Clearance;
row.minDenseGap = denseReport.minClearance;
row.finalGap_m = denseReport.finalGap;
if isfield(denseReport, 'maxDynResidual')
    row.maxDynResidual = denseReport.maxDynResidual;
else
    row.maxDynResidual = denseReport.maxDynResidualGeometric;
end
row.maxDefectResidual = denseReport.maxDefectResidual;
row.minSigmaMin = denseReport.minSigmaMin;
row.maxCondJ = denseReport.maxCondJ;
row.maxAbsLegSpeed = denseReport.maxAbsLd;
row.maxAbsLegAccel = denseReport.maxAbsLdd;
row.maxAbsForce = denseReport.maxAbsForce;
row.maxLegSpeedViolation = denseReport.maxPathViolation;
row.maxLegAccelViolation = denseReport.maxPathViolation;
row.maxPathViolation = denseReport.maxPathViolation;
row.maxForceViolation = max(denseReport.forceUpperViolationMax, denseReport.forceLowerViolationMax);
row.minSigma = denseReport.minSigmaMin;
row.maxStage2LateralError = denseReport.stage2MaxLateralError;
row.maxStage2HeightError = denseReport.stage2MaxHeightError;
row.maxStage2AttitudeError = denseReport.stage2MaxAttitudeError;
row.maxInsertionBackwardSpeedViolation = max(-denseReport.stage2MinInsertionSpeed, 0);
row.stage2Passed = denseReport.stage2Passed;
row.denseSampleCount = denseReport.sampleCount;
row.dynamicsEvalCountEstimate = dynamicsEvalCountEstimate;
row.maxDenseDynResidual = row.maxDynResidual;
row.maxHSDefectResidual = row.maxDefectResidual;
if isfield(data, 'maxMidConsistencyResidual')
    row.maxMidConsistencyResidual = data.maxMidConsistencyResidual;
end
row.ipoptLinearSolver = string(opts.ipopt.linear_solver);
row.ipoptHessianApproximation = string(opts.ipopt.hessian_approximation);
row.ipoptTol = opts.ipopt.tol;
row.ipoptConstrViolTol = opts.ipopt.constr_viol_tol;
row.ipoptMaxIter = opts.ipopt.max_iter;
if ~row.engineeringPassed
    row.failureReason = "求解完成但未通过统一工程后验验证";
end
end

function value = getStatNumber(stats, fieldName)
% getStatNumber - 兼容不同 CasADi 版本的 stats 数值字段读取
if isfield(stats, fieldName)
    rawValue = stats.(fieldName);
    if isnumeric(rawValue) || islogical(rawValue)
        value = double(rawValue);
    else
        value = str2double(char(string(rawValue)));
    end
else
    value = NaN;
end
end

function disc = buildComparisonDisc(scene, n1, n2)
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
if abs(disc.hApproach - disc.hInsertion) > 1e-12
    error('run_03_compare:NonUniformStep', '比较实验要求 N1/N2 对应统一步长。');
end
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


function writeComparisonReport(reportFile, summaryTable, scene)
fid = fopen(reportFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '离散方法筛选实验报告\n');
fprintf(fid, '当前场景：box rpy(deg)=[%.3f %.3f %.3f]，罩体构件=%s\n', ...
    rad2deg(scene.box.rpy), strjoin({scene.hood.obstacles.name}, ','));
fprintf(fid, '说明：当前 CHSID 基线包含三构件罩体碰撞证书，因此规模不同于旧单障碍提示词示例。\n\n');
fprintf(fid, '%s\n\n', evalc('disp(summaryTable)'));
fprintf(fid, '初步判断：%s\n', decideConclusion(summaryTable));
end

function writeThreeMethodReadme(readmeFile, summaryTable, solverOptions)
% writeThreeMethodReadme - 输出三种隐式动力学方法实验说明
fid = fopen(readmeFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
opts = makeCommonIpoptOptions(solverOptions);
fprintf(fid, '# CHSID / IHSID / DMSID 隐式动力学对比实验\n\n');
fprintf(fid, '本目录由 `run_04_compare_CHSID_IHSID_DMSID.m` 生成。\n\n');
fprintf(fid, '## 方法命名\n\n');
fprintf(fid, '当前 active 实验只保留 CHSID、IHSID、DMSID 三种隐式动力学方法。\n\n');
fprintf(fid, '- CHSID: Compressed Hermite-Simpson + Implicit Dynamics。旧名 HSI。\n');
fprintf(fid, '- IHSID: Implicit / Full Hermite-Simpson + Implicit Dynamics，中点状态为独立变量。\n');
fprintf(fid, '- DMSID: Direct Multiple Shooting + Implicit Dynamics。本轮新增。\n');
fprintf(fid, '\n');
fprintf(fid, '## 统一 IPOPT 设置\n\n');
fprintf(fid, '- linear_solver: `%s`\n', opts.ipopt.linear_solver);
fprintf(fid, '- hessian_approximation: `%s`\n', opts.ipopt.hessian_approximation);
fprintf(fid, '- max_iter: `%d`\n', opts.ipopt.max_iter);
fprintf(fid, '- tol: `%.3g`\n', opts.ipopt.tol);
fprintf(fid, '- constr_viol_tol: `%.3g`\n', opts.ipopt.constr_viol_tol);
fprintf(fid, '- acceptable_tol: `%.3g`\n', opts.ipopt.acceptable_tol);
fprintf(fid, '- acceptable_iter: `%d`\n', opts.ipopt.acceptable_iter);
fprintf(fid, '- bound_push/bound_frac: `%.3g` / `%.3g`\n', opts.ipopt.bound_push, opts.ipopt.bound_frac);
fprintf(fid, '- mu_strategy: `%s`\n\n', opts.ipopt.mu_strategy);
fprintf(fid, '三种方法均通过 `makeCommonIpoptOptions` 生成设置；若 exact Hessian 无法稳定运行，应统一切换，不允许单独调整某一种方法。\n\n');
fprintf(fid, 'IHSID 的 `maxMidConsistencyResidual` 来自新增 g_mid；CHSID/DMSID 对该字段填 NaN。\n\n');
fprintf(fid, '## 验证口径\n\n');
fprintf(fid, '验证分为三层：离散 NLP 可行性、工程可行性、轨迹质量诊断。\n');
fprintf(fid, 'CHS 的 dense 非配点动力学残差只作为轨迹质量诊断，不直接否决 engineeringPassed。\n');
fprintf(fid, 'DMS 的 dense 动力学残差由积分器定义，不能直接解释为 DMS 物理一致性优于 CHS。\n\n');
fprintf(fid, '## 当前自检结果\n\n');
fprintf(fid, '```text\n%s\n```\n', evalc('disp(summaryTable)'));
end

function conclusion = decideConclusion(T)
passed = T(T.engineeringPassed, :);
if isempty(passed)
    conclusion = '当前结果中的方法均未稳定通过当前硬碰撞统一验证，当前不足以判断离散方式优劣，应先检查可行性、初值或碰撞约束。';
    return;
end
defaultRows = T(T.N1 == 20 & T.N2 == 10, :);
requiredDefaultMethods = ["CHSID"; "IHSID"; "DMSID"];
if isempty(defaultRows) || ~all(ismember(requiredDefaultMethods, string(defaultRows.method)))
    conclusion = '当前结果未包含 20x10 的 CHSID/IHSID/DMSID 三个方法，先检查实验是否完整。';
    return;
end
chsid = defaultRows(defaultRows.method == "CHSID", :);
ihsid = defaultRows(defaultRows.method == "IHSID", :);
dmsid = defaultRows(defaultRows.method == "DMSID", :);
if ~isempty(chsid) && ~isempty(ihsid) && ihsid.successFlag && ihsid.jac_g_avg_s < chsid.jac_g_avg_s
    conclusion = 'IHSID 的 jac_g 单次评价时间低于 CHSID，可继续放大网格验证中点状态变量化收益。';
elseif ~isempty(chsid) && ~isempty(dmsid) && chsid.engineeringPassed && dmsid.engineeringPassed && dmsid.solveTime_s < 0.75*chsid.solveTime_s
    conclusion = '默认启用的 CHSID 与 DMSID 均通过且 DMSID 明显更快，后续可继续评估直接多重射击路线。';
else
    conclusion = '当前 20x10 结果尚不足以证明 IHSID 或 DMSID 相对 CHSID 的稳定优势，建议再看更大网格。';
end
end

function plotFiles = writeComparisonPlots(summaryTable, trials, resultDir, scene)
plotFiles = {};
defaultRows = summaryTable(summaryTable.N1 == 40 & summaryTable.N2 == 20, :);
if isempty(defaultRows)
    return;
end
fig = figure('Name', 'comparison_table', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; bar(categorical(defaultRows.method), defaultRows.solveTime_s); ylabel('求解时间 [s]'); grid on;
nexttile; bar(categorical(defaultRows.method), defaultRows.objectiveTotal); ylabel('目标函数'); grid on;
nexttile; bar(categorical(defaultRows.method), defaultRows.minStage1Clearance_m); ylabel('第一阶段最小间隙 [m]'); grid on;
nexttile; bar(categorical(defaultRows.method), defaultRows.maxDynResidual); ylabel('动力学残差'); grid on;
plotFiles{end+1} = saveFig(fig, resultDir, 'comparison_table.png'); %#ok<AGROW>

defaultTrials = trials(arrayfun(@(x) x.disc.numIntervalsApproach == 40 && x.disc.numIntervalsInsertion == 20 && ~isempty(x.traj), trials));
if ~isempty(defaultTrials)
    fig = figure('Name', 'trajectory_overlay_default', 'Color', 'w'); hold on; grid on; axis equal;
    for i = 1:numel(defaultTrials)
        traj = defaultTrials(i).traj;
        plot3(traj.Q(1,:), traj.Q(2,:), traj.Q(3,:), 'LineWidth', 1.2, 'DisplayName', defaultTrials(i).method);
    end
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); legend('Location', 'best');
    plotFiles{end+1} = saveFig(fig, resultDir, 'trajectory_overlay_default.png'); %#ok<AGROW>
    fig = figure('Name', 'clearance_compare_default', 'Color', 'w'); hold on; grid on;
    for i = 1:numel(defaultTrials)
        traj = defaultTrials(i).traj;
        plot(traj.t, traj.minClearance, 'LineWidth', 1.2, 'DisplayName', defaultTrials(i).method);
    end
    yline(scene.collision.safeDistance, 'r--'); %#ok<*UNRCH>
    xlabel('t [s]'); ylabel('节点最小间隙 [m]'); legend('Location', 'best');
    plotFiles{end+1} = saveFig(fig, resultDir, 'clearance_compare_default.png'); %#ok<AGROW>
end
end

function animationFile = writeBestDefaultAnimation(summaryTable, trials, model, scene, resultDir, timestamp)
animationFile = '';
defaultRows = summaryTable(summaryTable.N1 == 40 & summaryTable.N2 == 20 & summaryTable.engineeringPassed, :);
if isempty(defaultRows)
    return;
end
[~, bestIndex] = min(defaultRows.objectiveTotal);
bestMethod = defaultRows.method(bestIndex);
for i = 1:numel(trials)
    if string(trials(i).method) == bestMethod && trials(i).disc.numIntervalsApproach == 40 && ~isempty(trials(i).traj)
        animationFile = animateStewartTrajectory(trials(i).traj, model, scene, resultDir, ['compare_best_', char(bestMethod), '_', timestamp]);
        return;
    end
end
end

function fileName = saveFig(fig, resultDir, name)
fileName = fullfile(resultDir, name);
exportgraphics(fig, fileName, 'Resolution', 160);
close(fig);
end
