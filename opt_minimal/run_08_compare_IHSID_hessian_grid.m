function result = run_08_compare_IHSID_hessian_grid(varargin)
% run_08_compare_IHSID_hessian_grid - IHSID 主线 Hessian/网格对比实验
%
% 本入口只维护 standard IHSID 主线，不混入 FATROP manual 或其它离散方法。
% 每个网格共享同一个初值和 NLP 结构，只切换 IPOPT 的 Hessian 设置。

parser = inputParser();
parser.addParameter('gridList', [10 5; 20 10; 40 20; 60 30; 80 40]);
parser.addParameter('hessianModes', {'exact', 'limited-memory'});
parser.addParameter('maxIter', 300);
parser.addParameter('printLevel', 4);
parser.addParameter('acceptableTol', 1e-3);
parser.addParameter('acceptableIter', 1);
parser.addParameter('tol', 1e-6);
parser.addParameter('constrViolTol', 1e-6);

parser.addParameter('dryRun', false);
parser.parse(varargin{:});

gridList = normalizeGridList(parser.Results.gridList);
hessianModes = normalizeHessianModes(parser.Results.hessianModes);

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ...
    ['compare_IHSID_hessian_grid_', timestamp]);
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

rows = struct([]);
trials = struct([]);
for gridIndex = 1:size(gridList, 1)
    disc = buildIhsidDisc(scene, gridList(gridIndex, 1), gridList(gridIndex, 2));
    [z0, initialGuess] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
    for modeIndex = 1:numel(hessianModes)
        hessianMode = hessianModes{modeIndex};
        solverOptions = makeSolverOptions(parser.Results, hessianMode);
        fprintf('\n===== IHSID %s Hessian N1=%d N2=%d =====\n', ...
            hessianMode, disc.numIntervalsApproach, disc.numIntervalsInsertion);
        if parser.Results.dryRun
            trial = makeDryRunTrial(disc, hessianMode, solverOptions);
        else
            trial = runOneHessianMode(model, scene, disc, z0, initialGuess, ...
                hessianMode, solverOptions);
        end
        trials = [trials; trial]; %#ok<AGROW>
        rows = [rows; trial.row]; %#ok<AGROW>
        saveTrialDetail(resultDir, trial);
        savePartial(resultDir, model, scene, gridList, hessianModes, trials, rows);
    end
end

summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results.mat');
save(matFile, 'model', 'scene', 'gridList', 'hessianModes', 'trials', 'summaryTable');
reportFile = fullfile(resultDir, 'comparison_report.txt');
writeReport(reportFile, summaryTable);
readmeFile = fullfile(resultDir, 'README_IHSID_hessian_grid.md');
writeReadme(readmeFile, summaryTable, parser.Results);

result = struct();
result.resultDir = resultDir;
result.rows = rows;
result.trials = trials;
result.summaryTable = summaryTable;
result.csvFile = csvFile;
result.matFile = matFile;
result.reportFile = reportFile;
result.readmeFile = readmeFile;
fprintf('\nIHSID Hessian/网格对比结果目录：%s\n', resultDir);
end

function gridList = normalizeGridList(gridList)
gridList = double(gridList);
if isempty(gridList) || size(gridList, 2) ~= 2
    error('run_08_compare_IHSID_hessian_grid:BadGridList', ...
        'gridList 必须是 n 行 2 列的 [N1 N2] 矩阵。');
end
if any(gridList(:) <= 0) || any(abs(gridList(:) - round(gridList(:))) > 0)
    error('run_08_compare_IHSID_hessian_grid:BadGridList', ...
        'gridList 中的 N1/N2 必须是正整数。');
end
end

function hessianModes = normalizeHessianModes(hessianModes)
if ischar(hessianModes) || isstring(hessianModes)
    hessianModes = cellstr(hessianModes);
end
validModes = {'exact', 'limited-memory'};
for i = 1:numel(hessianModes)
    hessianModes{i} = char(string(hessianModes{i}));
    if ~ismember(hessianModes{i}, validModes)
        error('run_08_compare_IHSID_hessian_grid:BadHessianMode', ...
            'Hessian 模式必须是 exact 或 limited-memory。');
    end
end
end

function solverOptions = makeSolverOptions(parserResults, hessianMode)
solverOptions = struct();
solverOptions.solverBackend = 'ipopt';
solverOptions.maxIter = parserResults.maxIter;
solverOptions.printLevel = parserResults.printLevel;
solverOptions.hessianApproximation = hessianMode;
solverOptions.acceptableTol = parserResults.acceptableTol;
solverOptions.acceptableIter = parserResults.acceptableIter;
solverOptions.tol = parserResults.tol;
solverOptions.constrViolTol = parserResults.constrViolTol;
end

function trial = makeDryRunTrial(disc, hessianMode, solverOptions)
row = baseRow(disc, hessianMode);
opts = makeCommonIpoptOptions(solverOptions);
row.solverStatus = "DRY_RUN";
row.ipoptStatus = "DRY_RUN";
row.ipoptLinearSolver = string(opts.ipopt.linear_solver);
row.ipoptHessianApproximation = string(opts.ipopt.hessian_approximation);
row.ipoptTol = opts.ipopt.tol;
row.ipoptConstrViolTol = opts.ipopt.constr_viol_tol;
row.ipoptMaxIter = opts.ipopt.max_iter;
row.failureReason = "dryRun=true，仅检查实验编排和输出字段";
trial = struct('method', 'IHSID', 'hessianMode', hessianMode, 'disc', disc, ...
    'zOpt', [], 'traj', [], 'denseReport', [], 'solverResult', [], ...
    'objectiveBreakdown', [], 'row', row);
end

function trial = runOneHessianMode(model, scene, disc, z0, initialGuess, hessianMode, solverOptions)
row = baseRow(disc, hessianMode);
trial = struct('method', 'IHSID', 'hessianMode', hessianMode, 'disc', disc, ...
    'zOpt', [], 'traj', [], 'denseReport', [], 'solverResult', [], ...
    'objectiveBreakdown', [], 'row', row);

buildTimer = tic;
try
    nlpData = buildCasadiImplicitIHSNLP(model, scene, disc, initialGuess, solverOptions);
    buildTime = toc(buildTimer);
catch ME
    buildTime = toc(buildTimer);
    row.buildTime_s = buildTime;
    row.totalTime_s = buildTime;
    row.solverStatus = "FAILED_AT_INITIALIZATION";
    row.ipoptStatus = row.solverStatus;
    row.failureReason = string(compactMessage(ME.message));
    trial.row = row;
    fprintf('IHSID %s 初始化失败：%s\n', hessianMode, row.failureReason);
    return;
end

row.buildTime_s = buildTime;
row.numVariables = nlpData.sizes.numZ;
row.numZ = nlpData.sizes.numZ;
row.numEq = nlpData.sizes.numEq;
row.numIneq = nlpData.sizes.numIneq;
row.ipoptLinearSolver = string(nlpData.opts.ipopt.linear_solver);
row.ipoptHessianApproximation = string(nlpData.opts.ipopt.hessian_approximation);
row.ipoptTol = nlpData.opts.ipopt.tol;
row.ipoptConstrViolTol = nlpData.opts.ipopt.constr_viol_tol;
row.ipoptMaxIter = nlpData.opts.ipopt.max_iter;

try
    solveTimer = tic;
    sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, ...
        'lbg', nlpData.lbg, 'ubg', nlpData.ubg);
    solveTime = toc(solveTimer);
    stats = nlpData.solver.stats();
catch ME
    solveTime = toc(solveTimer);
    stats = nlpData.solver.stats();
    row = fillFailedSolveStats(row, stats, buildTime, solveTime, ME);
    trial.row = row;
    fprintf('IHSID %s 求解失败：%s\n', hessianMode, row.failureReason);
    return;
end

zOpt = full(sol.x);
fval = full(sol.f);
gOpt = full(sol.g);
gEqOpt = gOpt(1:nlpData.sizes.numEq);
cOpt = gOpt(nlpData.sizes.numEq+1:end);
data = evaluateImplicitIHSTrajectoryNumeric(zOpt, model, scene, disc);
traj = rebuildComparisonTrajectory(data, scene, disc);
denseReport = validateTrajectoryDenseComparison('IHSID', traj, data, model, scene, disc);
objectiveBreakdown = computeObjectiveBreakdownImplicit(data, model, scene, disc, initialGuess);
solverResult = struct('return_status', readStatus(stats), ...
    'success', readSuccess(stats), ...
    'iterations', readIterations(stats), ...
    'solveTime', solveTime, ...
    'fval', fval, ...
    'maxEqResidual', max(abs(gEqOpt(:))), ...
    'maxIneqViolation', max([cOpt(:); 0]), ...
    'stats', stats);

row = fillSuccessRow(row, nlpData.sizes, solverResult, denseReport, ...
    objectiveBreakdown, buildTime, disc.numNodes + disc.numMidpoints, nlpData, data);
trial.zOpt = zOpt;
trial.traj = traj;
trial.denseReport = denseReport;
trial.solverResult = solverResult;
trial.objectiveBreakdown = objectiveBreakdown;
trial.row = row;
end

function row = baseRow(disc, hessianMode)
row = struct();
row.comparisonGroup = "IHSID-HESSIAN-GRID";
row.caseName = sprintf('IHSID-IPOPT-%s-%dx%d', hessianMode, ...
    disc.numIntervalsApproach, disc.numIntervalsInsertion);
row.method = "IHSID";
row.solverBackend = "ipopt";
row.hessianMode = string(hessianMode);
row.N1 = disc.numIntervalsApproach;
row.N2 = disc.numIntervalsInsertion;
row.gridApproach = disc.numIntervalsApproach;
row.gridInsertion = disc.numIntervalsInsertion;
row.h = disc.h;
row.numVariables = NaN;
row.numZ = NaN;
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
row.objective = NaN;
row.iterations = NaN;
row.iterCount = NaN;
row.iter_count = NaN;
row.n_eval_f = NaN;
row.n_eval_g = NaN;
row.n_eval_grad_f = NaN;
row.n_eval_jac_g = NaN;
row.n_eval_hess_l = NaN;
row.time_nlp_f = NaN;
row.time_nlp_g = NaN;
row.time_grad_f = NaN;
row.time_jac_g = NaN;
row.time_hess_l = NaN;
row.hessianTimePct = NaN;
row.jac_g_total_s = NaN;
row.jac_g_avg_s = NaN;
row.jac_g_pct_solve = NaN;
row.casadi_eval_pct_solve = NaN;
row.time_other_upper_bound = NaN;
row.maxEqResidual = NaN;
row.maxIneqViolation = NaN;
row.minStage1Clearance_m = NaN;
row.finalGap_m = NaN;
row.maxDenseDynResidual = NaN;
row.maxHSDefectResidual = NaN;
row.maxMidConsistencyResidual = NaN;
row.maxStage2LateralError = NaN;
row.maxStage2HeightError = NaN;
row.maxStage2AttitudeError = NaN;
row.stage2Passed = false;
row.denseSampleCount = NaN;
row.dynamicsEvalCountEstimate = NaN;
row.ipoptLinearSolver = "";
row.ipoptHessianApproximation = "";
row.ipoptTol = NaN;
row.ipoptConstrViolTol = NaN;
row.ipoptMaxIter = NaN;
row.failureReason = "";
end

function row = fillFailedSolveStats(row, stats, buildTime, solveTime, ME)
row.solveTime_s = solveTime;
row.totalTime_s = buildTime + solveTime;
row.solverStatus = "FAILED_DURING_SOLVE";
row.ipoptStatus = row.solverStatus;
row.failureReason = string(compactMessage(ME.message));
row.iter_count = readIterations(stats);
row.iterations = row.iter_count;
row.iterCount = row.iter_count;
row = fillCasadiTimingStats(row, stats);
end

function row = fillSuccessRow(row, sizes, solverResult, denseReport, objectiveBreakdown, ...
    buildTime, dynamicsEvalCountEstimate, nlpData, data)
stats = solverResult.stats;
row.numVariables = sizes.numZ;
row.numZ = sizes.numZ;
row.numEq = sizes.numEq;
row.numIneq = sizes.numIneq;
row.solverStatus = string(solverResult.return_status);
row.ipoptStatus = row.solverStatus;
row.solverSuccess = solverResult.success;
row.successFlag = solverResult.success;
row.engineeringPassed = isfield(denseReport, 'engineeringTrajectoryPassed') && ...
    denseReport.engineeringTrajectoryPassed;
row.buildTime_s = buildTime;
row.solveTime_s = solverResult.solveTime;
row.totalTime_s = buildTime + solverResult.solveTime;
row.objective = objectiveBreakdown.total;
row.iterations = solverResult.iterations;
row.iterCount = solverResult.iterations;
row.iter_count = solverResult.iterations;
row = fillCasadiTimingStats(row, stats);
row.maxEqResidual = solverResult.maxEqResidual;
row.maxIneqViolation = solverResult.maxIneqViolation;
row.minStage1Clearance_m = denseReport.minStage1Clearance;
row.finalGap_m = denseReport.finalGap;
if isfield(denseReport, 'maxDynResidual')
    row.maxDenseDynResidual = denseReport.maxDynResidual;
else
    row.maxDenseDynResidual = denseReport.maxDynResidualGeometric;
end
row.maxHSDefectResidual = denseReport.maxDefectResidual;
if isfield(data, 'maxMidConsistencyResidual')
    row.maxMidConsistencyResidual = data.maxMidConsistencyResidual;
end
row.maxStage2LateralError = denseReport.stage2MaxLateralError;
row.maxStage2HeightError = denseReport.stage2MaxHeightError;
row.maxStage2AttitudeError = denseReport.stage2MaxAttitudeError;
row.stage2Passed = denseReport.stage2Passed;
row.denseSampleCount = denseReport.sampleCount;
row.dynamicsEvalCountEstimate = dynamicsEvalCountEstimate;
row.ipoptLinearSolver = string(nlpData.opts.ipopt.linear_solver);
row.ipoptHessianApproximation = string(nlpData.opts.ipopt.hessian_approximation);
row.ipoptTol = nlpData.opts.ipopt.tol;
row.ipoptConstrViolTol = nlpData.opts.ipopt.constr_viol_tol;
row.ipoptMaxIter = nlpData.opts.ipopt.max_iter;
if ~row.solverSuccess
    row.failureReason = sprintf('求解器未收敛：status=%s', row.solverStatus);
end
if ~row.engineeringPassed
    engineeringReason = sprintf(['求解完成但未通过统一工程后验验证：stage2Passed=%d, ', ...
        'maxDenseDynResidual=%.6g, minStage1Clearance_m=%.6g, finalGap_m=%.6g'], ...
        row.stage2Passed, row.maxDenseDynResidual, row.minStage1Clearance_m, row.finalGap_m);
    if strlength(string(row.failureReason)) > 0
        row.failureReason = row.failureReason + "；" + engineeringReason;
    else
        row.failureReason = engineeringReason;
    end
end
end

function row = fillCasadiTimingStats(row, stats)
row.n_eval_f = getStatSafe(stats, 'n_call_nlp_f');
row.n_eval_g = getStatSafe(stats, 'n_call_nlp_g');
row.n_eval_grad_f = getStatSafe(stats, 'n_call_nlp_grad_f');
row.n_eval_jac_g = getStatSafe(stats, 'n_call_nlp_jac_g');
row.n_eval_hess_l = getStatSafe(stats, 'n_call_nlp_hess_l');
row.time_nlp_f = getStatSafe(stats, 't_proc_nlp_f');
row.time_nlp_g = getStatSafe(stats, 't_proc_nlp_g');
row.time_grad_f = getStatSafe(stats, 't_proc_nlp_grad_f');
row.time_jac_g = getStatSafe(stats, 't_proc_nlp_jac_g');
row.time_hess_l = getStatSafe(stats, 't_proc_nlp_hess_l');
casadiEvalTime = sum([row.time_nlp_f, row.time_nlp_g, row.time_grad_f, ...
    row.time_jac_g, row.time_hess_l], 'omitnan');
row.time_other_upper_bound = max(row.solveTime_s - casadiEvalTime, 0);
row.jac_g_total_s = row.time_jac_g;
if row.n_eval_jac_g > 0
    row.jac_g_avg_s = row.jac_g_total_s / row.n_eval_jac_g;
end
if row.solveTime_s > 0
    row.jac_g_pct_solve = 100 * row.jac_g_total_s / row.solveTime_s;
    row.casadi_eval_pct_solve = 100 * casadiEvalTime / row.solveTime_s;
    row.hessianTimePct = 100 * row.time_hess_l / row.solveTime_s;
end
end

function status = readStatus(stats)
status = char(string(getStatSafe(stats, 'return_status', "")));
end

function success = readSuccess(stats)
rawSuccess = getStatSafe(stats, 'success', false);
if islogical(rawSuccess) || isnumeric(rawSuccess)
    success = logical(rawSuccess);
else
    status = readStatus(stats);
    success = contains(status, 'Succeed') || contains(status, 'Solve_Succeeded') || ...
        contains(status, 'Solved_To_Acceptable_Level');
end
end

function iterations = readIterations(stats)
iterations = getStatSafe(stats, 'iter_count');
end

function msg = compactMessage(msg)
msg = regexprep(char(string(msg)), '\s+', ' ');
maxLen = 450;
if strlength(string(msg)) > maxLen
    msg = extractBefore(string(msg), maxLen);
    msg = char(msg);
end
end

function saveTrialDetail(resultDir, trial)
detailDir = fullfile(resultDir, 'details');
if ~exist(detailDir, 'dir')
    mkdir(detailDir);
end
fileName = sprintf('IHSID_%s_%dx%d.mat', trial.hessianMode, ...
    trial.disc.numIntervalsApproach, trial.disc.numIntervalsInsertion);
fileName = strrep(fileName, '-', '_');
detail = trial;
save(fullfile(detailDir, fileName), 'detail');
clear detail
end

function savePartial(resultDir, model, scene, gridList, hessianModes, trials, rows)
summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary_partial.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results_partial.mat');
save(matFile, 'model', 'scene', 'gridList', 'hessianModes', 'trials', 'summaryTable');
end

function writeReport(reportFile, summaryTable)
fid = fopen(reportFile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'IHSID Hessian/网格对比实验报告\n\n');
fprintf(fid, '本报告只包含 standard IHSID 主线，逐网格对比 exact 与 limited-memory Hessian。\n\n');
fprintf(fid, '%s\n', tableDisplayText(summaryTable));
clear cleanup
end

function writeReadme(readmeFile, summaryTable, parserResults)
fid = fopen(readmeFile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# IHSID Hessian/网格对比实验\n\n');
fprintf(fid, '本目录由 `run_08_compare_IHSID_hessian_grid.m` 生成。\n\n');
fprintf(fid, '## 范围\n\n');
fprintf(fid, '- 方法：standard IHSID 主线。\n');
fprintf(fid, '- 求解器：IPOPT/MA27。\n');
fprintf(fid, '- Hessian 设置：`exact` 与 `limited-memory`。\n');
fprintf(fid, '- max_iter：`%d`。\n', parserResults.maxIter);
fprintf(fid, '- dryRun：`%d`。\n\n', parserResults.dryRun);
fprintf(fid, '## 输出表\n\n');
fprintf(fid, '```text\n%s\n```\n', tableDisplayText(summaryTable));
clear cleanup
end

function text = tableDisplayText(T)
if isempty(T)
    text = '';
else
    text = evalc('disp(T)');
end
end

function disc = buildIhsidDisc(scene, n1, n2)
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
    error('run_08_compare_IHSID_hessian_grid:NonUniformStep', ...
        'IHSID 对比实验要求 N1/N2 对应统一步长。');
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
