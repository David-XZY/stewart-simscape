function result = run_05_compare_IHSID_FATROP(varargin)
% run_05_compare_IHSID_FATROP - IHSID-IPOPT 涓?IHSID-FATROP-manual 鐨?20x10 鑷
parser = inputParser();
parser.addParameter('grid', [20 10]);
parser.addParameter('makePlots', false);
parser.addParameter('makeAnimation', false);
parser.parse(varargin{:});
if parser.Results.makePlots || parser.Results.makeAnimation
    warning('run_05_compare_IHSID_FATROP:NoMedia', '本脚本不生成图片或动画。');
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);

grid = parser.Results.grid;
disc = buildFatropDisc(scene, grid(1), grid(2));
manualStructureInfo = inspectIHSIDManualStructureDimensions(disc);
fatropProbe = probeFatropOptions();

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ...
    ['compare_IHSID_FATROP_manual_', timestamp]);
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

cases = { ...
    struct('label', 'IHSID-STANDARD-IPOPT', 'method', 'IHSID-STANDARD', 'solverBackend', 'ipopt', 'fatropStructure', ''), ...
    struct('label', 'IHSID-MANUAL-IPOPT', 'method', 'IHSID-MANUAL', 'solverBackend', 'ipopt', 'fatropStructure', 'manual'), ...
    struct('label', 'IHSID-MANUAL-FATROP', 'method', 'IHSID-MANUAL', 'solverBackend', 'fatrop', 'fatropStructure', 'manual')};

trials = struct([]);
rows = struct([]);
for i = 1:numel(cases)
    trial = runOneCase(cases{i}, model, scene, disc, resultDir, fatropProbe);
    trials = [trials; trial]; %#ok<AGROW>
    rows = [rows; trial.row]; %#ok<AGROW>
    saveTrialDetail(resultDir, trial);
    savePartial(resultDir, model, scene, disc, manualStructureInfo, fatropProbe, trials, rows);
end

summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results.mat');
save(matFile, 'model', 'scene', 'disc', 'manualStructureInfo', 'fatropProbe', 'trials', 'summaryTable');
reportFile = fullfile(resultDir, 'comparison_report.txt');
writeReport(reportFile, summaryTable, manualStructureInfo, fatropProbe);
readmeFile = fullfile(resultDir, 'README_IHSID_manual_FATROP_fixed.md');
writeReadme(readmeFile, summaryTable, fatropProbe);

result = struct('resultDir', resultDir, 'summaryTable', summaryTable, ...
    'fatropProbe', fatropProbe, 'trials', trials, 'csvFile', csvFile, ...
    'matFile', matFile, 'reportFile', reportFile, 'readmeFile', readmeFile);
fprintf('\nIHSID FATROP manual 结果目录：%s\n', resultDir);
end

function trial = runOneCase(caseDef, model, scene, disc, resultDir, fatropProbe)
fprintf('\n===== %s N1=%d N2=%d =====\n', caseDef.label, disc.numIntervalsApproach, disc.numIntervalsInsertion);
row = baseRow(caseDef, disc);
trial = struct('caseDef', caseDef, 'method', caseDef.method, 'disc', disc, ...
    'zOpt', [], 'traj', [], 'denseReport', [], 'solverResult', [], ...
    'objectiveBreakdown', [], 'row', row);

buildTimer = tic;
try
    [z0, zIHS, initialGuess, evalTrajectory, dynamicsEvalCountEstimate] = prepareCase(caseDef, model, scene, disc);
    solverOptions = makeSolverOptions(caseDef);
    nlpData = buildCaseNlp(caseDef, model, scene, disc, initialGuess, solverOptions);
    if strcmp(caseDef.fatropStructure, 'manual')
        structureFile = fullfile(resultDir, 'manual_structure_check.txt');
        equivalenceFile = fullfile(resultDir, 'manual_equivalence_check.txt');
        manualStructureCheck = checkFatropManualStructure(nlpData, z0, structureFile);
        stdOptions = struct('solverBackend', 'ipopt', 'maxIter', 30, 'printLevel', 0);
        stdData = buildCasadiImplicitIHSNLP(model, scene, disc, initialGuess, stdOptions);
        manualEquivalenceCheck = checkIHSManualEquivalence(stdData, nlpData, zIHS, z0, equivalenceFile);
        row.maxManualGapResidualAtX0 = manualStructureCheck.maxGapResidualAtX0;
        row.objectiveStdAtX0 = manualEquivalenceCheck.objectiveStdAtX0;
        row.objectiveManualAtX0 = manualEquivalenceCheck.objectiveManualAtX0;
        row.absObjectiveDiffAtX0 = manualEquivalenceCheck.absObjectiveDiffAtX0;
    end
    buildTime = toc(buildTimer);
catch ME
    buildTime = toc(buildTimer);
    row.buildTime_s = buildTime;
    row.totalTime_s = buildTime;
    row.solverStatus = "FAILED_AT_INITIALIZATION";
    row.failureReason = string(compactMessage(ME.message));
    fprintf('鏂规硶 %s 鍒濆鍖栧け璐ワ細%s\n', caseDef.label, row.failureReason);
    trial.row = row;
    return;
end

row.buildTime_s = buildTime;
row.solverBackend = string(nlpData.solverBackend);
row.fatropStructure = string(nlpData.fatropStructure);
row.numVariables = nlpData.sizes.numZ;
row.numZ = nlpData.sizes.numZ;
row.numEq = nlpData.sizes.numEq;
row.numIneq = nlpData.sizes.numIneq;

try
    solveTimer = tic;
    solverLogFile = '';
    diaryCleanup = [];
    if strcmp(caseDef.solverBackend, 'fatrop')
        solverLogFile = fullfile(resultDir, 'fatrop_solver_log.txt');
        diary(solverLogFile);
        diaryCleanup = onCleanup(@() diary('off'));
    end
    sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, ...
        'lbg', nlpData.lbg, 'ubg', nlpData.ubg);
    solveTime = toc(solveTimer);
    stats = nlpData.solver.stats();
    clear diaryCleanup;
    solverLogText = readTextIfExists(solverLogFile);
catch ME
    solveTime = toc(solveTimer);
    clear diaryCleanup;
    solverLogText = readTextIfExists(solverLogFile);
    row.solveTime_s = solveTime;
    row.totalTime_s = buildTime + solveTime;
    row.solverStatus = "FAILED_DURING_SOLVE";
    row.failureReason = string(compactMessage(ME.message));
    row.degenerateJacobianCount = countDegenerateJacobian(solverLogText);
    if strcmp(caseDef.solverBackend, 'fatrop')
        row.fatropStructureHealthy = row.degenerateJacobianCount <= 5;
    end
    fprintf('方法 %s 求解失败：%s\n', caseDef.label, row.failureReason);
    trial.row = row;
    return;
end

zOpt = full(sol.x);
fval = full(sol.f);
gOpt = full(sol.g);
eqMask = getEqualityMask(nlpData);
gEqOpt = gOpt(eqMask);
cOpt = gOpt(~eqMask);
data = evalTrajectory(zOpt);
traj = rebuildComparisonTrajectory(data, scene, disc);
denseReport = validateTrajectoryDenseComparison('IHSID', traj, data, model, scene, disc);
objectiveBreakdown = computeObjectiveBreakdownImplicit(data, model, scene, disc, initialGuess);
solverResult = struct('return_status', readStatus(stats), ...
    'success', readSuccess(stats), ...
    'iterations', readIterations(stats), 'solveTime', solveTime, 'fval', fval, ...
    'maxEqResidual', max(abs(gEqOpt(:))), 'maxIneqViolation', max([cOpt(:); 0]), ...
    'stats', stats);
row = fillSuccessRow(row, nlpData.sizes, solverResult, denseReport, ...
    objectiveBreakdown, buildTime, dynamicsEvalCountEstimate, nlpData, data, solverLogText);
trial.zOpt = zOpt;
trial.traj = traj;
trial.denseReport = denseReport;
trial.solverResult = solverResult;
trial.objectiveBreakdown = objectiveBreakdown;
trial.row = row;
end

function [z0, zIHS, initialGuess, evalTrajectory, dynamicsEvalCountEstimate] = prepareCase(caseDef, model, scene, disc)
[zIHS, initialGuess] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
if strcmp(caseDef.fatropStructure, 'manual')
    z0 = packIHSFatropManualDecision(zIHS, scene, disc);
    evalTrajectory = @(z) evaluateIHSFatropManualTrajectoryNumeric(z, model, scene, disc);
else
    z0 = zIHS;
    evalTrajectory = @(z) evaluateImplicitIHSTrajectoryNumeric(z, model, scene, disc);
end
dynamicsEvalCountEstimate = disc.numNodes + disc.numMidpoints;
end

function nlpData = buildCaseNlp(caseDef, model, scene, disc, initialGuess, solverOptions)
if strcmp(caseDef.fatropStructure, 'manual')
    nlpData = buildCasadiImplicitIHSFatropManualNLP(model, scene, disc, initialGuess, solverOptions);
else
    nlpData = buildCasadiImplicitIHSNLP(model, scene, disc, initialGuess, solverOptions);
end
end

function solverOptions = makeSolverOptions(caseDef)
solverOptions = struct('solverBackend', caseDef.solverBackend);
if strcmp(caseDef.solverBackend, 'ipopt')
    solverOptions.maxIter = 300;
    solverOptions.printLevel = 4;
    solverOptions.hessianApproximation = 'exact';
    solverOptions.acceptableTol = 1e-3;
    solverOptions.acceptableIter = 1;
else
    solverOptions.fatropStructure = caseDef.fatropStructure;
    solverOptions.maxIter = 30;
end
end

function mask = getEqualityMask(nlpData)
if isfield(nlpData, 'equalityMask')
    mask = logical(full(nlpData.equalityMask));
else
    mask = [true(nlpData.sizes.numEq, 1); false(nlpData.sizes.numIneq, 1)];
end
end

function row = baseRow(caseDef, disc)
row = struct();
row.method = string(caseDef.method);
row.caseName = string(caseDef.label);
row.solverBackend = string(caseDef.solverBackend);
row.fatropStructure = string(caseDef.fatropStructure);
row.N1 = disc.numIntervalsApproach;
row.N2 = disc.numIntervalsInsertion;
row.numVariables = NaN;
row.numZ = NaN;
row.numEq = NaN;
row.numIneq = NaN;
row.solverStatus = "";
row.solverSuccess = false;
row.engineeringPassed = false;
row.fatropComparisonValid = true;
row.fatropStructureHealthy = true;
row.buildTime_s = NaN;
row.solveTime_s = NaN;
row.totalTime_s = NaN;
row.objective = NaN;
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
row.objectiveStdAtX0 = NaN;
row.objectiveManualAtX0 = NaN;
row.absObjectiveDiffAtX0 = NaN;
row.maxManualGapResidualAtX0 = NaN;
row.maxManualMidResidualAtX0 = NaN;
row.maxManualHSResidualAtX0 = NaN;
row.maxManualDynResidualAtX0 = NaN;
row.minStage1Clearance_m = NaN;
row.finalGap_m = NaN;
row.maxDenseDynResidual = NaN;
row.maxHSDefectResidual = NaN;
row.maxMidConsistencyResidual = NaN;
row.maxStage2LateralError = NaN;
row.maxStage2HeightError = NaN;
row.maxStage2AttitudeError = NaN;
row.stage2Passed = false;
row.fatrop_iterations_count = NaN;
row.fatrop_eval_jac_count = NaN;
row.fatrop_eval_obj_count = NaN;
row.fatrop_eval_grad_count = NaN;
row.fatrop_eval_cv_count = NaN;
row.fatrop_return_flag = "";
row.degenerateJacobianCount = NaN;
row.failureReason = "";
end

function row = fillSuccessRow(row, sizes, solverResult, denseReport, objectiveBreakdown, ...
    buildTime, dynamicsEvalCountEstimate, nlpData, data, solverLogText) %#ok<INUSD>
stats = solverResult.stats;
row.numVariables = sizes.numZ;
row.numZ = sizes.numZ;
row.numEq = sizes.numEq;
row.numIneq = sizes.numIneq;
row.solverStatus = string(solverResult.return_status);
row.solverSuccess = solverResult.success;
row.engineeringPassed = isfield(denseReport, 'engineeringTrajectoryPassed') && denseReport.engineeringTrajectoryPassed;
row.buildTime_s = buildTime;
row.solveTime_s = solverResult.solveTime;
row.totalTime_s = buildTime + solverResult.solveTime;
row.objective = objectiveBreakdown.total;
row.iter_count = solverResult.iterations;
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
casadiEvalTime = sum([row.time_nlp_f, row.time_nlp_g, row.time_grad_f, row.time_jac_g, row.time_hess_l], 'omitnan');
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
row.fatrop_iterations_count = getStatSafe(stats, 'fatrop.iterations_count');
row.fatrop_eval_jac_count = getStatSafe(stats, 'fatrop.eval_jac_count');
row.fatrop_eval_obj_count = getStatSafe(stats, 'fatrop.eval_obj_count');
row.fatrop_eval_grad_count = getStatSafe(stats, 'fatrop.eval_grad_count');
row.fatrop_eval_cv_count = getStatSafe(stats, 'fatrop.eval_cv_count');
row.fatrop_return_flag = string(getStatSafe(stats, 'fatrop.return_flag', ""));
row.degenerateJacobianCount = countDegenerateJacobian(solverLogText);
if strcmp(string(row.solverBackend), "fatrop")
    row.fatropComparisonValid = true;
    row.fatropStructureHealthy = row.degenerateJacobianCount <= 5;
end
if ~row.engineeringPassed
    row.failureReason = sprintf(['求解完成但未通过统一工程后验验证：stage2Passed=%d, ', ...
        'maxDenseDynResidual=%.6g, minStage1Clearance_m=%.6g, finalGap_m=%.6g'], ...
        row.stage2Passed, row.maxDenseDynResidual, row.minStage1Clearance_m, row.finalGap_m);
end
end

function probe = probeFatropAvailability()
probe = struct('fatropPluginAvailable', false, 'fatropToySolvePassed', false, 'failureReason', "");
try
    probe.fatropPluginAvailable = casadi.has_nlpsol('fatrop');
catch ME
    probe.failureReason = string(compactMessage(ME.message));
    return;
end
if ~probe.fatropPluginAvailable
    probe.failureReason = "CasADi 鏈彂鐜?fatrop nlpsol 鎻掍欢";
    return;
end
try
    import casadi.*
    x0 = MX.sym('x0'); u0 = MX.sym('u0'); x1 = MX.sym('x1'); u1 = MX.sym('u1');
    z = [x0; u0; x1; u1];
    g = x1 - x0 - u0;
    nlp = struct('x', z, 'f', sumsqr(z), 'g', g);
    opts = struct();
    opts.structure_detection = 'manual';
    opts.N = 1;
    opts.nx = {1, 1};
    opts.nu = {1, 1};
    opts.ng = {0, 0};
    opts.equality = {true};
    opts.print_time = false;
    solver = nlpsol('toy_fatrop_manual_probe', 'fatrop', nlp, opts);
    sol = solver('x0', zeros(4, 1), 'lbx', -10*ones(4, 1), 'ubx', 10*ones(4, 1), ...
        'lbg', 0, 'ubg', 0); %#ok<NASGU>
    stats = solver.stats();
    probe.fatropToySolvePassed = isfield(stats, 'success') && stats.success;
catch ME
    probe.failureReason = string(compactMessage(ME.message));
end
end

function status = readStatus(stats)
rawStatus = getStatSafe(stats, 'return_status', "");
status = char(string(rawStatus));
if isempty(status)
    rawFlag = getStatSafe(stats, 'fatrop.return_flag', "");
    status = char(string(rawFlag));
end
end

function success = readSuccess(stats)
rawSuccess = getStatSafe(stats, 'success', false);
if islogical(rawSuccess) || isnumeric(rawSuccess)
    success = logical(rawSuccess);
else
    status = readStatus(stats);
    success = contains(status, 'Succeed') || contains(status, 'SUCCESS') || contains(status, 'Solve_Succeeded');
end
end

function iterations = readIterations(stats)
iterations = getStatSafe(stats, 'iter_count');
if isnan(iterations)
    iterations = getStatSafe(stats, 'fatrop.iterations_count');
end
end

function text = readTextIfExists(filePath)
text = "";
if ~isempty(filePath) && exist(filePath, 'file') == 2
    text = string(fileread(filePath));
end
end

function n = countDegenerateJacobian(logText)
if strlength(logText) == 0
    n = 0;
else
    n = count(logText, 'degenerate Jacobian');
end
end

function message = compactMessage(message)
message = regexprep(char(string(message)), '\s+', ' ');
if numel(message) > 800
    message = [message(1:800), ' ...'];
end
end

function saveTrialDetail(resultDir, trial)
detailDir = fullfile(resultDir, 'details');
if ~exist(detailDir, 'dir')
    mkdir(detailDir);
end
fileName = sprintf('%s_%s_%s_%dx%d.mat', char(trial.row.method), ...
    char(strrep(trial.row.solverBackend, ' ', '_')), ...
    char(strrep(trial.row.fatropStructure, ' ', '_')), ...
    trial.disc.numIntervalsApproach, trial.disc.numIntervalsInsertion);
detail = trial; %#ok<NASGU>
save(fullfile(detailDir, fileName), 'detail');
end

function savePartial(resultDir, model, scene, disc, manualStructureInfo, fatropProbe, trials, rows)
summaryTable = struct2table(rows);
writetable(summaryTable, fullfile(resultDir, 'comparison_summary_partial.csv'));
save(fullfile(resultDir, 'comparison_results_partial.mat'), ...
    'model', 'scene', 'disc', 'manualStructureInfo', 'fatropProbe', 'trials', 'summaryTable');
end

function writeReport(reportFile, summaryTable, manualStructureInfo, fatropProbe)
fid = fopen(reportFile, 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'IHSID-MANUAL-FATROP 20x10 fixed check report\n\n');
fprintf(fid, '%s\n\n', evalc('disp(summaryTable)'));
fprintf(fid, 'manual structure: N=%d, nx=%d, gapDim=%d, terminal stage uses minimal collision-certificate placeholder instead of full U_N.\n\n', ...
    manualStructureInfo.N, manualStructureInfo.nx, manualStructureInfo.gapDim);
fprintf(fid, 'FATROP plugin available: %s\n', yesNo(fatropProbe.fatropPluginAvailable));
fprintf(fid, 'FATROP toy solve passed: %s\n', yesNo(fatropProbe.fatropToySolvePassed));
fprintf(fid, 'IPOPT/FATROP Hessian mode: exact Hessian diagnostic comparison\n');
fprintf(fid, 'FATROP hessian_approximation limited-memory accepted: %s\n', yesNo(fatropProbe.hessianApproximationAccepted));
if strlength(fatropProbe.failureReason) > 0
    fprintf(fid, 'probe failureReason: %s\n', fatropProbe.failureReason);
end
fprintf(fid, '\nAnswers:\n');
writeQuestionAnswers(fid, summaryTable, fatropProbe);
end

function writeReadme(readmeFile, summaryTable, fatropProbe)
fid = fopen(readmeFile, 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# IHSID manual FATROP fixed check\n\n');
fprintf(fid, 'This directory is generated by `run_05_compare_IHSID_FATROP.m` for the 20x10 IHSID manual FATROP check only.\n\n');
fprintf(fid, '- fatropPluginAvailable: %s\n', yesNo(fatropProbe.fatropPluginAvailable));
fprintf(fid, '- fatropToySolvePassed: %s\n', yesNo(fatropProbe.fatropToySolvePassed));
fprintf(fid, '- hessianMode: exact Hessian diagnostic comparison\n');
fprintf(fid, '- limitedMemoryOptionAccepted: %s\n\n', yesNo(fatropProbe.hessianApproximationAccepted));
fprintf(fid, '```text\n%s\n```\n\n', evalc('disp(summaryTable)'));
fprintf(fid, '## Answers\n\n');
writeQuestionAnswers(fid, summaryTable, fatropProbe);
end

function writeQuestionAnswers(fid, summaryTable, fatropProbe)
standardIpopt = summaryTable(summaryTable.caseName == "IHSID-STANDARD-IPOPT", :);
manualIpopt = summaryTable(summaryTable.caseName == "IHSID-MANUAL-IPOPT", :);
manualFatrop = summaryTable(summaryTable.caseName == "IHSID-MANUAL-FATROP", :);
fatropInitialized = ~isempty(manualFatrop) && manualFatrop.solverStatus ~= "FAILED_AT_INITIALIZATION";
fatropSolved = ~isempty(manualFatrop) && manualFatrop.solverSuccess;
fatropEngineering = fatropSolved && manualFatrop.engineeringPassed;

fprintf(fid, '1. FATROP 插件是否可用：plugin=%s, toy solve=%s。\n', ...
    yesNo(fatropProbe.fatropPluginAvailable), yesNo(fatropProbe.fatropToySolvePassed));
fprintf(fid, '2. 本轮已把 IPOPT 也切到 exact Hessian；FATROP limited-memory option accepted=%s，仅作为接口信息记录。\n', ...
    yesNo(fatropProbe.hessianApproximationAccepted));
if ~isempty(manualIpopt) && ~isempty(standardIpopt)
    fprintf(fid, '3. manual IHSID 与 standard IHSID 是否公平：manual IPOPT solverSuccess=%s, engineeringPassed=%s, objectiveDiffAtX0=%.6g。\n', ...
        yesNo(manualIpopt.solverSuccess), yesNo(manualIpopt.engineeringPassed), manualIpopt.absObjectiveDiffAtX0);
else
    fprintf(fid, '3. manual IHSID 与 standard IHSID 是否公平：manual IPOPT 对比未完成。\n');
end
fprintf(fid, '4. IHSID-MANUAL-FATROP 是否初始化/求解/工程通过：init=%s, solve=%s, engineering=%s。\n', ...
    yesNo(fatropInitialized), yesNo(fatropSolved), yesNo(fatropEngineering));
if ~isempty(manualFatrop) && strlength(manualFatrop.failureReason) > 0
    fprintf(fid, '5. FATROP 失败位置：%s；failureReason=%s。\n', ...
        string(manualFatrop.solverStatus), string(manualFatrop.failureReason));
else
    fprintf(fid, '5. FATROP 失败位置：无失败原因记录。\n');
end
end

function text = yesNo(flag)
if flag
    text = '是';
else
    text = '否';
end
end

function disc = buildFatropDisc(scene, n1, n2)
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
    error('run_05_compare_IHSID_FATROP:NonUniformStep', '实验网格必须保持统一步长。');
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
