function result = run_06_compare_HS_FATROP_NATIVE(varargin)
% run_06_compare_HS_FATROP_NATIVE - 对比 FATROP-native HS 的 IPOPT 与 FATROP 求解
parser = inputParser();
parser.addParameter('grid', [20 10]);
parser.addParameter('fatropMaxIter', 100);
parser.addParameter('ipoptMaxIter', 300);
parser.parse(varargin{:});

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
grid = parser.Results.grid;
disc = buildNativeDisc(scene, grid(1), grid(2));
[z0, initialGuess] = buildInitialGuessFatropNativeHS(model, scene, disc);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ...
    ['compare_HS_FATROP_NATIVE_', timestamp]);
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

cases = { ...
    struct('label', 'NATIVE-HS-IPOPT', 'solverBackend', 'ipopt', 'maxIter', parser.Results.ipoptMaxIter), ...
    struct('label', 'NATIVE-HS-FATROP', 'solverBackend', 'fatrop', 'maxIter', parser.Results.fatropMaxIter)};
rows = struct([]);
trials = struct([]);
for caseIndex = 1:numel(cases)
    trial = runOneNativeCase(cases{caseIndex}, model, scene, disc, initialGuess, z0, resultDir);
    trials = [trials; trial]; %#ok<AGROW>
    rows = [rows; trial.row]; %#ok<AGROW>
    save(fullfile(resultDir, sprintf('%s_detail.mat', cases{caseIndex}.label)), 'trial');
end

summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results.mat');
save(matFile, 'model', 'scene', 'disc', 'initialGuess', 'trials', 'summaryTable');
reportFile = fullfile(resultDir, 'comparison_report.txt');
writeNativeReport(reportFile, summaryTable);

result = struct('resultDir', resultDir, 'summaryTable', summaryTable, ...
    'trials', trials, 'csvFile', csvFile, 'matFile', matFile, 'reportFile', reportFile);
fprintf('\nFATROP-native HS 结果目录：%s\n', resultDir);
end

function trial = runOneNativeCase(caseDef, model, scene, disc, initialGuess, z0, resultDir)
fprintf('\n===== %s N1=%d N2=%d =====\n', caseDef.label, disc.numIntervalsApproach, disc.numIntervalsInsertion);
row = baseRow(caseDef, disc);
trial = struct('caseDef', caseDef, 'zOpt', [], 'traj', [], 'denseReport', [], ...
    'solverResult', [], 'row', row);

buildTimer = tic;
try
    solverOptions = makeNativeSolverOptions(caseDef);
    nlpData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, solverOptions);
    buildTime = toc(buildTimer);
catch ME
    row.buildTime_s = toc(buildTimer);
    row.totalTime_s = row.buildTime_s;
    row.solverStatus = "FAILED_AT_INITIALIZATION";
    row.failureReason = string(compactMessage(ME.message));
    trial.row = row;
    return;
end

row.buildTime_s = buildTime;
row.solverBackend = string(nlpData.solverBackend);
row.fatropStructure = string(nlpData.fatropStructure);
row.numVariables = nlpData.sizes.numZ;
row.numEq = nlpData.sizes.numEq;
row.numIneq = nlpData.sizes.numIneq;

solverLogFile = '';
diaryCleanup = []; %#ok<NASGU>
try
    if strcmp(caseDef.solverBackend, 'fatrop')
        solverLogFile = fullfile(resultDir, 'fatrop_native_solver_log.txt');
        diary(solverLogFile);
        diaryCleanup = onCleanup(@() diary('off'));
    end
    solveTimer = tic;
    sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, ...
        'lbg', nlpData.lbg, 'ubg', nlpData.ubg);
    solveTime = toc(solveTimer);
    stats = nlpData.solver.stats();
    clear diaryCleanup;
    solverLogText = readTextIfExists(solverLogFile);
catch ME
    clear diaryCleanup;
    solverLogText = readTextIfExists(solverLogFile);
    row.solveTime_s = toc(solveTimer);
    row.totalTime_s = row.buildTime_s + row.solveTime_s;
    row.solverStatus = "FAILED_DURING_SOLVE";
    row.failureReason = string(compactMessage(ME.message));
    row.degenerateJacobianCount = countDegenerateJacobian(solverLogText);
    row.fatropStructureHealthy = row.degenerateJacobianCount <= 5;
    trial.row = row;
    return;
end

zOpt = full(sol.x);
gOpt = full(sol.g);
eqMask = logical(full(nlpData.equalityMask));
data = evaluateFatropNativeHSTrajectoryNumeric(zOpt, model, scene, disc);
traj = rebuildComparisonTrajectory(data, scene, disc);
denseReport = validateTrajectoryDenseComparison('FATROP_NATIVE_HS', traj, data, model, scene, disc);
solverResult = struct('return_status', readStatus(stats), 'success', readSuccess(stats), ...
    'iterations', readIterations(stats), 'solveTime', solveTime, 'fval', full(sol.f), ...
    'maxEqResidual', max(abs(gOpt(eqMask))), 'maxIneqViolation', max([gOpt(~eqMask); 0]), ...
    'stats', stats);
row = fillNativeSuccessRow(row, nlpData, solverResult, denseReport, data, solverLogText);
trial.zOpt = zOpt;
trial.traj = traj;
trial.denseReport = denseReport;
trial.solverResult = solverResult;
trial.row = row;
end

function solverOptions = makeNativeSolverOptions(caseDef)
solverOptions = struct('solverBackend', caseDef.solverBackend, 'maxIter', caseDef.maxIter);
if strcmp(caseDef.solverBackend, 'ipopt')
    solverOptions.printLevel = 4;
    solverOptions.hessianApproximation = 'exact';
    solverOptions.acceptableTol = 1e-3;
    solverOptions.acceptableIter = 1;
else
    solverOptions.fatropStructure = 'manual';
end
end

function row = baseRow(caseDef, disc)
row = struct();
row.caseName = string(caseDef.label);
row.method = "FATROP_NATIVE_HS";
row.solverBackend = string(caseDef.solverBackend);
row.fatropStructure = "";
row.N1 = disc.numIntervalsApproach;
row.N2 = disc.numIntervalsInsertion;
row.numVariables = NaN;
row.numEq = NaN;
row.numIneq = NaN;
row.solverStatus = "";
row.solverSuccess = false;
row.engineeringPassed = false;
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
row.time_hess_l = NaN;
row.maxEqResidual = NaN;
row.maxIneqViolation = NaN;
row.minStage1Clearance_m = NaN;
row.finalGap_m = NaN;
row.maxDenseDynResidual = NaN;
row.maxHSDefectResidual = NaN;
row.stage2Passed = false;
row.fatrop_return_flag = "";
row.degenerateJacobianCount = NaN;
row.failureReason = "";
end

function row = fillNativeSuccessRow(row, nlpData, solverResult, denseReport, data, solverLogText)
stats = solverResult.stats;
row.numVariables = nlpData.sizes.numZ;
row.numEq = nlpData.sizes.numEq;
row.numIneq = nlpData.sizes.numIneq;
row.solverStatus = string(solverResult.return_status);
row.solverSuccess = solverResult.success;
row.engineeringPassed = isfield(denseReport, 'engineeringTrajectoryPassed') && denseReport.engineeringTrajectoryPassed;
row.solveTime_s = solverResult.solveTime;
row.totalTime_s = row.buildTime_s + row.solveTime_s;
row.objective = solverResult.fval;
row.iter_count = solverResult.iterations;
row.n_eval_f = getStatSafe(stats, 'n_call_nlp_f');
row.n_eval_g = getStatSafe(stats, 'n_call_nlp_g');
row.n_eval_grad_f = getStatSafe(stats, 'n_call_nlp_grad_f');
row.n_eval_jac_g = getStatSafe(stats, 'n_call_nlp_jac_g');
row.n_eval_hess_l = getStatSafe(stats, 'n_call_nlp_hess_l');
row.time_hess_l = getStatSafe(stats, 't_proc_nlp_hess_l');
row.maxEqResidual = solverResult.maxEqResidual;
row.maxIneqViolation = solverResult.maxIneqViolation;
row.minStage1Clearance_m = denseReport.minStage1Clearance;
row.finalGap_m = denseReport.finalGap;
if isfield(denseReport, 'maxDynResidual')
    row.maxDenseDynResidual = denseReport.maxDynResidual;
else
    row.maxDenseDynResidual = denseReport.maxDynResidualGeometric;
end
row.maxHSDefectResidual = data.maxDefectResidual;
row.stage2Passed = denseReport.stage2Passed;
row.fatrop_return_flag = string(getStatSafe(stats, 'fatrop.return_flag', ""));
row.degenerateJacobianCount = countDegenerateJacobian(solverLogText);
if strcmp(string(row.solverBackend), "fatrop")
    row.fatropStructureHealthy = row.degenerateJacobianCount <= 5;
end
if ~row.engineeringPassed
    row.failureReason = string(sprintf(['求解完成但未通过统一工程后验验证：stage2Passed=%d, ', ...
        'maxDenseDynResidual=%.6g, minStage1Clearance_m=%.6g, finalGap_m=%.6g'], ...
        row.stage2Passed, row.maxDenseDynResidual, row.minStage1Clearance_m, row.finalGap_m));
end
end

function writeNativeReport(reportFile, summaryTable)
fid = fopen(reportFile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'FATROP-native HS comparison report\n\n');
disp(summaryTable);
fprintf(fid, '%s\n', evalc('disp(summaryTable)'));
for i = 1:height(summaryTable)
    fprintf(fid, '%s: solverSuccess=%d, engineeringPassed=%d, degenerateJacobianCount=%g, objective=%.12g\n', ...
        string(summaryTable.caseName(i)), summaryTable.solverSuccess(i), summaryTable.engineeringPassed(i), ...
        summaryTable.degenerateJacobianCount(i), summaryTable.objective(i));
end
end

function disc = buildNativeDisc(scene, n1, n2)
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
    error('run_06_compare_HS_FATROP_NATIVE:NonUniformStep', '实验网格必须保持统一步长。');
end
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage1NodeIndices = 1:(n1 + 1);
disc.stage2NodeIndices = (n1 + 1):disc.numNodes;
disc.stage1MidIndices = 1:n1;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end

function status = readStatus(stats)
status = char(string(getStatSafe(stats, 'return_status', "")));
if strlength(string(status)) == 0
    status = char(string(getStatSafe(stats, 'fatrop.return_flag', "")));
end
end

function success = readSuccess(stats)
success = isfield(stats, 'success') && stats.success;
end

function iterations = readIterations(stats)
iterations = getStatSafe(stats, 'iter_count');
if isnan(iterations)
    iterations = getStatSafe(stats, 'fatrop.iterations_count');
end
end

function value = getStatSafe(stats, fieldPath, defaultValue)
if nargin < 3
    defaultValue = NaN;
end
value = defaultValue;
parts = split(string(fieldPath), '.');
current = stats;
for i = 1:numel(parts)
    key = char(parts(i));
    if isstruct(current) && isfield(current, key)
        current = current.(key);
    else
        return;
    end
end
value = current;
end

function n = countDegenerateJacobian(textValue)
if strlength(string(textValue)) == 0
    n = 0;
else
    n = numel(regexp(char(textValue), 'degenerate Jacobian', 'match'));
end
end

function textValue = readTextIfExists(fileName)
textValue = "";
if ~isempty(fileName) && exist(fileName, 'file')
    textValue = string(fileread(fileName));
end
end

function msg = compactMessage(msg)
msg = regexprep(char(msg), '\s+', ' ');
end
