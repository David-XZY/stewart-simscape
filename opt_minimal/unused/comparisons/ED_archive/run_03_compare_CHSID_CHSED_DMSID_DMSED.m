function result = run_03_compare_CHSID_CHSED_DMSID_DMSED(varargin)
% run_03_compare_CHSID_CHSED_DMSID_DMSED - 四组离散/动力学表达对比实验入口
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
%   当前默认只运行 CHSID 和 DMSID；CHSED、DMSED 可通过 methods 参数手动启用。
%   运行后生成 MAT、CSV、TXT，并可选生成默认网格对比图。
parser = inputParser();
parser.addParameter('gridList', [20 10; 40 20; 60 30; 80 40]);
parser.addParameter('makePlots', false);
parser.addParameter('makeAnimation', false);
parser.addParameter('maxIter', 300);
parser.addParameter('printLevel', 4);
% 临时只默认启用 CHSID 和 DMSID；CHSED/DMSED 仍可通过 methods 参数手动指定。
parser.addParameter('methods', {'CHSID', 'DMSID'});
parser.addParameter('hessianApproximation', 'limited-memory');
parser.addParameter('acceptableTol', 1e-3);
parser.addParameter('acceptableIter', 1);
parser.parse(varargin{:});
gridList = parser.Results.gridList;
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
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ['four_method_compare_', timestamp]);
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
readmeFile = fullfile(resultDir, 'README_four_method_compare.md');
writeFourMethodReadme(readmeFile, summaryTable, solverOptions);
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
validMethods = {'CHSID', 'CHSED', 'DMSID', 'DMSED'};
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
    case 'HS-E'
        method = 'CHSED';
    case 'DMS-E'
        method = 'DMSED';
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
fileName = sprintf('%s_N%d_%d_detail.mat', methodName, ...
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
        case 'CHSED'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseReduced(model, scene, disc);
            nlpData = buildCasadiEliminatedAccelHSNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateReducedHSTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = disc.numNodes + disc.numMidpoints;
        case 'DMSID'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
            nlpData = buildCasadiImplicitDMSNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateImplicitDMSTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = 8*disc.numIntervals + disc.numNodes + disc.numMidpoints;
        case 'DMSED'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseReduced(model, scene, disc);
            nlpData = buildCasadiMultipleShootingNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateDMSTrajectoryNumeric(z, model, scene, disc);
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
        buildTime, dynamicsEvalCountEstimate, nlpData.opts);
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
row.solveTime = NaN;
row.iterations = NaN;
row.iterCount = NaN;
row.objectiveTotal = NaN;
row.objective = NaN;
row.objectiveForce = NaN;
row.objectiveLegAccel = NaN;
row.objectiveSingularity = NaN;
row.maxEqResidual = NaN;
row.maxIneqViolation = NaN;
row.minStage1Clearance_m = NaN;
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
row.maxForceViolation = NaN;
row.minSigma = NaN;
row.maxStage2LateralError = NaN;
row.maxStage2HeightError = NaN;
row.maxStage2AttitudeError = NaN;
row.maxInsertionBackwardSpeedViolation = NaN;
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

function row = fillSuccessRow(row, sizes, solverResult, denseReport, objectiveBreakdown, buildTime, dynamicsEvalCountEstimate, opts)
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
row.solveTime = solverResult.solveTime;
row.iterations = solverResult.iterations;
row.iterCount = solverResult.iterations;
row.objectiveTotal = objectiveBreakdown.total;
row.objective = objectiveBreakdown.total;
row.objectiveForce = objectiveBreakdown.forceRate + objectiveBreakdown.power;
row.objectiveLegAccel = objectiveBreakdown.legAccel;
row.objectiveSingularity = objectiveBreakdown.singularity;
row.maxEqResidual = solverResult.maxEqResidual;
row.maxIneqViolation = solverResult.maxIneqViolation;
row.minStage1Clearance_m = denseReport.minStage1Clearance;
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
row.maxForceViolation = max(denseReport.forceUpperViolationMax, denseReport.forceLowerViolationMax);
row.minSigma = denseReport.minSigmaMin;
row.maxStage2LateralError = denseReport.stage2MaxLateralError;
row.maxStage2HeightError = denseReport.stage2MaxHeightError;
row.maxStage2AttitudeError = denseReport.stage2MaxAttitudeError;
row.maxInsertionBackwardSpeedViolation = max(-denseReport.stage2MinInsertionSpeed, 0);
row.stage2Passed = denseReport.stage2Passed;
row.denseSampleCount = denseReport.sampleCount;
row.dynamicsEvalCountEstimate = dynamicsEvalCountEstimate;
row.ipoptLinearSolver = string(opts.ipopt.linear_solver);
row.ipoptHessianApproximation = string(opts.ipopt.hessian_approximation);
row.ipoptTol = opts.ipopt.tol;
row.ipoptConstrViolTol = opts.ipopt.constr_viol_tol;
row.ipoptMaxIter = opts.ipopt.max_iter;
if ~row.engineeringPassed
    row.failureReason = "求解完成但未通过统一工程后验验证";
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

function writeFourMethodReadme(readmeFile, summaryTable, solverOptions)
% writeFourMethodReadme - 输出四方法实验说明
fid = fopen(readmeFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
opts = makeCommonIpoptOptions(solverOptions);
fprintf(fid, '# 四组离散/动力学表达对比实验\n\n');
fprintf(fid, '本目录由 `run_03_compare_CHSID_CHSED_DMSID_DMSED.m` 生成。\n\n');
fprintf(fid, '## 方法命名\n\n');
fprintf(fid, '当前默认只运行 CHSID 和 DMSID；CHSED、DMSED 暂时不在默认实验队列中。\n\n');
fprintf(fid, '- CHSID: Compressed Hermite-Simpson + Implicit Dynamics。旧名 HSI。\n');
fprintf(fid, '- CHSED: Compressed Hermite-Simpson + Explicit/Solved Dynamics。旧名 HS-E。\n');
fprintf(fid, '- DMSID: Direct Multiple Shooting + Implicit Dynamics。本轮新增。\n');
fprintf(fid, '- DMSED: Direct Multiple Shooting + Explicit/Solved Dynamics。旧名 DMS-E。\n\n');
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
fprintf(fid, '四组方法均通过 `makeCommonIpoptOptions` 生成设置；若 exact Hessian 无法稳定运行，应统一切换，不允许单独救活某一种方法。\n\n');
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
defaultRows = T(T.N1 == 40 & T.N2 == 20, :);
requiredDefaultMethods = ["CHSID"; "DMSID"];
if isempty(defaultRows) || ~all(ismember(requiredDefaultMethods, string(defaultRows.method)))
    conclusion = '当前结果未包含默认网格的 CHSID/DMSID 两个默认启用方法，先以已完成网格作为 smoke 结果，不能给出最终离散方式结论。';
    return;
end
chsid = defaultRows(defaultRows.method == "CHSID", :);
chsed = defaultRows(defaultRows.method == "CHSED", :);
dmsid = defaultRows(defaultRows.method == "DMSID", :);
dmsed = defaultRows(defaultRows.method == "DMSED", :);
if ~isempty(chsed) && ~isempty(dmsed) && chsed.engineeringPassed && ~dmsed.engineeringPassed
    conclusion = 'CHSED 通过而 DMSED 未通过，CHS 在当前近障硬约束问题中具有保留价值。';
elseif ~isempty(chsed) && ~isempty(dmsed) && chsed.engineeringPassed && dmsed.engineeringPassed && dmsed.solveTime_s < 0.75*chsed.solveTime_s
    conclusion = 'DMSED 与 CHSED 均通过且 DMSED 明显更快，后续可优先直接多重射击路线。';
elseif ~isempty(chsid) && ~isempty(chsed) && ~isempty(dmsed) && abs(chsid.objectiveTotal-chsed.objectiveTotal) > abs(chsed.objectiveTotal-dmsed.objectiveTotal)
    conclusion = 'CHSID 与 CHSED 差异更大，主要收益可能来自消去加速度变量，而不是 CHS 离散本身。';
elseif ~isempty(chsid) && ~isempty(dmsid) && chsid.engineeringPassed && dmsid.engineeringPassed && dmsid.solveTime_s < 0.75*chsid.solveTime_s
    conclusion = '默认启用的 CHSID 与 DMSID 均通过且 DMSID 明显更快，后续可继续评估直接多重射击路线。';
else
    conclusion = '当前默认网格结果未显示 DMS 系列对 CHS 系列的明确优势，CHS 仍有保留研究价值。';
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
