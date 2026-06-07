function result = run_03_compare_hs_vs_dms_ipopt(varargin)
% run_03_compare_hs_vs_dms_ipopt - HS 与直接多重射击离散方法筛选实验入口
%
% 输入（名称-值）：
%   gridList      - 每行 [N1 N2]，默认 [20 10; 40 20; 60 30]。
%   makePlots     - 是否输出对比图，默认 true。
%   makeAnimation - 是否输出默认网格最佳方法动画，默认 true。
%
% 输出：
%   result - 保存结果目录、逐项 trial 结果和汇总行。
%
% 在实验链路中的作用：
%   在同一模型、同一场景、同一 IPOPT/MA27 下运行 HSI、HS-E、DMS-E，
%   并生成 MAT、CSV、TXT 和默认网格对比图。
parser = inputParser();
parser.addParameter('gridList', [20 10; 40 20; 60 30]);
parser.addParameter('makePlots', true);
parser.addParameter('makeAnimation', true);
parser.addParameter('maxIter', 300);
parser.addParameter('methods', {'HSI', 'HS-E', 'DMS-E'});
parser.addParameter('hessianApproximation', 'limited-memory');
parser.addParameter('acceptableTol', 1e-3);
parser.addParameter('acceptableIter', 1);
parser.parse(varargin{:});
gridList = parser.Results.gridList;
makePlots = parser.Results.makePlots;
makeAnimation = parser.Results.makeAnimation;
methods = normalizeMethodList(parser.Results.methods);
solverOptions = struct('maxIter', parser.Results.maxIter, 'printLevel', 4, ...
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
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ['discretization_compare_', timestamp]);
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
validMethods = {'HSI', 'HS-E', 'DMS-E'};
for i = 1:numel(methods)
    methods{i} = char(string(methods{i}));
    if ~ismember(methods{i}, validMethods)
        error('run_03_compare:UnknownMethod', '未知方法 %s。', methods{i});
    end
end
end

function savePartialComparison(resultDir, model, scene, trials, rows, gridList)
summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'comparison_summary_partial.csv');
writetable(summaryTable, csvFile);
matFile = fullfile(resultDir, 'comparison_results_partial.mat');
save(matFile, 'model', 'scene', 'trials', 'summaryTable', 'gridList');
end

function trial = runOneMethod(method, model, scene, disc, solverOptions)
trial = struct('method', method, 'disc', disc, 'zOpt', [], 'traj', [], 'denseReport', [], ...
    'solverResult', [], 'objectiveBreakdown', [], 'row', []);
row = baseRow(method, disc);
try
    switch method
        case 'HSI'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
            nlpData = buildCasadiImplicitHSNLP(model, scene, disc, initialGuess);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateImplicitTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = disc.numNodes + disc.numMidpoints;
        case 'HS-E'
            buildTimer = tic;
            [z0, initialGuess] = buildInitialGuessTwoPhaseReduced(model, scene, disc);
            nlpData = buildCasadiEliminatedAccelHSNLP(model, scene, disc, initialGuess, solverOptions);
            buildTime = toc(buildTimer);
            evalTrajectory = @(z) evaluateReducedHSTrajectoryNumeric(z, model, scene, disc);
            dynamicsEvalCountEstimate = disc.numNodes + disc.numMidpoints;
        case 'DMS-E'
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
        buildTime, dynamicsEvalCountEstimate);
    trial.zOpt = zOpt;
    trial.traj = traj;
    trial.denseReport = denseReport;
    trial.solverResult = solverResult;
    trial.objectiveBreakdown = objectiveBreakdown;
catch ME
    row.failureReason = string(ME.message);
    row.solverStatus = "FAILED_BEFORE_RESULT";
    fprintf('方法 %s 失败：%s\n', method, ME.message);
end
trial.row = row;
end

function row = baseRow(method, disc)
row = struct();
row.method = string(method);
row.N1 = disc.numIntervalsApproach;
row.N2 = disc.numIntervalsInsertion;
row.h = disc.h;
row.numZ = NaN;
row.numEq = NaN;
row.numIneq = NaN;
row.solverStatus = "";
row.solverSuccess = false;
row.engineeringPassed = false;
row.buildTime_s = NaN;
row.solveTime_s = NaN;
row.iterations = NaN;
row.objectiveTotal = NaN;
row.objectiveForce = NaN;
row.objectiveLegAccel = NaN;
row.objectiveSingularity = NaN;
row.maxEqResidual = NaN;
row.maxIneqViolation = NaN;
row.minStage1Clearance_m = NaN;
row.finalGap_m = NaN;
row.maxDynResidual = NaN;
row.maxDefectResidual = NaN;
row.minSigmaMin = NaN;
row.maxCondJ = NaN;
row.maxAbsLegSpeed = NaN;
row.maxAbsLegAccel = NaN;
row.maxAbsForce = NaN;
row.stage2Passed = false;
row.denseSampleCount = NaN;
row.dynamicsEvalCountEstimate = NaN;
row.failureReason = "";
end

function row = fillSuccessRow(row, sizes, solverResult, denseReport, objectiveBreakdown, buildTime, dynamicsEvalCountEstimate)
row.numZ = sizes.numZ;
row.numEq = sizes.numEq;
row.numIneq = sizes.numIneq;
row.solverStatus = string(solverResult.return_status);
row.solverSuccess = solverResult.success;
row.engineeringPassed = isfield(denseReport, 'engineeringTrajectoryPassed') && denseReport.engineeringTrajectoryPassed;
row.buildTime_s = buildTime;
row.solveTime_s = solverResult.solveTime;
row.iterations = solverResult.iterations;
row.objectiveTotal = objectiveBreakdown.total;
row.objectiveForce = objectiveBreakdown.forceRate + objectiveBreakdown.power;
row.objectiveLegAccel = objectiveBreakdown.legAccel;
row.objectiveSingularity = objectiveBreakdown.singularity;
row.maxEqResidual = solverResult.maxEqResidual;
row.maxIneqViolation = solverResult.maxIneqViolation;
row.minStage1Clearance_m = denseReport.minStage1Clearance;
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
row.stage2Passed = denseReport.stage2Passed;
row.denseSampleCount = denseReport.sampleCount;
row.dynamicsEvalCountEstimate = dynamicsEvalCountEstimate;
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
fprintf(fid, '说明：当前 HSI 基线包含三构件罩体碰撞证书，因此规模不同于旧单障碍提示词示例。\n\n');
fprintf(fid, '%s\n\n', evalc('disp(summaryTable)'));
fprintf(fid, '初步判断：%s\n', decideConclusion(summaryTable));
end

function conclusion = decideConclusion(T)
passed = T(T.engineeringPassed, :);
if isempty(passed)
    conclusion = '三种方法均未稳定通过当前硬碰撞统一验证，当前不足以判断离散方式优劣，应先检查可行性、初值或碰撞约束。';
    return;
end
defaultRows = T(T.N1 == 40 & T.N2 == 20, :);
if height(defaultRows) < 3
    conclusion = '当前结果未包含完整默认网格三方法，先以已完成网格作为 smoke 结果，不能给出最终离散方式结论。';
    return;
end
hse = defaultRows(defaultRows.method == "HS-E", :);
dms = defaultRows(defaultRows.method == "DMS-E", :);
hsi = defaultRows(defaultRows.method == "HSI", :);
if ~isempty(hse) && ~isempty(dms) && hse.engineeringPassed && ~dms.engineeringPassed
    conclusion = 'HS-E 通过而 DMS-E 未通过，HS 在当前近障硬约束问题中具有保留价值。';
elseif ~isempty(hse) && ~isempty(dms) && hse.engineeringPassed && dms.engineeringPassed && dms.solveTime_s < 0.75*hse.solveTime_s
    conclusion = 'DMS-E 与 HS-E 均通过且 DMS-E 明显更快，后续可优先直接多重射击路线。';
elseif ~isempty(hsi) && ~isempty(hse) && ~isempty(dms) && abs(hsi.objectiveTotal-hse.objectiveTotal) > abs(hse.objectiveTotal-dms.objectiveTotal)
    conclusion = 'HSI 与 HS-E 差异更大，主要收益可能来自消去加速度变量，而不是 HS 离散本身。';
else
    conclusion = '当前默认网格结果未显示 DMS-E 对 HS-E 的明确优势，HS 仍有保留研究价值。';
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
