function comparison = compareSolverResults(solverEntries, resultDir, timestamp, problemInfo)
% compareSolverResults - 汇总 IPOPT 与 fmincon-SQP 的求解结果并生成对比图
%
% 文件用途：
%   将不同求解器在同一 J、gEq、cIneq、lbz、ubz 上得到的结果整理为统一表格、
%   UTF-8 summary 文本和性能对比图。
%
% 输入：
%   solverEntries - struct 数组，每个元素含 solverName、solverResult、denseReport、
%                   objectiveBreakdown、result、iterationHistory 等字段
%   resultDir     - 输出目录
%   timestamp     - 时间戳
%   problemInfo   - 含 numIntervals、numZ、numEq、numIneq 等规模信息
%
% 输出：
%   comparison - 含 summaryFile、plotFiles、tableData 的比较结果结构
%
% 求解链路位置：
%   run_02_compare_ipopt_sqp 在两个求解器都完成后调用。
%
% 是否改变数学问题：
%   否。本函数只汇总和绘图。

if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

numSolvers = numel(solverEntries);
names = strings(numSolvers, 1);
solveTime = nan(numSolvers, 1);
iterations = nan(numSolvers, 1);
functionCount = nan(numSolvers, 1);
objectiveTotal = nan(numSolvers, 1);
objectiveForce = nan(numSolvers, 1);
objectiveLegAccel = nan(numSolvers, 1);
objectiveSingularity = nan(numSolvers, 1);
maxEq = nan(numSolvers, 1);
maxIneq = nan(numSolvers, 1);
minClearance = nan(numSolvers, 1);
minSigma = nan(numSolvers, 1);
maxCond = nan(numSolvers, 1);
nlpPassed = false(numSolvers, 1);
forcePassed = false(numSolvers, 1);
solverTrajectoryPassed = false(numSolvers, 1);
engineeringTrajectoryPassed = false(numSolvers, 1);

for index = 1:numSolvers
    entry = solverEntries(index);
    names(index) = string(entry.solverName);
    solveTime(index) = entry.solverResult.solveTime;
    iterations(index) = entry.solverResult.iterations;
    functionCount(index) = entry.solverResult.functionCount;
    objectiveTotal(index) = entry.objectiveBreakdown.total;
    objectiveForce(index) = entry.objectiveBreakdown.force;
    objectiveLegAccel(index) = entry.objectiveBreakdown.legAccel;
    objectiveSingularity(index) = entry.objectiveBreakdown.singularity;
    maxEq(index) = entry.solverResult.maxEqResidual;
    maxIneq(index) = entry.solverResult.maxIneqViolation;
    minClearance(index) = entry.denseReport.minClearance;
    minSigma(index) = entry.denseReport.minSigmaMin;
    maxCond(index) = entry.denseReport.maxCondJ;
    nlpPassed(index) = entry.result.nlpPassed;
    forcePassed(index) = entry.denseReport.forcePassed;
    solverTrajectoryPassed(index) = entry.result.solverTrajectoryPassed;
    engineeringTrajectoryPassed(index) = entry.result.engineeringTrajectoryPassed;
end

comparison = struct();
comparison.tableData = table(names, solveTime, iterations, functionCount, objectiveTotal, ...
    objectiveForce, objectiveLegAccel, objectiveSingularity, maxEq, maxIneq, ...
    minClearance, minSigma, maxCond, nlpPassed, forcePassed, ...
    solverTrajectoryPassed, engineeringTrajectoryPassed);
comparison.plotFiles = {};

comparison.plotFiles{end+1} = plotObjectiveBars(names, objectiveForce, objectiveLegAccel, ...
    objectiveSingularity, resultDir, timestamp);
comparison.plotFiles{end+1} = plotIterationHistory(solverEntries, resultDir, timestamp);
comparison.plotFiles{end+1} = plotConstraintHistory(solverEntries, resultDir, timestamp);
comparison.plotFiles{end+1} = plotTimeIterationBars(names, solveTime, iterations, functionCount, resultDir, timestamp);

summaryFile = fullfile(resultDir, ['summary_compare_ipopt_sqp_', timestamp, '.txt']);
writeSummary(summaryFile, solverEntries, problemInfo, comparison.tableData, comparison.plotFiles);
comparison.summaryFile = summaryFile;
end

function fileName = plotObjectiveBars(names, forceTerm, accelTerm, singTerm, resultDir, timestamp)
fig = figure('Name', 'solver_objective_comparison', 'Color', 'w');
bar(categorical(names), [forceTerm, accelTerm, singTerm], 'stacked');
grid on;
ylabel('objective value');
legend({'force', 'legAccel', 'singularity'}, 'Location', 'best');
title('solver objective comparison');
fileName = fullfile(resultDir, ['solver_objective_comparison_', timestamp, '.png']);
exportgraphics(fig, fileName, 'Resolution', 160);
close(fig);
end

function fileName = plotIterationHistory(solverEntries, resultDir, timestamp)
fig = figure('Name', 'solver_iteration_history', 'Color', 'w');
hold on; grid on;
for index = 1:numel(solverEntries)
    history = solverEntries(index).iterationHistory;
    if isstruct(history) && isfield(history, 'iteration') && ~isempty(history.iteration)
        plot(history.iteration, history.fval, 'o-', 'LineWidth', 1.1, ...
            'DisplayName', solverEntries(index).solverName);
    end
end
xlabel('iteration');
ylabel('objective');
title('iteration objective history; IPOPT per-iteration history is unavailable when absent');
legend('Location', 'best');
fileName = fullfile(resultDir, ['solver_iteration_history_', timestamp, '.png']);
exportgraphics(fig, fileName, 'Resolution', 160);
close(fig);
end

function fileName = plotConstraintHistory(solverEntries, resultDir, timestamp)
fig = figure('Name', 'solver_constraint_history', 'Color', 'w');
hold on; grid on;
for index = 1:numel(solverEntries)
    history = solverEntries(index).iterationHistory;
    if isstruct(history) && isfield(history, 'iteration') && ~isempty(history.iteration)
        semilogy(history.iteration, max(history.maxConstraintViolation, eps), 'o-', ...
            'LineWidth', 1.1, 'DisplayName', solverEntries(index).solverName);
    end
end
xlabel('iteration');
ylabel('max constraint violation');
title('constraint violation history; IPOPT history omitted if not available');
legend('Location', 'best');
fileName = fullfile(resultDir, ['solver_constraint_history_', timestamp, '.png']);
exportgraphics(fig, fileName, 'Resolution', 160);
close(fig);
end

function fileName = plotTimeIterationBars(names, solveTime, iterations, functionCount, resultDir, timestamp)
fig = figure('Name', 'solver_time_iteration_comparison', 'Color', 'w');
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; bar(categorical(names), solveTime); grid on; ylabel('seconds'); title('solve time');
nexttile; bar(categorical(names), iterations); grid on; ylabel('iterations'); title('iterations');
nexttile; bar(categorical(names), functionCount); grid on; ylabel('function evals'); title('function count');
fileName = fullfile(resultDir, ['solver_time_iteration_comparison_', timestamp, '.png']);
exportgraphics(fig, fileName, 'Resolution', 160);
close(fig);
end

function writeSummary(summaryFile, solverEntries, problemInfo, tableData, plotFiles)
fid = fopen(summaryFile, 'w', 'n', 'UTF-8');
if fid < 0
    error('compareSolverResults:CannotOpenSummary', '无法写入 summary：%s', summaryFile);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'Implicit HS shared NLP solver comparison summary\n');
fprintf(fid, 'numIntervals: %d\n', problemInfo.numIntervals);
fprintf(fid, 'numZ: %d\n', problemInfo.numZ);
fprintf(fid, 'numEq: %d\n', problemInfo.numEq);
fprintf(fid, 'numIneq: %d\n', problemInfo.numIneq);
if isfield(problemInfo, 'solverCount')
    fprintf(fid, 'solverCount: %d\n', problemInfo.solverCount);
end
fprintf(fid, '\n');

for index = 1:numel(solverEntries)
    entry = solverEntries(index);
    fprintf(fid, '=== %s ===\n', entry.solverName);
    fprintf(fid, 'initialValue: %s\n', entry.initialValueDescription);
    fprintf(fid, 'usesExactGradient: %d\n', entry.usesExactGradient);
    fprintf(fid, 'usesVariableScaling: %d\n', entry.usesVariableScaling);
    fprintf(fid, 'status: %s\n', entry.statusText);
    fprintf(fid, 'solveTime: %.6f\n', entry.solverResult.solveTime);
    fprintf(fid, 'iterations: %.0f\n', entry.solverResult.iterations);
    fprintf(fid, 'functionCount: %.0f\n', entry.solverResult.functionCount);
    fprintf(fid, 'objective total: %.12e\n', entry.objectiveBreakdown.total);
    fprintf(fid, 'objective force: %.12e\n', entry.objectiveBreakdown.force);
    fprintf(fid, 'objective legAccel: %.12e\n', entry.objectiveBreakdown.legAccel);
    fprintf(fid, 'objective singularity: %.12e\n', entry.objectiveBreakdown.singularity);
    fprintf(fid, 'max equality residual: %.12e\n', entry.solverResult.maxEqResidual);
    fprintf(fid, 'max inequality violation: %.12e\n', entry.solverResult.maxIneqViolation);
    fprintf(fid, 'force upper violation max: %.12e\n', entry.denseReport.forceUpperViolationMax);
    fprintf(fid, 'force lower violation max: %.12e\n', entry.denseReport.forceLowerViolationMax);
    fprintf(fid, 'min clearance: %.12e\n', entry.denseReport.minClearance);
    fprintf(fid, 'min sigmaMin: %.12e\n', entry.denseReport.minSigmaMin);
    fprintf(fid, 'max condJ: %.12e\n', entry.denseReport.maxCondJ);
    fprintf(fid, 'max r_kin: %.12e\n', entry.denseReport.maxKinematicResidual);
    fprintf(fid, 'max r_acc: %.12e\n', entry.denseReport.maxAccelConsistencyResidual);
    fprintf(fid, 'max r_dyn_state: %.12e\n', entry.denseReport.maxDynResidualState);
    fprintf(fid, 'max r_dyn_geom: %.12e\n', entry.denseReport.maxDynResidualGeometric);
    fprintf(fid, 'nlpPassed: %d\n', entry.result.nlpPassed);
    fprintf(fid, 'pathPassed: %d\n', entry.denseReport.pathPassed);
    fprintf(fid, 'forcePassed: %d\n', entry.denseReport.forcePassed);
    fprintf(fid, 'singularityPassed: %d\n', entry.denseReport.singularityPassed);
    fprintf(fid, 'kinematicsPassed: %d\n', entry.denseReport.kinematicsPassed);
    fprintf(fid, 'stateDynamicsPassed: %d\n', entry.denseReport.stateDynamicsPassed);
    fprintf(fid, 'geometricDynamicsPassed: %d\n', entry.denseReport.geometricDynamicsPassed);
    fprintf(fid, 'solverTrajectoryPassed: %d\n', entry.result.solverTrajectoryPassed);
    fprintf(fid, 'engineeringTrajectoryPassed: %d\n\n', entry.result.engineeringTrajectoryPassed);
end

fprintf(fid, 'Table:\n');
fprintf(fid, '%s\n\n', evalc('disp(tableData)'));
fprintf(fid, 'plots:\n');
for index = 1:numel(plotFiles)
    fprintf(fid, '  %s\n', plotFiles{index});
end
end
