function result = writeMidtermReportFigures(comparisonMatFile, outputDir)
% writeMidtermReportFigures - 使用既有比较结果生成中期报告轨迹优化补图
%
% 输入：
%   comparisonMatFile - run_04 生成的 comparison_results.mat。
%   outputDir         - 图片输出目录；默认写入比较结果目录下的 midterm_report_figures。
%
% 输出：
%   result - 图片路径、数据来源和最终轨迹选择信息。
%
% 说明：
%   CHSID/IHSID 数据与最终轨迹来自 comparisonMatFile；CHSED 数值来自
%   《表4.docx》表1，仅用于复现中期报告中的方法演进对比，不代表当前
%   active 代码重新运行的结果。

if nargin < 1 || isempty(comparisonMatFile)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    comparisonMatFile = fullfile(projectRoot, 'opt_minimal', 'results', ...
        'compare_CHSID_IHSID_DMSID_20260531_125756', 'comparison_results.mat');
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(fileparts(comparisonMatFile), 'midterm_report_figures');
end
assert(isfile(comparisonMatFile), '比较结果文件不存在：%s', comparisonMatFile);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

loaded = load(comparisonMatFile, 'model', 'scene', 'trials', 'summaryTable');
requiredFields = {'model', 'scene', 'trials', 'summaryTable'};
assert(all(isfield(loaded, requiredFields)), '比较结果 MAT 缺少 model/scene/trials/summaryTable。');

model = loaded.model;
scene = loaded.scene;
trials = loaded.trials;
summaryTable = loaded.summaryTable;
methods = string(summaryTable.method);
assert(all(ismember(["CHSID", "IHSID"], methods)), '比较结果缺少 CHSID 或 IHSID。');

chsedData = struct( ...
    'source', "表4.docx 表1", ...
    'numVariables', 913, ...
    'numEq', 466, ...
    'numIneq', NaN, ...
    'solveTime_s', 462.39, ...
    'minClearance_m', -0.12438, ...
    'maxHSDefectResidual', 5.0022, ...
    'engineeringPassed', false);

defaultRows = summaryTable(summaryTable.N1 == 20 & summaryTable.N2 == 10 & ...
    ismember(string(summaryTable.method), ["CHSID", "IHSID"]), :);
assert(height(defaultRows) == 2, '图4 需要完整的 CHSID/IHSID 20+10 网格结果。');
defaultRows = orderRows(defaultRows, ["CHSID", "IHSID"]);
defaultTrialIndex = find(arrayfun(@(trial) trial.disc.numIntervalsApproach == 20 && ...
    trial.disc.numIntervalsInsertion == 10, trials), 1);
assert(~isempty(defaultTrialIndex), '未找到用于计算 CHSED 不等式约束数量的 20+10 离散结构。');
chsedData.numIneq = computeChsedNumIneq(trials(defaultTrialIndex).disc);

selectedIndex = find(arrayfun(@(trial) string(trial.method) == "IHSID" && ...
    trial.disc.numIntervalsApproach == 60 && trial.disc.numIntervalsInsertion == 30 && ...
    ~isempty(trial.traj), trials), 1);
assert(~isempty(selectedIndex), '未找到 IHSID 60+30 最终轨迹。');
bestTrial = trials(selectedIndex);
assert(bestTrial.row.solverSuccess && bestTrial.row.engineeringPassed, ...
    'IHSID 60+30 未同时通过求解与工程后验，不能作为最终最佳轨迹。');

figureBaseNames = ["fig4_method_performance_feasibility", "fig7_ihsid_engineering_constraints"];
pngFiles = strings(1, numel(figureBaseNames));
pdfFiles = strings(1, numel(figureBaseNames));
figureHandles = gobjects(1, numel(figureBaseNames));

fig = drawMethodPerformanceFigure(defaultRows, chsedData);
[pngFiles(1), pdfFiles(1)] = saveReportFigure(fig, outputDir, figureBaseNames(1));
figureHandles(1) = fig;
fig = drawEngineeringConstraintsFigure(bestTrial.traj, model, scene);
[pngFiles(2), pdfFiles(2)] = saveReportFigure(fig, outputDir, figureBaseNames(2));
figureHandles(2) = fig;

result = struct();
result.comparisonMatFile = string(comparisonMatFile);
result.outputDir = string(outputDir);
result.figureBaseNames = figureBaseNames;
result.pngFiles = pngFiles;
result.pdfFiles = pdfFiles;
result.figureHandles = figureHandles;
result.figureMethods = ["CHSED", "CHSID", "IHSID"];
result.methodNumIneq = [chsedData.numIneq; defaultRows.numIneq];
result.chsedData = chsedData;
result.selectedTrial = struct('method', "IHSID", 'N1', 60, 'N2', 30, ...
    'solverSuccess', logical(bestTrial.row.solverSuccess), ...
    'engineeringPassed', logical(bestTrial.row.engineeringPassed), ...
    'solveTime_s', bestTrial.row.solveTime_s, ...
    'minStage1Clearance_m', bestTrial.row.minStage1Clearance_m, ...
    'maxHSDefectResidual', bestTrial.row.maxHSDefectResidual);
end

function rows = orderRows(rows, methodOrder)
[~, index] = ismember(methodOrder, string(rows.method));
assert(all(index > 0), '方法结果不完整。');
rows = rows(index, :);
end

function numIneq = computeChsedNumIneq(disc)
numStage2Points = disc.numIntervalsInsertion + 1 + disc.numIntervalsInsertion;
numIneq = 36*(disc.numNodes + disc.numMidpoints) + ...
    numStage2Points + 10*disc.numCollisionCertificates;
end

function fig = drawMethodPerformanceFigure(rows, chsed)
fig = makeFigure('不同轨迹优化方法的求解性能与可行性比较');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
methods = {'CHSED', 'CHSID', 'IHSID'};
colors = [0.78 0.22 0.20; 0.22 0.45 0.72; 0.18 0.62 0.36];

ax = nexttile(layout);
values = [chsed.numVariables, chsed.numEq, chsed.numIneq; ...
    rows.numVariables, rows.numEq, rows.numIneq];
b = bar(ax, values, 'grouped');
b(1).FaceColor = [0.32 0.52 0.76];
b(2).FaceColor = [0.86 0.55 0.20];
b(3).FaceColor = [0.48 0.68 0.38];
styleCategoricalAxes(ax, methods, '数量');
title(ax, '(a) 优化变量与约束数量');
legend(ax, {'优化变量', '等式约束', '不等式约束'}, 'Location', 'northwest');
addGroupedBarLabels(ax, b, '%.0f');

ax = nexttile(layout);
values = [chsed.solveTime_s; rows.solveTime_s];
b = bar(ax, values, 0.58, 'FaceColor', 'flat');
b.CData = colors;
set(ax, 'YScale', 'log');
styleCategoricalAxes(ax, methods, '求解时间 / s');
title(ax, '(b) 求解时间');
addSingleBarLabels(ax, values, '%.2f s');

ax = nexttile(layout);
values = [chsed.minClearance_m; rows.minStage1Clearance_m];
b = bar(ax, values, 0.58, 'FaceColor', 'flat');
b.CData = colors;
yline(ax, 0, 'k-', 'LineWidth', 1.0, 'Label', '碰撞边界');
styleCategoricalAxes(ax, methods, '最小碰撞间隙 / m');
title(ax, '(c) 碰撞间隙');
addSingleBarLabels(ax, values, '%.5f m');

ax = nexttile(layout);
values = [chsed.maxHSDefectResidual; rows.maxHSDefectResidual];
b = bar(ax, values, 0.58, 'FaceColor', 'flat');
b.CData = colors;
set(ax, 'YScale', 'log');
styleCategoricalAxes(ax, methods, 'HS 离散残差');
grid(ax, 'off');
title(ax, '(d) HS 离散残差');
addSingleBarLabels(ax, values, '%.2e');
end

function fig = drawEngineeringConstraintsFigure(traj, model, scene)
fig = makeFigure('IHSID 最优轨迹的关键工程约束验证');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
phaseTime = scene.phase.durationApproach;

ax = nexttile(layout);
hold(ax, 'on');
plot(ax, traj.t, traj.collisionDistances.', '-', 'Color', [0.55 0.65 0.78], 'LineWidth', 0.9);
set(findobj(ax, 'Type', 'line'), 'HandleVisibility', 'off');
plot(ax, traj.tc, traj.collisionDistancesMid.', '.', 'Color', [0.72 0.78 0.86], ...
    'MarkerSize', 7, 'HandleVisibility', 'off');
plot(ax, traj.t, traj.minClearance, 'Color', [0.10 0.35 0.72], 'LineWidth', 2.2, 'DisplayName', '节点最小间隙');
plot(ax, traj.tc, traj.minClearanceMid, '--', 'Color', [0.90 0.42 0.12], 'LineWidth', 1.8, 'DisplayName', '中点最小间隙');
yline(ax, scene.collision.safeDistance, 'r--', '安全间隙', 'HandleVisibility', 'off');
yline(ax, scene.collision.finalGap, 'g:', '终端间隙', 'HandleVisibility', 'off');
styleTimeAxes(ax, phaseTime, '装备与障碍物间隙 / m', '(a) 碰撞安全间隙');
legend(ax, 'Location', 'best');

ax = nexttile(layout);
plotLegEnvelopePanel(ax, traj.t, traj.tc, traj.Unode, traj.Umid, ...
    model.actuator.forceMin(1), model.actuator.forceMax(1), phaseTime, ...
    '支腿驱动力 / N', '(b) 六条支腿驱动力');

ax = nexttile(layout);
plotLegEnvelopePanel(ax, traj.t, traj.tc, traj.Ldd, traj.LddMid, ...
    -model.actuator.lddotMax(1), model.actuator.lddotMax(1), phaseTime, ...
    '支腿加速度 / (m/s^2)', '(c) 六条支腿加速度');

ax = nexttile(layout);
hold(ax, 'on');
plot(ax, traj.t, traj.sigmaMin, 'Color', [0.10 0.35 0.72], 'LineWidth', 2.0, 'DisplayName', '节点');
plot(ax, traj.tc, traj.sigmaMinMid, '--', 'Color', [0.90 0.42 0.12], 'LineWidth', 1.6, 'DisplayName', '中点');
yline(ax, model.singularity.sigmaMinSafe, 'r--', '安全阈值', 'HandleVisibility', 'off');
[minValue, minIndex] = min([traj.sigmaMin, traj.sigmaMinMid]);
allTime = [traj.t, traj.tc];
plot(ax, allTime(minIndex), minValue, 'ro', 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
text(ax, allTime(minIndex), minValue, sprintf('  最小值 %.3f', minValue), 'VerticalAlignment', 'bottom');
styleTimeAxes(ax, phaseTime, '归一化雅可比最小奇异值', '(d) 奇异性安全裕度');
legend(ax, 'Location', 'best');

end

function plotLegEnvelopePanel(ax, t, tc, nodeValues, midValues, lowerBound, upperBound, phaseTime, yLabelText, titleText)
hold(ax, 'on');
plot(ax, t, nodeValues.', '-', 'Color', [0.68 0.75 0.84], 'LineWidth', 0.8, 'HandleVisibility', 'off');
plot(ax, tc, midValues.', '.', 'Color', [0.80 0.84 0.90], 'MarkerSize', 5, 'HandleVisibility', 'off');
nodeEnvelope = max(abs(nodeValues), [], 1);
midEnvelope = max(abs(midValues), [], 1);
plot(ax, t, nodeEnvelope, 'Color', [0.10 0.35 0.72], 'LineWidth', 2.0, 'DisplayName', '节点最不利包络');
plot(ax, tc, midEnvelope, '--', 'Color', [0.90 0.42 0.12], 'LineWidth', 1.7, 'DisplayName', '中点最不利包络');
yline(ax, lowerBound, 'k--', '下限', 'HandleVisibility', 'off');
yline(ax, upperBound, 'k--', '上限', 'HandleVisibility', 'off');
[peak, peakIndex] = max([nodeEnvelope, midEnvelope]);
allTime = [t, tc];
plot(ax, allTime(peakIndex), peak, 'ro', 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
text(ax, allTime(peakIndex), peak, sprintf('  峰值 %.3g', peak), 'VerticalAlignment', 'bottom');
styleTimeAxes(ax, phaseTime, yLabelText, titleText);
legend(ax, 'Location', 'best');
end

function styleTimeAxes(ax, phaseTime, yLabelText, titleText)
xline(ax, phaseTime, 'k-.', '阶段分界', 'HandleVisibility', 'off');
xlabel(ax, '时间 / s'); ylabel(ax, yLabelText); title(ax, titleText);
grid(ax, 'on'); box(ax, 'on');
end

function styleCategoricalAxes(ax, methods, yLabelText)
xticks(ax, 1:numel(methods)); xticklabels(ax, methods);
ylabel(ax, yLabelText); grid(ax, 'on'); box(ax, 'on');
end

function addGroupedBarLabels(ax, bars, formatText)
for groupIndex = 1:numel(bars)
    x = bars(groupIndex).XEndPoints;
    y = bars(groupIndex).YEndPoints;
    labels = arrayfun(@(value) sprintf(formatText, value), bars(groupIndex).YData, 'UniformOutput', false);
    text(ax, x, y, labels, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
end
end

function addSingleBarLabels(ax, values, formatText)
for index = 1:numel(values)
    if values(index) >= 0
        verticalAlignment = 'bottom';
    else
        verticalAlignment = 'top';
    end
    text(ax, index, values(index), sprintf(formatText, values(index)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', verticalAlignment, 'FontSize', 9);
end
end

function fig = makeFigure(name)
fig = figure('Name', name, 'Color', 'w', 'Visible', 'on', ...
    'Position', [80 80 1500 1050]);
set(fig, 'DefaultAxesFontName', 'Microsoft YaHei', 'DefaultTextFontName', 'Microsoft YaHei', ...
    'DefaultAxesFontSize', 11, 'DefaultTextFontSize', 11);
end

function [pngFile, pdfFile] = saveReportFigure(fig, outputDir, baseName)
pngFile = string(fullfile(outputDir, baseName + ".png"));
pdfFile = string(fullfile(outputDir, baseName + ".pdf"));
axesHandles = findall(fig, 'Type', 'axes');
for axesIndex = 1:numel(axesHandles)
    axesHandles(axesIndex).Toolbar.Visible = 'off';
end
exportgraphics(fig, pngFile, 'Resolution', 300, 'BackgroundColor', 'white');
exportgraphics(fig, pdfFile, 'ContentType', 'vector', 'BackgroundColor', 'white');
end
