function result = exportRun02Run03MeetingFigures(run02MatFile, run03MatFile, outputDir)
% exportRun02Run03MeetingFigures - 导出 Run02/Run03 组会汇报图
arguments
    run02MatFile {mustBeTextScalar} = ""
    run03MatFile {mustBeTextScalar} = ""
    outputDir {mustBeTextScalar} = ""
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
resultRoot = fullfile(optRoot, 'results');
if strlength(string(run02MatFile)) == 0
    run02MatFile = fullfile(resultRoot, ...
        'result_simscape_length_control_20260610_215007.mat');
end
if strlength(string(run03MatFile)) == 0
    run03MatFile = fullfile(resultRoot, ...
        'result_simscape_length_cascade_20260610_220059.mat');
end
if strlength(string(outputDir)) == 0
    outputDir = fullfile(resultRoot, 'group_meeting_run02_run03_20260611');
end
run02MatFile = char(run02MatFile);
run03MatFile = char(run03MatFile);
outputDir = char(outputDir);

assert(isfile(run02MatFile), 'Run02 结果文件不存在：%s', run02MatFile);
assert(isfile(run03MatFile), 'Run03 结果文件不存在：%s', run03MatFile);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

run02 = load(run02MatFile);
run03 = load(run03MatFile);
validateResultData(run02, run03);

colors = struct( ...
    'run02', [0.00 0.35 0.70], ...
    'run03', [0.90 0.40 0.05], ...
    'threshold', [0.80 0.10 0.10], ...
    'reference', [0.35 0.35 0.35], ...
    'command', [0.55 0.20 0.70], ...
    'actual', [0.00 0.55 0.35]);

[figFiles{1}, pngFiles{1}] = exportRun02Overview(run02, outputDir, colors);
[figFiles{2}, pngFiles{2}] = exportRun03Overview(run03, outputDir, colors);
[figFiles{3}, pngFiles{3}] = exportComparison(run02, run03, outputDir, colors);

result = struct();
result.run02MatFile = run02MatFile;
result.run03MatFile = run03MatFile;
result.outputDir = outputDir;
result.figFiles = figFiles;
result.pngFiles = pngFiles;
result.run02Metrics = run02.report.metrics;
result.run03Metrics = run03.report.metrics;
end

function validateResultData(run02, run03)
% validateResultData - 校验绘图所需的最终验收结果和信号
assert(isfield(run02, 'report') && run02.report.passed, ...
    'Run02 结果必须通过硬验收。');
assert(isfield(run03, 'report') && run03.report.passed && isfield(run03, 'setup'), ...
    'Run03 结果必须通过硬验收并包含 setup。');
requiredRun02 = {'time', 'poseTime', 'forceTime', 'feedbackForceTime', ...
    'feedforwardForceTime', 'lengthError', 'poseError', 'controlForce', ...
    'feedbackForce', 'feedforwardForce', 'metrics'};
requiredRun03 = {'time', 'poseTime', 'commandTime', 'lengthError', ...
    'poseError', 'Ldref', 'LdCmd', 'LdActual', 'servoCommand', 'metrics'};
assert(all(isfield(run02.report, requiredRun02)), 'Run02 结果缺少组会绘图信号。');
assert(all(isfield(run03.report, requiredRun03)), 'Run03 结果缺少组会绘图信号。');
end

function [figFile, pngFile] = exportRun02Overview(run02, outputDir, colors)
% exportRun02Overview - 绘制力驱动控制总览
report = run02.report;
fig = makeFigure('Run02 力驱动控制总览');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layout.Position = [0.045 0.085 0.93 0.84];
sgtitle(layout, 'Run02：力前馈 + 长度反馈（重力开启）', ...
    'FontSize', 18, 'FontWeight', 'bold');

ax = nexttile(layout);
plotLegErrors(ax, report.time, report.lengthError * 1e3);
addSymmetricThreshold(ax, 5, colors.threshold);
title(ax, '六腿长度跟踪误差'); ylabel(ax, '误差 (mm)');

ax = nexttile(layout);
plot(ax, report.poseTime, report.poseError(:, 1:3) * 1e3, 'LineWidth', 1.2);
styleAxes(ax); title(ax, '三轴平移跟踪误差'); ylabel(ax, '误差 (mm)');
legend(ax, {'x', 'y', 'z'}, 'Location', 'best', 'NumColumns', 3);

ax = nexttile(layout);
plot(ax, report.poseTime, rad2deg(report.poseError(:, 4:6)), 'LineWidth', 1.2);
styleAxes(ax); title(ax, '三轴转角跟踪误差'); ylabel(ax, '误差 (deg)');
legend(ax, {'roll', 'pitch', 'yaw'}, 'Location', 'best', 'NumColumns', 3);

[~, forceLeg] = max(max(abs(report.controlForce), [], 1));
ax = nexttile(layout);
plot(ax, report.forceTime, report.controlForce(:, forceLeg), ...
    'Color', colors.run02, 'LineWidth', 1.6);
hold(ax, 'on');
plot(ax, report.feedforwardForceTime, report.feedforwardForce(:, forceLeg), ...
    '--', 'Color', colors.reference, 'LineWidth', 1.3);
plot(ax, report.feedbackForceTime, report.feedbackForce(:, forceLeg), ...
    ':', 'Color', colors.command, 'LineWidth', 1.5);
yline(ax, 2000, '--', 'Color', colors.threshold, 'LineWidth', 1.1);
yline(ax, -2000, '--', 'Color', colors.threshold, 'LineWidth', 1.1);
styleAxes(ax); title(ax, sprintf('最大受力支链：第 %d 腿', forceLeg));
ylabel(ax, '控制力 (N)');
legend(ax, {'总控制力', '前馈力', '反馈力', '±2000 N 限制'}, ...
    'Location', 'best', 'NumColumns', 2);

annotationText = sprintf(['硬验收：通过  |  最大腿长误差 %.3f mm  |  ', ...
    '最大平移误差 %.3f mm  |  最大转角误差 %.3f deg  |  最大总控制力 %.1f N'], ...
    report.metrics.maxLengthTrackingPeak * 1e3, ...
    report.metrics.maxTranslationPeak * 1e3, ...
    rad2deg(report.metrics.maxRotationPeak), report.metrics.maxAbsControlForce);
addFooter(fig, annotationText);
[figFile, pngFile] = saveReopenExport(fig, outputDir, 'run02_force_control_overview');
end

function [figFile, pngFile] = exportRun03Overview(run03, outputDir, colors)
% exportRun03Overview - 绘制纯长度串级控制总览
report = run03.report;
setup = run03.setup;
[positionGainScale, velocityGainScale] = getGainScales(setup.design);
fig = makeFigure('Run03 纯长度串级控制总览');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layout.Position = [0.045 0.085 0.93 0.84];
sgtitle(layout, 'Run03：纯长度串级控制（重力关闭）', ...
    'FontSize', 18, 'FontWeight', 'bold');

ax = nexttile(layout);
plotLegErrors(ax, report.time, report.lengthError * 1e3);
addSymmetricThreshold(ax, 5, colors.threshold);
title(ax, '六腿长度跟踪误差'); ylabel(ax, '误差 (mm)');

ax = nexttile(layout);
plot(ax, report.poseTime, report.poseError(:, 1:3) * 1e3, 'LineWidth', 1.2);
styleAxes(ax); title(ax, '三轴平移跟踪误差'); ylabel(ax, '误差 (mm)');
legend(ax, {'x', 'y', 'z'}, 'Location', 'best', 'NumColumns', 3);

ax = nexttile(layout);
plot(ax, report.poseTime, rad2deg(report.poseError(:, 4:6)), 'LineWidth', 1.2);
styleAxes(ax); title(ax, '三轴转角跟踪误差'); ylabel(ax, '误差 (deg)');
legend(ax, {'roll', 'pitch', 'yaw'}, 'Location', 'best', 'NumColumns', 3);

[~, speedLeg] = max(max(abs(report.Ldref), [], 1));
actualSpeedTime = signalTime(report.time, setup.config.sampleTime, ...
    size(report.LdActual, 1));
ax = nexttile(layout);
plot(ax, report.time, report.Ldref(:, speedLeg), '--', ...
    'Color', colors.reference, 'LineWidth', 1.4);
hold(ax, 'on');
plot(ax, report.time, report.LdCmd(:, speedLeg), ...
    'Color', colors.command, 'LineWidth', 1.4);
plot(ax, actualSpeedTime, report.LdActual(:, speedLeg), ...
    'Color', colors.actual, 'LineWidth', 1.5);
plot(ax, report.commandTime, report.servoCommand(:, speedLeg), ':', ...
    'Color', colors.run03, 'LineWidth', 1.4);
styleAxes(ax); title(ax, sprintf('最大参考腿速支链：第 %d 腿', speedLeg));
ylabel(ax, '腿速 / 命令 (m/s)');
legend(ax, {'Ldref', 'LdCmd', 'LdActual', 'servoCommand'}, ...
    'Location', 'best', 'NumColumns', 2);

annotationText = sprintf(['硬验收：通过  |  Ts=%.3f s  |  内/外环=%.1f/%.1f Hz  |  位置/速度增益缩放=%.2f/%.2f  |  ', ...
    '最大腿长误差 %.3f mm  |  最大腿速 %.3f m/s  |  最大腿加速度 %.3f m/s^2'], ...
    setup.config.sampleTime, setup.design.innerBandwidthHz, ...
    setup.design.outerBandwidthHz, positionGainScale, ...
    velocityGainScale, report.metrics.maxLengthTrackingPeak * 1e3, ...
    report.metrics.maxAbsLegSpeed, report.metrics.maxAbsLegAcceleration);
addFooter(fig, annotationText);
[figFile, pngFile] = saveReopenExport(fig, outputDir, 'run03_length_cascade_overview');
end

function [figFile, pngFile] = exportComparison(run02, run03, outputDir, colors)
% exportComparison - 绘制两种控制器的共同跟踪指标对比
r2 = run02.report;
r3 = run03.report;
commonTime = (max([r2.time(1), r3.time(1)]):0.01: ...
    min([r2.time(end), r3.time(end)])).';

r2Length = interp1(r2.time, max(abs(r2.lengthError), [], 2) * 1e3, commonTime);
r3Length = interp1(r3.time, max(abs(r3.lengthError), [], 2) * 1e3, commonTime);
r2Translation = interp1(r2.poseTime, ...
    max(abs(r2.poseError(:, 1:3)), [], 2) * 1e3, commonTime);
r3Translation = interp1(r3.poseTime, ...
    max(abs(r3.poseError(:, 1:3)), [], 2) * 1e3, commonTime);
r2Rotation = interp1(r2.poseTime, ...
    rad2deg(max(abs(r2.poseError(:, 4:6)), [], 2)), commonTime);
r3Rotation = interp1(r3.poseTime, ...
    rad2deg(max(abs(r3.poseError(:, 4:6)), [], 2)), commonTime);

fig = makeFigure('Run02 / Run03 跟踪性能对比');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layout.Position = [0.045 0.085 0.93 0.84];
[~, velocityGainScale] = getGainScales(run03.setup.design);
run03Label = sprintf('Run03（速度环×%.2f）', velocityGainScale);
sgtitle(layout, sprintf('Run02 / %s 共同跟踪指标对比', run03Label), ...
    'FontSize', 18, 'FontWeight', 'bold');

ax = nexttile(layout);
plotComparisonSeries(ax, commonTime, r2Length, r3Length, 5, ...
    '逐时刻最大腿长误差', '误差 (mm)', colors, run03Label);
ax = nexttile(layout);
plotComparisonSeries(ax, commonTime, r2Translation, r3Translation, 10, ...
    '逐时刻最大平移轴误差', '误差 (mm)', colors, run03Label);
ax = nexttile(layout);
plotComparisonSeries(ax, commonTime, r2Rotation, r3Rotation, 1, ...
    '逐时刻最大转角轴误差', '误差 (deg)', colors, run03Label);

ax = nexttile(layout);
values = [ ...
    r2.metrics.maxLengthTrackingPeak / 5e-3, ...
    r2.metrics.maxTranslationPeak / 10e-3, ...
    r2.metrics.maxRotationPeak / deg2rad(1); ...
    r3.metrics.maxLengthTrackingPeak / 5e-3, ...
    r3.metrics.maxTranslationPeak / 10e-3, ...
    r3.metrics.maxRotationPeak / deg2rad(1)];
bars = bar(ax, values.', 'grouped');
bars(1).FaceColor = colors.run02;
bars(2).FaceColor = colors.run03;
hold(ax, 'on');
yline(ax, 1, '--', 'Color', colors.threshold, 'LineWidth', 1.3);
set(ax, 'XTickLabel', {'腿长误差', '平移误差', '转角误差'});
ylabel(ax, '峰值 / 硬阈值'); title(ax, '硬指标阈值占用率');
legend(ax, {'Run02', run03Label, '硬阈值'}, 'Location', 'best');
styleAxes(ax); ylim(ax, [0, 1.15]);
actualLabels = {
    sprintf('%.3f mm', r2.metrics.maxLengthTrackingPeak * 1e3), ...
    sprintf('%.3f mm', r2.metrics.maxTranslationPeak * 1e3), ...
    sprintf('%.3f deg', rad2deg(r2.metrics.maxRotationPeak)); ...
    sprintf('%.3f mm', r3.metrics.maxLengthTrackingPeak * 1e3), ...
    sprintf('%.3f mm', r3.metrics.maxTranslationPeak * 1e3), ...
    sprintf('%.3f deg', rad2deg(r3.metrics.maxRotationPeak))};
addBarLabels(ax, bars, values, actualLabels);

addFooter(fig, ['说明：Run02 为重力开启的力驱动；Run03 为重力关闭的纯长度伺服。', ...
    '对比仅展示控制特性，不能直接作为执行器能力优劣结论。']);
[figFile, pngFile] = saveReopenExport(fig, outputDir, 'run02_run03_tracking_comparison');
end

function fig = makeFigure(name)
% makeFigure - 创建适合 16:9 PPT 的白底图
fig = figure('Name', name, 'Color', 'w', 'Visible', 'off', ...
    'Units', 'pixels', 'Position', [80, 80, 1600, 900], ...
    'PaperPositionMode', 'auto');
set(fig, 'DefaultAxesFontName', 'Microsoft YaHei', ...
    'DefaultTextFontName', 'Microsoft YaHei', ...
    'DefaultAxesFontSize', 11);
end

function plotLegErrors(ax, time, errors)
% plotLegErrors - 绘制六腿误差
plot(ax, time, errors, 'LineWidth', 1.15);
styleAxes(ax);
legend(ax, compose('腿%d', 1:6), 'Location', 'best', 'NumColumns', 3);
end

function plotComparisonSeries(ax, time, run02Value, run03Value, threshold, titleText, yLabelText, colors, run03Label)
% plotComparisonSeries - 绘制统一阈值下的两种控制器时序对比
plot(ax, time, run02Value, 'Color', colors.run02, 'LineWidth', 1.6);
hold(ax, 'on');
plot(ax, time, run03Value, 'Color', colors.run03, 'LineWidth', 1.6);
yline(ax, threshold, '--', 'Color', colors.threshold, 'LineWidth', 1.2);
styleAxes(ax); title(ax, titleText); ylabel(ax, yLabelText);
legend(ax, {'Run02', run03Label, '硬阈值'}, 'Location', 'best');
end

function [positionGainScale, velocityGainScale] = getGainScales(design)
% getGainScales - 兼容缩放参数加入前保存的 Run03 结果
positionGainScale = 1;
velocityGainScale = 1;
if isfield(design, 'positionGainScale')
    positionGainScale = design.positionGainScale;
end
if isfield(design, 'velocityGainScale')
    velocityGainScale = design.velocityGainScale;
end
end

function addSymmetricThreshold(ax, threshold, color)
% addSymmetricThreshold - 添加正负硬阈值
hold(ax, 'on');
yline(ax, threshold, '--', 'Color', color, 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
yline(ax, -threshold, '--', 'Color', color, 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
end

function styleAxes(ax)
% styleAxes - 统一投影友好的坐标轴样式
grid(ax, 'on');
box(ax, 'on');
xlabel(ax, '时间 (s)');
ax.GridAlpha = 0.18;
ax.LineWidth = 0.8;
end

function addFooter(fig, textValue)
% addFooter - 在图底部添加关键结论或公平性说明
annotation(fig, 'textbox', [0.025 0.008 0.95 0.025], ...
    'String', textValue, 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'FontSize', 9, ...
    'FontWeight', 'normal', 'Interpreter', 'none');
end

function addBarLabels(ax, bars, values, actualLabels)
% addBarLabels - 标注归一化指标数值
for seriesIndex = 1:numel(bars)
    x = bars(seriesIndex).XEndPoints;
    y = bars(seriesIndex).YEndPoints;
    labels = strings(1, numel(x));
    for valueIndex = 1:numel(x)
        labels(valueIndex) = sprintf('%.2f | %s', ...
            values(seriesIndex, valueIndex), actualLabels{seriesIndex, valueIndex});
    end
    text(ax, x, y + 0.025, labels, 'HorizontalAlignment', 'center', ...
        'FontSize', 8.5, 'FontWeight', 'bold');
end
end

function time = signalTime(referenceTime, sampleTime, sampleCount)
% signalTime - 重建未单独记录时间戳的规则采样信号时间
candidate = (referenceTime(1):sampleTime:referenceTime(end)).';
if numel(candidate) == sampleCount
    time = candidate;
else
    time = linspace(referenceTime(1), referenceTime(end), sampleCount).';
end
end

function [figFile, pngFile] = saveReopenExport(fig, outputDir, baseName)
% saveReopenExport - 保存 FIG，关闭后重新打开并导出 300 DPI PNG
figFile = fullfile(outputDir, [baseName, '.fig']);
pngFile = fullfile(outputDir, [baseName, '.png']);
drawnow;
savefig(fig, figFile);
close(fig);
reopenedFig = openfig(figFile, 'invisible');
cleanup = onCleanup(@() close(reopenedFig));
set(reopenedFig, 'Units', 'pixels', 'Position', [80, 80, 1600, 900]);
exportgraphics(reopenedFig, pngFile, 'Resolution', 300, ...
    'BackgroundColor', 'white', 'Width', 16, 'Height', 9, ...
    'Units', 'inches', 'Padding', 'figure', 'PreserveAspectRatio', 'off');
fprintf('组会 FIG 已保存：%s\n', figFile);
fprintf('组会 PNG 已保存：%s\n', pngFile);
end
