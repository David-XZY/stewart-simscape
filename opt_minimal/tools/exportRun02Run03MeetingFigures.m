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
    run02MatFile = latestResultFile(resultRoot, ...
        'result_simscape_pose_force_control_*.mat');
end
if strlength(string(run03MatFile)) == 0
    run03MatFile = latestRun03Baseline(resultRoot);
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
[figFiles{4}, pngFiles{4}, rippleDiagnostics] = ...
    exportRippleDiagnostics(run02, run03, outputDir, colors);

result = struct();
result.run02MatFile = run02MatFile;
result.run03MatFile = run03MatFile;
result.outputDir = outputDir;
result.figFiles = figFiles;
result.pngFiles = pngFiles;
result.run02Metrics = run02.report.metrics;
result.run03Metrics = run03.report.metrics;
result.rippleDiagnostics = rippleDiagnostics;
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
sgtitle(layout, 'Run02：逆动力学前馈 + 位姿动态反馈（力输入、重力开启）', ...
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
run03Label = 'Run03（默认参数）';
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
characteristicLength = 0.5;
r2Equivalent = [r2.poseError(:, 1:3), characteristicLength * r2.poseError(:, 4:6)];
r3Equivalent = [r3.poseError(:, 1:3), characteristicLength * r3.poseError(:, 4:6)];
r2EquivalentRms = sqrt(mean(r2Equivalent.^2, 'all'));
r3EquivalentRms = sqrt(mean(r3Equivalent.^2, 'all'));
r2EquivalentPeak = max(abs(r2Equivalent), [], 'all');
r3EquivalentPeak = max(abs(r3Equivalent), [], 'all');
values = [ ...
    r2EquivalentRms / r3EquivalentRms, ...
    r2EquivalentPeak / r3EquivalentPeak, ...
    r2.metrics.maxTranslationPeak / r3.metrics.maxTranslationPeak, ...
    r2.metrics.maxRotationPeak / r3.metrics.maxRotationPeak; ...
    1, 1, 1, 1];
bars = bar(ax, values.', 'grouped');
bars(1).FaceColor = colors.run02;
bars(2).FaceColor = colors.run03;
hold(ax, 'on');
yline(ax, 1, '--', 'Color', colors.threshold, 'LineWidth', 1.3);
set(ax, 'XTickLabel', {'等效位姿 RMS', '等效位姿峰值', '平移峰值', '转角峰值'});
ylabel(ax, 'Run02 / Run03'); title(ax, '位姿综合指标对比');
legend(ax, {'Run02', run03Label, '默认基准'}, 'Location', 'best');
styleAxes(ax); ylim(ax, [0, 1.15]);
actualLabels = {
    sprintf('%.3f mm', r2EquivalentRms * 1e3), ...
    sprintf('%.3f mm', r2EquivalentPeak * 1e3), ...
    sprintf('%.3f mm', r2.metrics.maxTranslationPeak * 1e3), ...
    sprintf('%.3f deg', rad2deg(r2.metrics.maxRotationPeak)); ...
    sprintf('%.3f mm', r3EquivalentRms * 1e3), ...
    sprintf('%.3f mm', r3EquivalentPeak * 1e3), ...
    sprintf('%.3f mm', r3.metrics.maxTranslationPeak * 1e3), ...
    sprintf('%.3f deg', rad2deg(r3.metrics.maxRotationPeak))};
addBarLabels(ax, bars, values, actualLabels);

addFooter(fig, ['说明：Run02 为重力开启的力输入位姿轨迹跟踪；Run03 为重力关闭的纯长度伺服。', ...
    '综合比较以位姿误差为主，Run02 同时执行总力与腿运动约束验收。']);
[figFile, pngFile] = saveReopenExport(fig, outputDir, 'run02_run03_tracking_comparison');
end

function [figFile, pngFile, diagnostics] = exportRippleDiagnostics(run02, run03, outputDir, colors)
% exportRippleDiagnostics - 绘制 Run02/Run03 局部误差与 Run03 纹波来源诊断
r2 = run02.report;
r3 = run03.report;
diagnostics = analyzeRipple(run03);
legIndex = diagnostics.legIndex;
zoomWindow = diagnostics.zoomWindow;
inZoom3 = r3.time >= zoomWindow(1) & r3.time <= zoomWindow(2);
inPoseZoom3 = r3.poseTime >= zoomWindow(1) & r3.poseTime <= zoomWindow(2);
inZoom2 = r2.time >= zoomWindow(1) & r2.time <= zoomWindow(2);
inPoseZoom2 = r2.poseTime >= zoomWindow(1) & r2.poseTime <= zoomWindow(2);

fig = makeFigure('Run02 / Run03 纹波诊断');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layout.Position = [0.045 0.085 0.93 0.84];
sgtitle(layout, sprintf('Run02 / Run03 纹波诊断：第 %d 腿，局部窗口 %.3f–%.3f s', ...
    legIndex, zoomWindow(1), zoomWindow(2)), 'FontSize', 18, 'FontWeight', 'bold');

ax = nexttile(layout);
plot(ax, r2.time(inZoom2), r2.lengthError(inZoom2, legIndex) * 1e3, ...
    'Color', colors.run02, 'LineWidth', 1.4);
hold(ax, 'on');
plot(ax, r3.time(inZoom3), r3.lengthError(inZoom3, legIndex) * 1e3, ...
    'Color', colors.run03, 'LineWidth', 1.4);
styleAxes(ax); title(ax, sprintf('局部腿长误差：第 %d 腿', legIndex));
ylabel(ax, '误差 (mm)'); legend(ax, {'Run02', 'Run03（默认参数）'}, 'Location', 'best');

characteristicLength = 0.5;
r2Equivalent = max(abs([r2.poseError(:, 1:3), ...
    characteristicLength * r2.poseError(:, 4:6)]), [], 2) * 1e3;
r3Equivalent = max(abs([r3.poseError(:, 1:3), ...
    characteristicLength * r3.poseError(:, 4:6)]), [], 2) * 1e3;
ax = nexttile(layout);
plot(ax, r2.poseTime(inPoseZoom2), r2Equivalent(inPoseZoom2), ...
    'Color', colors.run02, 'LineWidth', 1.4);
hold(ax, 'on');
plot(ax, r3.poseTime(inPoseZoom3), r3Equivalent(inPoseZoom3), ...
    'Color', colors.run03, 'LineWidth', 1.4);
styleAxes(ax); title(ax, '局部等效位姿误差');
ylabel(ax, '等效误差 (mm)'); legend(ax, {'Run02', 'Run03（默认参数）'}, 'Location', 'best');

actualSpeedTime = signalTime(r3.time, run03.setup.config.sampleTime, size(r3.LdActual, 1));
inActualZoom = actualSpeedTime >= zoomWindow(1) & actualSpeedTime <= zoomWindow(2);
pCorrection = r3.lengthError(:, legIndex) * run03.setup.design.Kpos(legIndex);
ax = nexttile(layout);
plot(ax, r3.time(inZoom3), r3.Ldref(inZoom3, legIndex), '--', ...
    'Color', colors.reference, 'LineWidth', 1.3);
hold(ax, 'on');
plot(ax, r3.time(inZoom3), pCorrection(inZoom3), ':', ...
    'Color', colors.run03, 'LineWidth', 1.5);
plot(ax, r3.time(inZoom3), r3.LdCmd(inZoom3, legIndex), ...
    'Color', colors.command, 'LineWidth', 1.4);
plot(ax, actualSpeedTime(inActualZoom), r3.LdActual(inActualZoom, legIndex), ...
    'Color', colors.actual, 'LineWidth', 1.4);
styleAxes(ax); title(ax, 'Run03 腿速信号链');
ylabel(ax, '腿速 / P 修正 (m/s)');
legend(ax, {'Ldref', 'P 修正', 'LdCmd', 'LdActual'}, 'Location', 'best', 'NumColumns', 2);

ax = nexttile(layout);
plot(ax, diagnostics.frequencyHz, diagnostics.normalizedSpectrum, ...
    'Color', colors.run03, 'LineWidth', 1.6);
hold(ax, 'on');
nodeLine = xline(ax, diagnostics.referenceNodeFrequencyHz, '--', 'Color', colors.reference, ...
    'LineWidth', 1.3, 'DisplayName', sprintf('参考节点 %.1f Hz', diagnostics.referenceNodeFrequencyHz));
innerLine = xline(ax, diagnostics.innerBandwidthHz, '--', 'Color', colors.command, ...
    'LineWidth', 1.3, 'DisplayName', sprintf('速度内环 %.1f Hz', diagnostics.innerBandwidthHz));
rippleLine = xline(ax, diagnostics.dominantFrequencyHz, ':', 'Color', colors.threshold, ...
    'LineWidth', 1.5, 'DisplayName', sprintf('主纹波 %.2f Hz', diagnostics.dominantFrequencyHz));
styleAxes(ax); title(ax, 'Run03 腿长误差高频残差频谱');
xlabel(ax, '频率 (Hz)'); ylabel(ax, '归一化幅值'); xlim(ax, [0, diagnostics.spectrumUpperHz]);
legend(ax, [rippleLine, nodeLine, innerLine], 'Location', 'northeast');

if abs(diagnostics.dominantFrequencyHz - diagnostics.referenceNodeFrequencyHz) <= 1
    conclusion = sprintf(['主纹波 %.2f Hz 与参考节点 %.1f Hz 接近，', ...
        '节点化参考仍是主要周期激励。'], ...
        diagnostics.dominantFrequencyHz, diagnostics.referenceNodeFrequencyHz);
else
    conclusion = sprintf(['参考节点 %.1f Hz 已不再主导误差频谱，残余主频转为 %.2f Hz；', ...
        '节点频率归一化幅值为 %.3f。'], ...
        diagnostics.referenceNodeFrequencyHz, diagnostics.dominantFrequencyHz, ...
        diagnostics.nodeFrequencyAmplitudeNormalized);
end
addFooter(fig, ['诊断：外环 P 修正跟随腿长误差，不是独立自激振荡源；', conclusion]);
[figFile, pngFile] = saveReopenExport(fig, outputDir, 'run02_run03_ripple_diagnostics');
end

function diagnostics = analyzeRipple(run03)
% analyzeRipple - 自动选择高频纹波最明显的 Run03 支链和局部时间窗
report = run03.report;
sampleTime = run03.setup.config.sampleTime;
time = (report.time(1):sampleTime:report.time(end)).';
regularLengthError = interp1(report.time, report.lengthError, time, 'linear');
sampleFrequencyHz = 1 / sampleTime;
trendWindow = max(3, round(0.5 * sampleFrequencyHz));
energyWindow = max(3, round(1.5 * sampleFrequencyHz));
highFrequencyResidual = regularLengthError - movmean(regularLengthError, trendWindow, 1);
[~, legIndex] = max(sqrt(mean(highFrequencyResidual.^2, 1)));
localEnergy = movmean(highFrequencyResidual(:, legIndex).^2, energyWindow);
[~, centerIndex] = max(localEnergy);
zoomWindow = [max(time(1), time(centerIndex) - 0.75), ...
    min(time(end), time(centerIndex) + 0.75)];

residual = highFrequencyResidual(:, legIndex) - mean(highFrequencyResidual(:, legIndex));
sampleCount = numel(residual);
spectrum = abs(fft(residual));
frequencyHz = (0:sampleCount - 1).' * sampleFrequencyHz / sampleCount;
spectrumUpperHz = min(20, 0.5 * sampleFrequencyHz);
keep = frequencyHz <= spectrumUpperHz;
frequencyHz = frequencyHz(keep);
spectrum = spectrum(keep);
searchMask = frequencyHz >= 0.5;
[~, dominantIndex] = max(spectrum(searchMask));
searchFrequency = frequencyHz(searchMask);
dominantFrequencyHz = searchFrequency(dominantIndex);
normalizedSpectrum = spectrum / max(spectrum);

diagnostics = struct();
diagnostics.legIndex = legIndex;
diagnostics.zoomWindow = zoomWindow;
diagnostics.dominantFrequencyHz = dominantFrequencyHz;
if isfield(run03.setup.refs, 'nodeTime')
    nodeTime = run03.setup.refs.nodeTime(:);
else
    nodeTime = run03.setup.refs.t(:);
end
diagnostics.referenceNodeFrequencyHz = 1 / median(diff(nodeTime));
diagnostics.innerBandwidthHz = run03.setup.design.innerBandwidthHz;
diagnostics.sampleFrequencyHz = sampleFrequencyHz;
diagnostics.spectrumUpperHz = spectrumUpperHz;
diagnostics.frequencyHz = frequencyHz;
diagnostics.normalizedSpectrum = normalizedSpectrum;
[~, nodeFrequencyIndex] = min(abs(frequencyHz - diagnostics.referenceNodeFrequencyHz));
diagnostics.nodeFrequencyAmplitudeNormalized = normalizedSpectrum(nodeFrequencyIndex);
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

function fileName = latestResultFile(resultRoot, pattern)
% latestResultFile - 返回指定模式的最新结果文件
files = dir(fullfile(resultRoot, pattern));
assert(~isempty(files), '未找到结果文件：%s', pattern);
[~, index] = max([files.datenum]);
fileName = fullfile(files(index).folder, files(index).name);
end

function fileName = latestRun03Baseline(resultRoot)
% latestRun03Baseline - 返回最新的 Run03 默认参数基准
files = dir(fullfile(resultRoot, 'result_simscape_length_cascade_*.mat'));
[~, order] = sort([files.datenum], 'descend');
files = files(order);
for index = 1:numel(files)
    candidateFile = fullfile(files(index).folder, files(index).name);
    candidate = load(candidateFile, 'setup', 'report');
    if isfield(candidate, 'setup') && isfield(candidate, 'report') && candidate.report.passed && ...
            isfield(candidate.setup.design, 'positionGainScale') && ...
            isfield(candidate.setup.design, 'velocityGainScale') && ...
            abs(candidate.setup.design.positionGainScale - 1) < 1e-12 && ...
            abs(candidate.setup.design.velocityGainScale - 0.7) < 1e-12
        fileName = candidateFile;
        return;
    end
end
error('exportRun02Run03MeetingFigures:BaselineNotFound', ...
    '未找到通过硬验收的 Run03 默认参数基准结果。');
end
