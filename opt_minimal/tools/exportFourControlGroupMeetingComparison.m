function result = exportFourControlGroupMeetingComparison(run02File, pwmFile, outputDir)
% exportFourControlGroupMeetingComparison - 导出四种控制方案的中文组会对比图
arguments
    run02File {mustBeTextScalar} = ""
    pwmFile {mustBeTextScalar} = ""
    outputDir {mustBeTextScalar} = ""
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
resultRoot = fullfile(optRoot, 'results');
run02File = latestFile(run02File, resultRoot, 'result_simscape_pose_force_control_*.mat');
pwmFile = latestFile(pwmFile, resultRoot, 'pwm_pose_force_comparison_*.mat');
if strlength(string(outputDir)) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(resultRoot, ['four_control_group_meeting_comparison_', timestamp]);
end
outputDir = char(outputDir);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

run02 = load(run02File);
pwm = load(pwmFile);
runs = buildAlignedRuns(run02, pwm);
validateRuns(runs, run02, pwm);
[summaryMetrics, perAxisMetrics] = calculateMetrics(runs, pwm);

writeChineseTable(summaryMetrics(:, 2:end), fullfile(outputDir, '四方案总体指标.csv'), ...
    {'方案', '平移RMS_mm', '平移峰值_mm', '旋转RMS_deg', '旋转峰值_deg', ...
    '终点平移误差_mm', '终点旋转误差_deg', '输出力RMS_N', '输出力峰值_N', ...
    'PWM适用', 'PWM峰值', 'PWM饱和比例', 'PWM变化率RMS_每秒'});
writeChineseTable(perAxisMetrics(:, 2:end), fullfile(outputDir, '四方案分轴指标.csv'), ...
    {'方案', '自由度', '单位', 'RMS', '峰值'});

figureNames = {
    '01_四方案核心性能总览'
    '02_六自由度分轴误差对比'
    '03_轨迹与关键时刻局部放大'
    '04_控制负担对比'
    '05_逐级性能差距图'
    };
figures = {
    makeOverviewFigure(runs, summaryMetrics)
    makePerAxisFigure(perAxisMetrics)
    makeCriticalMomentFigure(runs)
    makeControlEffortFigure(summaryMetrics)
    makeStageGapFigure(summaryMetrics)
    };
for index = 1:numel(figures)
    saveMeetingFigure(figures{index}, outputDir, figureNames{index});
end

writeReadme(outputDir, run02File, pwmFile, summaryMetrics, perAxisMetrics);
result = struct('outputDir', string(outputDir), 'run02File', string(run02File), ...
    'pwmFile', string(pwmFile), 'runs', runs, 'summaryMetrics', summaryMetrics, ...
    'perAxisMetrics', perAxisMetrics, 'figureNames', string(figureNames));
save(fullfile(outputDir, '四方案对比结果.mat'), 'result');
fprintf('四种控制方案中文组会对比图已导出：%s\n', outputDir);
end

function fileName = latestFile(requestedFile, resultRoot, pattern)
if strlength(string(requestedFile)) > 0
    fileName = char(requestedFile);
else
    candidates = dir(fullfile(resultRoot, pattern));
    assert(~isempty(candidates), '未找到结果文件：%s', pattern);
    [~, index] = max([candidates.datenum]);
    fileName = fullfile(candidates(index).folder, candidates(index).name);
end
assert(isfile(fileName), '结果文件不存在：%s', fileName);
end

function runs = buildAlignedRuns(run02, pwm)
time = pwm.comparison.oracle.t(:).';
q0 = run02.setup.refs.q0(:);
pureReference = interp1(run02.report.poseTime, ...
    run02.report.referenceRelativePose + q0.', time, 'linear').';
pureActual = interp1(run02.report.poseTime, ...
    run02.report.actualRelativePose + q0.', time, 'linear').';
pureForce = interp1(run02.report.forceTime, run02.report.controlForce, time, 'linear').';

keys = ["pureForce", "oracle", "identifiedTruthFeedback", "identified"];
labels = ["纯力控制", "高保真控制", "真实位姿反馈辨识控制", "UKF反馈辨识控制"];
colors = [0.00, 0.38, 0.70; 0.12, 0.58, 0.32; 0.93, 0.48, 0.12; 0.72, 0.16, 0.48];
runs = makeRun(keys(1), labels(1), colors(1, :), time, ...
    pureReference, pureActual, pureForce, nan(6, numel(time)), false);
runs = repmat(runs, 4, 1);
for index = 2:4
    source = pwm.comparison.(keys(index));
    runs(index) = makeRun(keys(index), labels(index), colors(index, :), time, ...
        source.qReference, source.qTrue, source.trueForce, source.pwm, true);
end
end

function run = makeRun(key, label, color, time, referencePose, actualPose, force, pwm, pwmApplicable)
run = struct('key', key, 'label', label, 'color', color, 'time', time, ...
    'referencePose', referencePose, 'actualPose', actualPose, ...
    'poseError', actualPose - referencePose, 'force', force, 'pwm', pwm, ...
    'pwmApplicable', pwmApplicable);
end

function validateRuns(runs, run02, pwm)
assert(isfield(run02, 'setup') && isfield(run02, 'report'), 'Run02 结果结构不完整。');
assert(isfield(pwm, 'comparison') && isfield(pwm.comparison, 'identifiedTruthFeedback'), ...
    'PWM 三线结果结构不完整。');
assert(abs(run02.report.poseTime(1) - runs(1).time(1)) < 1e-12);
assert(abs(run02.report.poseTime(end) - runs(1).time(end)) < 1e-9);
assert(norm(runs(1).referencePose(:, 1) - runs(2).referencePose(:, 1)) < 1e-12);
for index = 1:4
    assert(isequal(size(runs(index).referencePose), size(runs(index).actualPose)));
    assert(size(runs(index).actualPose, 2) == numel(runs(index).time));
    assert(all(isfinite(runs(index).actualPose), 'all'));
    assert(all(isfinite(runs(index).force), 'all'));
end
end

function [summary, perAxis] = calculateMetrics(runs, pwm)
summary = table();
perAxis = table();
axisNames = ["X"; "Y"; "Z"; "横滚"; "俯仰"; "偏航"];
axisUnits = ["mm"; "mm"; "mm"; "deg"; "deg"; "deg"];
axisScale = [1e3; 1e3; 1e3; 180 / pi; 180 / pi; 180 / pi];
pwmLimit = pwm.teacher.pwmMax;
for index = 1:4
    errorValue = runs(index).poseError;
    translationNorm = vecnorm(errorValue(1:3, :), 2, 1);
    rotationNorm = vecnorm(errorValue(4:6, :), 2, 1);
    forceRms = sqrt(mean(runs(index).force.^2, 'all'));
    forcePeak = max(abs(runs(index).force), [], 'all');
    if runs(index).pwmApplicable
        pwmPeak = max(abs(runs(index).pwm), [], 'all');
        pwmSaturation = mean(abs(runs(index).pwm) >= pwmLimit - 1e-9, 'all');
        dt = mean(diff(runs(index).time));
        pwmRateRms = sqrt(mean((diff(runs(index).pwm, 1, 2) / dt).^2, 'all'));
    else
        pwmPeak = NaN;
        pwmSaturation = NaN;
        pwmRateRms = NaN;
    end
    row = table(runs(index).key, runs(index).label, ...
        1e3 * sqrt(mean(translationNorm.^2)), 1e3 * max(translationNorm), ...
        rad2deg(sqrt(mean(rotationNorm.^2))), rad2deg(max(rotationNorm)), ...
        1e3 * translationNorm(end), rad2deg(rotationNorm(end)), forceRms, forcePeak, ...
        runs(index).pwmApplicable, pwmPeak, pwmSaturation, pwmRateRms, ...
        'VariableNames', {'line', 'label', 'translationRmsMm', 'translationPeakMm', ...
        'rotationRmsDeg', 'rotationPeakDeg', 'endpointTranslationMm', ...
        'endpointRotationDeg', 'forceRmsN', 'forcePeakN', 'pwmApplicable', ...
        'maxAbsPwm', 'pwmSaturationRatio', 'pwmRateRmsPerSecond'});
    summary = [summary; row]; %#ok<AGROW>

    axisRows = table(repmat(runs(index).key, 6, 1), repmat(runs(index).label, 6, 1), ...
        axisNames, axisUnits, axisScale .* sqrt(mean(errorValue.^2, 2)), ...
        axisScale .* max(abs(errorValue), [], 2), ...
        'VariableNames', {'line', 'label', 'axis', 'unit', 'rms', 'peak'});
    perAxis = [perAxis; axisRows]; %#ok<AGROW>
end
end

function fig = makeOverviewFigure(runs, metrics)
fig = meetingFigure('四种控制方案核心性能总览');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
translation = arrayfun(@(run) 1e3 * vecnorm(run.poseError(1:3, :), 2, 1), runs, 'UniformOutput', false);
rotation = arrayfun(@(run) rad2deg(vecnorm(run.poseError(4:6, :), 2, 1)), runs, 'UniformOutput', false);

ax = nexttile(layout); plotHistories(ax, runs, translation, '平移误差范数时序', '平移误差 (mm)');
addInset(fig, runs, translation, [0.31, 0.64, 0.15, 0.15], [0, 1.15]);
ax = nexttile(layout); plotHistories(ax, runs, rotation, '旋转误差范数时序', '旋转误差 (deg)');
addInset(fig, runs, rotation, [0.80, 0.64, 0.15, 0.15], [0, 0.85]);
plotRanking(nexttile(layout), metrics.translationRmsMm, metrics.translationPeakMm, ...
    metrics.label, runs, '平移误差指标排名', '误差 (mm)');
plotRanking(nexttile(layout), metrics.rotationRmsDeg, metrics.rotationPeakDeg, ...
    metrics.label, runs, '旋转误差指标排名', '误差 (deg)');
end

function addInset(fig, runs, histories, position, yLimits)
ax = axes(fig, 'Position', position); hold(ax, 'on');
for index = 1:4
    plot(ax, runs(index).time, histories{index}, 'Color', runs(index).color, 'LineWidth', 0.8);
end
grid(ax, 'on'); ylim(ax, yLimits); xlim(ax, [runs(1).time(1), runs(1).time(end)]);
title(ax, '低误差区放大', 'FontSize', 8); set(ax, 'FontSize', 7);
end

function plotRanking(ax, rmsValue, peakValue, labels, runs, titleText, xLabel)
hold(ax, 'on');
for index = 1:4
    semilogx(ax, rmsValue(index), index, 'o', 'Color', runs(index).color, ...
        'MarkerFaceColor', runs(index).color, 'MarkerSize', 7);
    semilogx(ax, peakValue(index), index, 'd', 'Color', runs(index).color, ...
        'MarkerFaceColor', runs(index).color, 'MarkerSize', 7);
    text(ax, rmsValue(index) * 1.08, index + 0.10, sprintf('RMS %.4g', rmsValue(index)), ...
        'FontSize', 8, 'Color', runs(index).color);
    text(ax, peakValue(index) * 1.08, index - 0.13, sprintf('峰值 %.4g', peakValue(index)), ...
        'FontSize', 8, 'Color', runs(index).color);
end
grid(ax, 'on'); yticks(ax, 1:4); yticklabels(ax, labels); ylim(ax, [0.5, 4.5]);
xlabel(ax, xLabel); title(ax, titleText);
end

function fig = makePerAxisFigure(perAxis)
fig = meetingFigure('六自由度分轴误差对比');
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
rmsValues = reshape(perAxis.rms, 6, 4).';
peakValues = reshape(perAxis.peak, 6, 4).';
drawHeatmap(nexttile(layout), rmsValues, '六自由度 RMS', perAxis);
drawHeatmap(nexttile(layout), peakValues, '六自由度峰值', perAxis);
end

function drawHeatmap(ax, values, titleText, perAxis)
imagesc(ax, log10(max(values, 1e-6))); colormap(ax, parula); colorbar(ax);
xticks(ax, 1:6); xticklabels(ax, perAxis.axis(1:6));
yticks(ax, 1:4); yticklabels(ax, perAxis.label(1:6:end));
xlabel(ax, '自由度（X/Y/Z：mm；横滚/俯仰/偏航：deg）'); ylabel(ax, '控制方案');
title(ax, [titleText, '（颜色为对数尺度，数字为真实值）']);
for row = 1:4
    for column = 1:6
        text(ax, column, row, sprintf('%.3g', values(row, column)), ...
            'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 8);
    end
end
end

function fig = makeCriticalMomentFigure(runs)
ukfTranslation = vecnorm(runs(4).poseError(1:3, :), 2, 1);
ukfRotation = vecnorm(runs(4).poseError(4:6, :), 2, 1);
[~, translationIndex] = max(ukfTranslation);
[~, rotationIndex] = max(ukfRotation);
fig = meetingFigure('空间轨迹与 UKF 关键误差时刻');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout, [2, 1]); hold(ax, 'on');
plot3(ax, runs(1).referencePose(1, :), runs(1).referencePose(2, :), ...
    runs(1).referencePose(3, :), 'k--', 'LineWidth', 2);
for index = 1:4
    plot3(ax, runs(index).actualPose(1, :), runs(index).actualPose(2, :), ...
        runs(index).actualPose(3, :), 'Color', runs(index).color, 'LineWidth', 1.2);
end
grid(ax, 'on'); axis(ax, 'tight'); view(ax, 35, 25);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)'); title(ax, '平台空间轨迹');
legend(ax, ["参考轨迹"; string({runs.label}).'], 'Location', 'best');
plotCriticalZoom(nexttile(layout), runs, translationIndex, true);
plotCriticalZoom(nexttile(layout), runs, rotationIndex, false);
end

function plotCriticalZoom(ax, runs, center, translationMode)
window = max(1, center - 60):min(numel(runs(1).time), center + 60);
hold(ax, 'on');
for index = 1:4
    if translationMode
        value = 1e3 * vecnorm(runs(index).poseError(1:3, window), 2, 1);
        yLabel = '平移误差 (mm)';
        titleText = sprintf('UKF 最大平移误差附近（%.3f s）', runs(1).time(center));
    else
        value = rad2deg(vecnorm(runs(index).poseError(4:6, window), 2, 1));
        yLabel = '旋转误差 (deg)';
        titleText = sprintf('UKF 最大旋转误差附近（%.3f s）', runs(1).time(center));
    end
    plot(ax, runs(index).time(window), value, 'Color', runs(index).color, 'LineWidth', 1.3);
end
xline(ax, runs(1).time(center), 'k--', '关键时刻');
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel); title(ax, titleText);
end

function fig = makeControlEffortFigure(metrics)
fig = meetingFigure('四种控制方案的控制负担对比');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
drawColoredBars(nexttile(layout), metrics.forceRmsN, metrics.label, '输出力 RMS', '力 (N)');
drawColoredBars(nexttile(layout), metrics.forcePeakN, metrics.label, '输出力峰值', '力 (N)');
drawPwmBars(nexttile(layout), metrics.maxAbsPwm, metrics.label, 'PWM 峰值', 'PWM');
drawPwmBurden(nexttile(layout), metrics, 'PWM 指令变化率与饱和比例');
end

function drawColoredBars(ax, values, labels, titleText, yLabel)
bars = bar(ax, values, 'FaceColor', 'flat');
bars.CData = palette();
grid(ax, 'on'); xticks(ax, 1:4); xticklabels(ax, labels); xtickangle(ax, 12);
ylabel(ax, yLabel); title(ax, titleText);
for index = 1:4
    text(ax, index, values(index), sprintf(' %.1f', values(index)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
end

function drawPwmBars(ax, values, labels, titleText, yLabel)
bars = bar(ax, values, 'FaceColor', 'flat'); bars.CData = palette();
grid(ax, 'on'); xticks(ax, 1:4); xticklabels(ax, labels); xtickangle(ax, 12);
ylabel(ax, yLabel); title(ax, titleText);
text(ax, 1, max(values(2:4)) * 0.08, '不适用', 'HorizontalAlignment', 'center');
for index = 2:4
    text(ax, index, values(index), sprintf(' %.0f', values(index)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
end

function drawPwmBurden(ax, metrics, titleText)
yyaxis(ax, 'left');
bar(ax, 2:4, metrics.pwmRateRmsPerSecond(2:4), 'FaceColor', [0.38, 0.58, 0.82]);
ylabel(ax, 'PWM 变化率 RMS (每秒)');
yyaxis(ax, 'right');
plot(ax, 2:4, 100 * metrics.pwmSaturationRatio(2:4), 'o-', 'LineWidth', 1.5, ...
    'MarkerFaceColor', [0.72, 0.16, 0.48]);
ylabel(ax, 'PWM 饱和时间比例 (%)');
ylim(ax, [0, 1]);
xticks(ax, 1:4); xticklabels(ax, metrics.label); xtickangle(ax, 12); grid(ax, 'on');
title(ax, titleText); text(ax, 1, 0, '不适用', 'HorizontalAlignment', 'center');
end

function fig = makeStageGapFigure(metrics)
fig = meetingFigure('四种控制方案逐级性能差距（首级为系统差距，后两级为辨识与 UKF 反馈变化）');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
drawStage(nexttile(layout), metrics.translationRmsMm, metrics.label, '平移 RMS 逐级变化', 'mm');
drawStage(nexttile(layout), metrics.translationPeakMm, metrics.label, '平移峰值逐级变化', 'mm');
drawStage(nexttile(layout), metrics.rotationRmsDeg, metrics.label, '旋转 RMS 逐级变化', 'deg');
drawStage(nexttile(layout), metrics.rotationPeakDeg, metrics.label, '旋转峰值逐级变化', 'deg');
end

function drawStage(ax, values, labels, titleText, unit)
semilogy(ax, 1:4, values, 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', [0.15, 0.45, 0.75]);
grid(ax, 'on'); xticks(ax, 1:4); xticklabels(ax, labels); xtickangle(ax, 12);
ylabel(ax, ['误差 (', unit, ')']); title(ax, titleText);
for index = 1:4
    text(ax, index, values(index) * 1.12, sprintf('%.4g', values(index)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end
end

function plotHistories(ax, runs, histories, titleText, yLabel)
hold(ax, 'on');
for index = 1:4
    plot(ax, runs(index).time, histories{index}, 'Color', runs(index).color, 'LineWidth', 1.2);
end
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel); title(ax, titleText);
legend(ax, string({runs.label}), 'Location', 'best');
end

function fig = meetingFigure(titleText)
fig = figure('Color', 'w', 'Visible', 'off', 'Units', 'pixels', ...
    'Position', [80, 80, 1600, 900]);
sgtitle(fig, titleText, 'FontSize', 17, 'FontWeight', 'bold');
end

function colors = palette()
colors = [0.00, 0.38, 0.70; 0.12, 0.58, 0.32; 0.93, 0.48, 0.12; 0.72, 0.16, 0.48];
end

function saveMeetingFigure(fig, outputDir, baseName)
figFile = fullfile(outputDir, [baseName, '.fig']);
pngFile = fullfile(outputDir, [baseName, '.png']);
pdfFile = fullfile(outputDir, [baseName, '.pdf']);
savefig(fig, figFile);
set(fig, 'PaperUnits', 'inches', 'PaperPosition', [0, 0, 16, 9], ...
    'PaperSize', [16, 9], 'InvertHardcopy', 'off');
print(fig, pngFile, '-dpng', '-r300');
print(fig, pdfFile, '-dpdf', '-vector', '-bestfit');
close(fig);
end

function writeChineseTable(source, fileName, chineseNames)
output = source;
output.Properties.VariableNames = chineseNames;
writetable(output, fileName, 'Encoding', 'UTF-8');
end

function writeReadme(outputDir, run02File, pwmFile, summary, perAxis)
fid = fopen(fullfile(outputDir, 'README.md'), 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid));
[~, worstAxisIndex] = max(perAxis.rms(perAxis.line == "identified"));
ukfRows = perAxis(perAxis.line == "identified", :);
fprintf(fid, '# 四种控制方案中文组会对比\n\n');
fprintf(fid, '- 纯力控制结果：`%s`\n', run02File);
fprintf(fid, '- PWM 三线结果：`%s`\n\n', pwmFile);
fprintf(fid, '> 公平性说明：纯力控制是理想力源系统基线；其余三条使用 PWM 高保真执行器。四方案可以同级展示性能，但纯力到高保真之间不是严格单变量消融。\n\n');
fprintf(fid, '## 核心指标\n\n');
fprintf(fid, '| 方案 | 平移 RMS (mm) | 平移峰值 (mm) | 旋转 RMS (deg) | 旋转峰值 (deg) | 力峰值 (N) | PWM峰值 |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|\n');
for index = 1:height(summary)
    if summary.pwmApplicable(index)
        pwmText = sprintf('%.0f', summary.maxAbsPwm(index));
    else
        pwmText = '不适用';
    end
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f | %.2f | %s |\n', ...
        summary.label(index), summary.translationRmsMm(index), summary.translationPeakMm(index), ...
        summary.rotationRmsDeg(index), summary.rotationPeakDeg(index), ...
        summary.forcePeakN(index), pwmText);
end
fprintf(fid, '\n## 组会讲述建议\n\n');
fprintf(fid, '1. 先用核心总览展示四方案跨数量级的整体差异，强调对数排名图旁标注的真实数值。\n');
fprintf(fid, '2. 用六自由度热力图说明 UKF 辨识控制退化最明显的方向是 `%s`。\n', ukfRows.axis(worstAxisIndex));
fprintf(fid, '3. 用关键时刻局部放大图解释 UKF 峰值误差发生在哪一段轨迹。\n');
fprintf(fid, '4. 用控制负担图说明精度差异是否伴随更大的真实力、PWM峰值或指令变化率。\n');
fprintf(fid, '5. 最后用逐级性能差距图总结：系统级执行器差距、辨识反馈差距和 UKF 反馈差距。\n');
end
