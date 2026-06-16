function result = exportPwmFeedbackSourceComparison(outputDir, teacher, identified, refs, overrides)
% exportPwmFeedbackSourceComparison - 导出 PWM 闭环反馈源消融实验的组会结果
arguments
    outputDir string = ""
    teacher struct = struct()
    identified struct = struct()
    refs struct = struct()
    overrides struct = struct()
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));

[teacher, identified, identifierFile] = resolveIdentifier( ...
    fullfile(optRoot, 'results'), teacher, identified);
[refs, referenceFile] = resolveReference(optRoot, refs);
if strlength(outputDir) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(optRoot, 'results', ...
        ['pwm_feedback_source_comparison_', timestamp]);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

comparison = comparePwmPoseForceControlLines(refs, teacher, identified, overrides);
report = evaluatePwmPoseForceControlComparison(comparison, teacher);
[summaryMetrics, perAxisMetrics, errorAttribution] = buildTables(comparison, report, teacher);
acceptance = table(string(fieldnames(report.acceptance)), ...
    logical(cell2mat(struct2cell(report.acceptance))), ...
    'VariableNames', {'check', 'passed'});

writetable(summaryMetrics, fullfile(outputDir, 'summary_metrics.csv'));
writetable(perAxisMetrics, fullfile(outputDir, 'per_axis_metrics.csv'));
writetable(errorAttribution, fullfile(outputDir, 'error_attribution.csv'));
writetable(acceptance, fullfile(outputDir, 'acceptance.csv'));

plotSpatialTrajectory(comparison, fullfile(outputDir, '01_three_line_spatial_trajectory.png'));
plotPoseErrorHistory(comparison, fullfile(outputDir, '02_pose_error_time_history.png'));
plotPerAxisErrors(comparison, fullfile(outputDir, '03_per_axis_pose_errors.png'));
plotCoreMetrics(summaryMetrics, fullfile(outputDir, '04_core_metrics.png'));
plotForceEstimation(comparison, fullfile(outputDir, '05_force_estimation_comparison.png'));
plotForceAndPwm(comparison, fullfile(outputDir, '06_force_pwm_comparison.png'));
plotAttribution(errorAttribution, fullfile(outputDir, '07_error_attribution.png'));
plotCriticalMoments(comparison, fullfile(outputDir, '08_critical_moments_zoom.png'));
writeReadme(outputDir, summaryMetrics, errorAttribution, report, identifierFile, referenceFile);

result = struct('outputDir', string(outputDir), 'comparison', comparison, ...
    'report', report, 'summaryMetrics', summaryMetrics, ...
    'perAxisMetrics', perAxisMetrics, 'errorAttribution', errorAttribution, ...
    'acceptance', acceptance, 'identifierFile', string(identifierFile), ...
    'referenceFile', string(referenceFile));
save(fullfile(outputDir, 'comparison.mat'), 'result');
fprintf('PWM 反馈源消融实验已导出：%s\n', outputDir);
end

function [teacher, identified, fileName] = resolveIdentifier(resultDir, teacher, identified)
fileName = "";
if ~isempty(fieldnames(teacher)) && ~isempty(fieldnames(identified))
    return;
end
candidates = dir(fullfile(resultDir, 'pwm_force_identifier_*.mat'));
if isempty(candidates)
    error('exportPwmFeedbackSourceComparison:MissingIdentifier', '未找到 PWM 辨识模型。');
end
[~, index] = max([candidates.datenum]);
fileName = string(fullfile(candidates(index).folder, candidates(index).name));
sample = load(fileName, 'teacher', 'identified');
teacher = sample.teacher;
identified = sample.identified;
end

function [refs, fileName] = resolveReference(optRoot, refs)
fileName = "";
if ~isempty(fieldnames(refs))
    return;
end
fileName = string(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'));
sample = load(fileName, 'refs');
refs = sample.refs;
end

function [summary, perAxis, attribution] = buildTables(comparison, report, teacher)
lineNames = ["oracle"; "identifiedTruthFeedback"; "identified"];
summary = table();
perAxis = table();
for index = 1:numel(lineNames)
    name = lineNames(index);
    line = comparison.(name);
    poseError = line.qTrue - line.qReference;
    alignedForceError = line.estimatedForce(:, 2:end) - line.trueForce(:, 1:end - 1);
    forceTrackingError = line.trueForce - line.targetForce;
    translationNorm = vecnorm(poseError(1:3, :), 2, 1);
    rotationNorm = vecnorm(poseError(4:6, :), 2, 1);
    row = table(name, 1e3 * sqrt(mean(translationNorm.^2)), 1e3 * max(translationNorm), ...
        rad2deg(sqrt(mean(rotationNorm.^2))), rad2deg(max(rotationNorm)), ...
        1e3 * translationNorm(end), rad2deg(rotationNorm(end)), ...
        sqrt(mean(forceTrackingError.^2, 'all')) / (2 * max(teacher.forceLimit)), ...
        sqrt(mean(alignedForceError.^2, 'all')), max(abs(line.pwm), [], 'all'), ...
        mean(abs(line.pwm) >= teacher.pwmMax - 1e-9, 'all'), ...
        'VariableNames', {'line', 'translationRmsMm', 'translationPeakMm', ...
        'rotationRmsDeg', 'rotationPeakDeg', 'endpointTranslationMm', ...
        'endpointRotationDeg', 'forceTrackingNrmse', 'alignedForceEstimateRmsN', ...
        'maxAbsPwm', 'pwmSaturationRatio'});
    summary = [summary; row]; %#ok<AGROW>

    axisNames = ["x"; "y"; "z"; "roll"; "pitch"; "yaw"];
    scale = [1e3; 1e3; 1e3; 180 / pi; 180 / pi; 180 / pi];
    unit = ["mm"; "mm"; "mm"; "deg"; "deg"; "deg"];
    axisRows = table(repmat(name, 6, 1), axisNames, unit, ...
        scale .* sqrt(mean(poseError.^2, 2)), scale .* max(abs(poseError), [], 2), ...
        'VariableNames', {'line', 'axis', 'unit', 'rms', 'peak'});
    perAxis = [perAxis; axisRows]; %#ok<AGROW>
end

oracle = summary(summary.line == "oracle", :);
truth = summary(summary.line == "identifiedTruthFeedback", :);
ukf = summary(summary.line == "identified", :);
metric = ["translationRmsMm"; "translationPeakMm"; "rotationRmsDeg"; ...
    "rotationPeakDeg"; "forceTrackingNrmse"; "alignedForceEstimateRmsN"];
oracleValue = [oracle.translationRmsMm; oracle.translationPeakMm; oracle.rotationRmsDeg; ...
    oracle.rotationPeakDeg; oracle.forceTrackingNrmse; oracle.alignedForceEstimateRmsN];
truthValue = [truth.translationRmsMm; truth.translationPeakMm; truth.rotationRmsDeg; ...
    truth.rotationPeakDeg; truth.forceTrackingNrmse; truth.alignedForceEstimateRmsN];
ukfValue = [ukf.translationRmsMm; ukf.translationPeakMm; ukf.rotationRmsDeg; ...
    ukf.rotationPeakDeg; ukf.forceTrackingNrmse; ukf.alignedForceEstimateRmsN];
attribution = table(metric, oracleValue, truthValue, ukfValue, ...
    truthValue - oracleValue, ukfValue - truthValue, ukfValue - oracleValue, ...
    safeRatio(ukfValue, truthValue), ...
    'VariableNames', {'metric', 'oracle', 'identifiedTruthFeedback', 'identifiedUkf', ...
    'identificationAndForceLoopPenalty', 'ukfFeedbackPenalty', ...
    'totalDeployablePenalty', 'ukfToTruthRatio'});

% 保留评价器中的验收数据，便于结果文件与主入口一致。
assert(isfield(report.metrics, 'ukfTranslationRmsPenalty'));
end

function ratio = safeRatio(numerator, denominator)
ratio = numerator ./ denominator;
ratio(abs(denominator) < eps) = NaN;
end

function plotSpatialTrajectory(comparison, fileName)
fig = makeFigure([80, 80, 1250, 850]);
plot3(comparison.oracle.qReference(1, :), comparison.oracle.qReference(2, :), ...
    comparison.oracle.qReference(3, :), 'k--', 'LineWidth', 2); hold on;
plot3(comparison.oracle.qTrue(1, :), comparison.oracle.qTrue(2, :), ...
    comparison.oracle.qTrue(3, :), 'LineWidth', 1.2);
plot3(comparison.identifiedTruthFeedback.qTrue(1, :), ...
    comparison.identifiedTruthFeedback.qTrue(2, :), ...
    comparison.identifiedTruthFeedback.qTrue(3, :), 'LineWidth', 1.2);
plot3(comparison.identified.qTrue(1, :), comparison.identified.qTrue(2, :), ...
    comparison.identified.qTrue(3, :), 'LineWidth', 1.2);
grid on; axis equal; view(35, 25); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('PWM 闭环三线空间轨迹对比');
legend('参考', 'Oracle', '辨识模型+真值位姿反馈', '辨识模型+UKF反馈', 'Location', 'best');
exportAndClose(fig, fileName);
end

function plotPoseErrorHistory(comparison, fileName)
fig = makeFigure([80, 80, 1450, 900]);
layout = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
lines = getLines(comparison);
for index = 1:3
    errorValue = lines{index}.qTrue - lines{index}.qReference;
    nexttile(layout, 1); hold on;
    plot(lines{index}.t, 1e3 * vecnorm(errorValue(1:3, :), 2, 1), 'LineWidth', 1.1);
    nexttile(layout, 2); hold on;
    plot(lines{index}.t, rad2deg(vecnorm(errorValue(4:6, :), 2, 1)), 'LineWidth', 1.1);
end
formatTile(nexttile(layout, 1), '平移误差范数', '误差 (mm)');
formatTile(nexttile(layout, 2), '旋转误差范数', '误差 (deg)');
legend(nexttile(layout, 1), lineLabels(), 'Location', 'best');
exportAndClose(fig, fileName);
end

function plotPerAxisErrors(comparison, fileName)
fig = makeFigure([60, 60, 1650, 1000]);
layout = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
lines = getLines(comparison);
names = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
for axisIndex = 1:6
    ax = nexttile(layout); hold(ax, 'on');
    for lineIndex = 1:3
        errorValue = lines{lineIndex}.qTrue(axisIndex, :) - lines{lineIndex}.qReference(axisIndex, :);
        if axisIndex <= 3
            errorValue = 1e3 * errorValue;
            unit = 'mm';
        else
            errorValue = rad2deg(errorValue);
            unit = 'deg';
        end
        plot(ax, lines{lineIndex}.t, errorValue, 'LineWidth', 0.9);
    end
    grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, ['误差 (' unit ')']); title(ax, names{axisIndex});
end
legend(nexttile(layout, 1), lineLabels(), 'Location', 'best');
exportAndClose(fig, fileName);
end

function plotCoreMetrics(summary, fileName)
fig = makeFigure([80, 80, 1450, 800]);
layout = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
bar(ax, [summary.translationRmsMm, summary.translationPeakMm, ...
    summary.endpointTranslationMm]); grid(ax, 'on'); ylabel(ax, 'mm');
xticklabels(ax, lineLabels()); title(ax, '平移跟踪指标');
legend(ax, 'RMS', '峰值', '终点', 'Location', 'best');
ax = nexttile(layout);
bar(ax, [summary.rotationRmsDeg, summary.rotationPeakDeg, ...
    summary.endpointRotationDeg]); grid(ax, 'on'); ylabel(ax, 'deg');
xticklabels(ax, lineLabels()); title(ax, '旋转跟踪指标');
legend(ax, 'RMS', '峰值', '终点', 'Location', 'best');
exportAndClose(fig, fileName);
end

function plotForceEstimation(comparison, fileName)
fig = makeFigure([80, 80, 1450, 900]);
layout = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
truthLine = comparison.identifiedTruthFeedback;
ukfLine = comparison.identified;
truthError = truthLine.estimatedForce(:, 2:end) - truthLine.trueForce(:, 1:end - 1);
ukfError = ukfLine.estimatedForce(:, 2:end) - ukfLine.trueForce(:, 1:end - 1);
ax = nexttile(layout); plot(ax, truthLine.t(2:end), truthError.'); grid(ax, 'on');
title(ax, '辨识模型+真值位姿反馈：对齐后的力估计误差'); ylabel(ax, '误差 (N)');
ax = nexttile(layout); plot(ax, ukfLine.t(2:end), ukfError.'); grid(ax, 'on');
title(ax, '辨识模型+UKF反馈：对齐后的力估计误差'); ylabel(ax, '误差 (N)'); xlabel(ax, '时间 (s)');
exportAndClose(fig, fileName);
end

function plotForceAndPwm(comparison, fileName)
fig = makeFigure([60, 60, 1550, 1000]);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
truthLine = comparison.identifiedTruthFeedback;
ukfLine = comparison.identified;
plotSignal(nexttile(layout), truthLine.t, truthLine.targetForce, truthLine.trueForce, ...
    '真值位姿反馈：目标力与真实力', '力 (N)');
plotSignal(nexttile(layout), ukfLine.t, ukfLine.targetForce, ukfLine.trueForce, ...
    'UKF反馈：目标力与真实力', '力 (N)');
plotSingle(nexttile(layout), truthLine.t, truthLine.pwm, '真值位姿反馈：PWM命令', 'PWM');
plotSingle(nexttile(layout), ukfLine.t, ukfLine.pwm, 'UKF反馈：PWM命令', 'PWM');
exportAndClose(fig, fileName);
end

function plotAttribution(attribution, fileName)
fig = makeFigure([50, 80, 1750, 800]);
layout = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
bar(ax, [attribution.identificationAndForceLoopPenalty(1:4), attribution.ukfFeedbackPenalty(1:4)], ...
    'grouped'); grid(ax, 'on'); xticklabels(ax, attribution.metric(1:4));
ylabel(ax, '绝对误差增量'); title(ax, '位姿跟踪误差来源分解');
legend(ax, '辨识模型/力环代价', 'UKF反馈代价', 'Location', 'best');
ax = nexttile(layout);
bar(ax, 100 * [attribution.identificationAndForceLoopPenalty(5), ...
    attribution.ukfFeedbackPenalty(5)], 'grouped'); grid(ax, 'on');
xticklabels(ax, {'辨识模型/力环代价', 'UKF反馈代价'});
ylabel(ax, 'NRMSE 增量 (%)'); title(ax, '力跟踪误差来源分解');
ax = nexttile(layout);
bar(ax, [attribution.identificationAndForceLoopPenalty(6), ...
    attribution.ukfFeedbackPenalty(6)], 'grouped'); grid(ax, 'on');
xticklabels(ax, {'辨识模型/力环代价', 'UKF反馈代价'});
ylabel(ax, 'RMS 增量 (N)'); title(ax, '力估计误差来源分解');
exportAndClose(fig, fileName);
end

function plotCriticalMoments(comparison, fileName)
line = comparison.identified;
poseError = line.qTrue - line.qReference;
forceError = line.estimatedForce(:, 2:end) - line.trueForce(:, 1:end - 1);
[~, translationIndex] = max(vecnorm(poseError(1:3, :), 2, 1));
[~, rotationIndex] = max(vecnorm(poseError(4:6, :), 2, 1));
[~, forceIndex] = max(vecnorm(forceError, 2, 1));
indices = [translationIndex, rotationIndex, forceIndex + 1];
titles = {'最大平移误差附近', '最大旋转误差附近', '最大力估计误差附近'};
fig = makeFigure([60, 60, 1550, 1000]);
layout = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
lines = getLines(comparison);
for panelIndex = 1:3
    ax = nexttile(layout); hold(ax, 'on');
    center = indices(panelIndex);
    window = max(1, center - 30):min(numel(line.t), center + 30);
    for lineIndex = 1:3
        errorValue = lines{lineIndex}.qTrue - lines{lineIndex}.qReference;
        plot(ax, lines{lineIndex}.t(window), ...
            1e3 * vecnorm(errorValue(1:3, window), 2, 1), 'LineWidth', 1.0);
    end
    grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '平移误差 (mm)'); title(ax, titles{panelIndex});
end
legend(nexttile(layout, 1), lineLabels(), 'Location', 'best');
exportAndClose(fig, fileName);
end

function writeReadme(outputDir, summary, attribution, report, identifierFile, referenceFile)
fid = fopen(fullfile(outputDir, 'README.md'), 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid));
truth = summary(summary.line == "identifiedTruthFeedback", :);
ukf = summary(summary.line == "identified", :);
fprintf(fid, '# PWM 辨识闭环反馈源消融实验\n\n');
fprintf(fid, '- 辨识模型：`%s`\n', identifierFile);
fprintf(fid, '- 参考轨迹：`%s`\n', referenceFile);
fprintf(fid, '- 总体验收通过：`%d`\n\n', report.passed);
fprintf(fid, '## 核心结果\n\n');
fprintf(fid, '| 控制线 | 平移 RMS (mm) | 平移峰值 (mm) | 旋转 RMS (deg) | 旋转峰值 (deg) | 力跟踪 NRMSE | 对齐力估计 RMS (N) |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|\n');
for index = 1:height(summary)
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f | %.6f | %.3f |\n', ...
        summary.line(index), summary.translationRmsMm(index), summary.translationPeakMm(index), ...
        summary.rotationRmsDeg(index), summary.rotationPeakDeg(index), ...
        summary.forceTrackingNrmse(index), summary.alignedForceEstimateRmsN(index));
end
fprintf(fid, '\n## 误差归因\n\n');
fprintf(fid, '- UKF 相对真值位姿反馈的平移 RMS 增量：`%.6f mm`\n', ...
    attribution.ukfFeedbackPenalty(attribution.metric == "translationRmsMm"));
fprintf(fid, '- 辨识模型/力环相对 oracle 的平移 RMS 增量：`%.6f mm`\n', ...
    attribution.identificationAndForceLoopPenalty(attribution.metric == "translationRmsMm"));
fprintf(fid, '- UKF 相对真值位姿反馈的旋转 RMS 增量：`%.6f deg`\n', ...
    attribution.ukfFeedbackPenalty(attribution.metric == "rotationRmsDeg"));
fprintf(fid, '- 真值位姿反馈与 UKF 反馈的对齐力估计 RMS：`%.3f N / %.3f N`\n\n', ...
    truth.alignedForceEstimateRmsN, ukf.alignedForceEstimateRmsN);
fprintf(fid, '## 组会讲述建议\n\n');
fprintf(fid, '1. 原双线实验把力辨识误差与 UKF 反馈误差混在一起，本实验新增真值位姿反馈线进行单变量消融。\n');
fprintf(fid, '2. 真值位姿反馈线仍使用灰箱+NARX辨识力，因此不是完整 oracle。\n');
fprintf(fid, '3. 先展示三线总体指标，再使用误差来源分解图判断后续优化优先级。\n');
fprintf(fid, '4. 若真值位姿反馈显著改善力估计，说明 UKF 速度误差通过腿速输入污染了 NARX。\n');
end

function lines = getLines(comparison)
lines = {comparison.oracle, comparison.identifiedTruthFeedback, comparison.identified};
end

function labels = lineLabels()
labels = {'Oracle', '辨识模型+真值位姿反馈', '辨识模型+UKF反馈'};
end

function fig = makeFigure(position)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', position);
end

function formatTile(ax, plotTitle, yLabel)
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel); title(ax, plotTitle);
end

function plotSignal(ax, time, target, actual, plotTitle, yLabel)
plot(ax, time, target.', '--', time, actual.', 'LineWidth', 0.7);
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel); title(ax, plotTitle);
end

function plotSingle(ax, time, value, plotTitle, yLabel)
plot(ax, time, value.', 'LineWidth', 0.7);
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel); title(ax, plotTitle);
end

function exportAndClose(fig, fileName)
exportgraphics(fig, fileName, 'Resolution', 200);
close(fig);
end
