function outputDir = exportCurrentPoseImuUkfEvaluation(outputDir)
% exportCurrentPoseImuUkfEvaluation - 导出当前实物参数下的 UKF 完整评估结果
arguments
    outputDir string = ""
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));

if strlength(outputDir) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(optRoot, 'results', ['ukf_current_evaluation_', timestamp]);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

teacher = makeHighFidelityPwmActuator();
identifierFile = latestFile(fullfile(optRoot, 'results'), 'pwm_force_identifier_*.mat');
identifierSample = load(identifierFile, 'identified');
identified = identifierSample.identified;
referenceFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
referenceSample = load(referenceFile, 'refs');

comparison = comparePwmPoseForceControlLines(referenceSample.refs, teacher, identified);
controlReport = evaluatePwmPoseForceControlComparison(comparison, teacher);
benchmark = benchmarkPwmPoseEstimators(comparison.identified, struct('encoderNoiseStd', 5e-4));

metrics = buildEvaluationMetrics(benchmark, comparison, controlReport);
perAxisMetrics = buildPerAxisMetrics(benchmark, comparison);
acceptance = struct2table(controlReport.acceptance, 'AsArray', true);
writetable(metrics, fullfile(outputDir, 'summary_metrics.csv'));
writetable(perAxisMetrics, fullfile(outputDir, 'per_axis_metrics.csv'));
writetable(acceptance, fullfile(outputDir, 'control_acceptance.csv'));
save(fullfile(outputDir, 'evaluation.mat'), 'benchmark', 'comparison', ...
    'controlReport', 'metrics', 'perAxisMetrics', 'identifierFile', 'referenceFile');

plotPoseErrors(benchmark, fullfile(outputDir, '01_measurement_and_ukf_pose_errors.png'));
plotPoseComparison(benchmark, fullfile(outputDir, '02_truth_measurement_ukf_comparison.png'));
plotVelocityErrors(benchmark, fullfile(outputDir, '03_measurement_and_ukf_velocity_errors.png'));
plotControlEvaluation(comparison, controlReport, ...
    fullfile(outputDir, '04_closed_loop_control_evaluation.png'));
plotOverallMetrics(metrics, controlReport, fullfile(outputDir, '05_overall_metrics.png'));
writeReadme(outputDir, metrics, perAxisMetrics, controlReport, identifierFile, referenceFile);

fprintf('当前 UKF 完整评估已导出：%s\n', outputDir);
end

function fileName = latestFile(folder, pattern)
candidates = dir(fullfile(folder, pattern));
if isempty(candidates)
    error('exportCurrentPoseImuUkfEvaluation:MissingIdentifier', ...
        '未找到 PWM 辨识器结果文件。');
end
[~, index] = max([candidates.datenum]);
fileName = fullfile(candidates(index).folder, candidates(index).name);
end

function metrics = buildEvaluationMetrics(benchmark, comparison, controlReport)
truthPose = benchmark.truth.pose;
truthVelocity = benchmark.truth.velocity;
measurementPose = benchmark.kinematic.pose;
measurementVelocity = benchmark.kinematic.velocity;
predictedPose = benchmark.ukf.predictedPose;
predictedVelocity = benchmark.ukf.predictedVelocity;
ukfPose = benchmark.ukf.pose;
ukfVelocity = benchmark.ukf.velocity;
settled = benchmark.t >= 0.2;

measurementPoseError = measurementPose - truthPose;
ukfPoseError = ukfPose - truthPose;
measurementVelocityError = measurementVelocity - truthVelocity;
predictedPoseError = predictedPose - truthPose;
predictedVelocityError = predictedVelocity - truthVelocity;
ukfVelocityError = ukfVelocity - truthVelocity;
trackingError = comparison.identified.qTrue - comparison.identified.qReference;

metrics = table( ...
    1e3 * rmsAll(measurementPoseError(1:3, settled)), ...
    1e3 * peakAll(measurementPoseError(1:3, settled)), ...
    rad2deg(rmsAll(measurementPoseError(4:6, settled))), ...
    rad2deg(peakAll(measurementPoseError(4:6, settled))), ...
    rmsAll(measurementVelocityError(1:3, settled)), ...
    1e3 * rmsAll(predictedPoseError(1:3, settled)), ...
    1e3 * peakAll(predictedPoseError(1:3, settled)), ...
    rad2deg(rmsAll(predictedPoseError(4:6, settled))), ...
    rad2deg(peakAll(predictedPoseError(4:6, settled))), ...
    rmsAll(predictedVelocityError(1:3, settled)), ...
    1e3 * rmsAll(ukfPoseError(1:3, settled)), ...
    1e3 * peakAll(ukfPoseError(1:3, settled)), ...
    rad2deg(rmsAll(ukfPoseError(4:6, settled))), ...
    rad2deg(peakAll(ukfPoseError(4:6, settled))), ...
    rmsAll(ukfVelocityError(1:3, settled)), ...
    1e3 * rmsAll(trackingError(1:3, :)), ...
    1e3 * peakAll(trackingError(1:3, :)), ...
    rad2deg(rmsAll(trackingError(4:6, :))), ...
    rad2deg(peakAll(trackingError(4:6, :))), ...
    controlReport.metrics.identifiedForceTrackingNrmse, ...
    controlReport.metrics.identifiedAlignedForceEstimateRms, ...
    controlReport.metrics.estimatorVelocityRms, ...
    controlReport.passed, ...
    'VariableNames', { ...
    'measurementTranslationRmsMm', 'measurementTranslationPeakMm', ...
    'measurementRotationRmsDeg', 'measurementRotationPeakDeg', ...
    'measurementTranslationVelocityRmsMps', ...
    'predictionTranslationRmsMm', 'predictionTranslationPeakMm', ...
    'predictionRotationRmsDeg', 'predictionRotationPeakDeg', ...
    'predictionTranslationVelocityRmsMps', ...
    'ukfTranslationRmsMm', 'ukfTranslationPeakMm', ...
    'ukfRotationRmsDeg', 'ukfRotationPeakDeg', ...
    'ukfTranslationVelocityRmsMps', ...
    'controlTrackingTranslationRmsMm', 'controlTrackingTranslationPeakMm', ...
    'controlTrackingRotationRmsDeg', 'controlTrackingRotationPeakDeg', ...
    'forceTrackingNrmse', 'alignedForceEstimateRmsN', ...
    'controlEstimatorEquivalentVelocityRmsMps', 'controlPassed'});
end

function metrics = buildPerAxisMetrics(benchmark, comparison)
axisNames = ["x"; "y"; "z"; "roll"; "pitch"; "yaw"];
truthPose = benchmark.truth.pose;
measurementError = benchmark.kinematic.pose - truthPose;
predictionError = benchmark.ukf.predictedPose - truthPose;
ukfError = benchmark.ukf.pose - truthPose;
trackingError = comparison.identified.qTrue - comparison.identified.qReference;
settled = benchmark.t >= 0.2;

scale = [1e3; 1e3; 1e3; 180 / pi; 180 / pi; 180 / pi];
unit = ["mm"; "mm"; "mm"; "deg"; "deg"; "deg"];
metrics = table(axisNames, unit, ...
    scale .* sqrt(mean(measurementError(:, settled).^2, 2)), ...
    scale .* max(abs(measurementError(:, settled)), [], 2), ...
    scale .* sqrt(mean(predictionError(:, settled).^2, 2)), ...
    scale .* max(abs(predictionError(:, settled)), [], 2), ...
    scale .* sqrt(mean(ukfError(:, settled).^2, 2)), ...
    scale .* max(abs(ukfError(:, settled)), [], 2), ...
    scale .* sqrt(mean(trackingError.^2, 2)), ...
    scale .* max(abs(trackingError), [], 2), ...
    'VariableNames', {'axis', 'unit', 'measurementRms', 'measurementPeak', ...
    'predictionRms', 'predictionPeak', 'ukfRms', 'ukfPeak', ...
    'controlTrackingRms', 'controlTrackingPeak'});
end

function plotPoseErrors(benchmark, fileName)
t = benchmark.t;
measurementError = benchmark.kinematic.pose - benchmark.truth.pose;
predictionError = benchmark.ukf.predictedPose - benchmark.truth.pose;
ukfError = benchmark.ukf.pose - benchmark.truth.pose;
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50, 50, 1900, 980]);
layout = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '测量重建、UKF 先验预测与校正后融合结果相对真实值的误差', ...
    'FontSize', 17, 'FontWeight', 'bold');

plotErrorTile(nexttile(layout), t, 1e3 * measurementError(1:3, :), ...
    '测量重建平移误差', '误差 (mm)', {'x', 'y', 'z'});
plotErrorTile(nexttile(layout), t, 1e3 * predictionError(1:3, :), ...
    'UKF 先验预测平移误差', '误差 (mm)', {'x', 'y', 'z'});
plotErrorTile(nexttile(layout), t, 1e3 * ukfError(1:3, :), ...
    'UKF 校正后平移误差', '误差 (mm)', {'x', 'y', 'z'});
plotErrorTile(nexttile(layout), t, rad2deg(measurementError(4:6, :)), ...
    '姿态测量误差', '误差 (°)', {'横滚', '俯仰', '航向'});
plotErrorTile(nexttile(layout), t, rad2deg(predictionError(4:6, :)), ...
    'UKF 先验预测姿态误差', '误差 (°)', {'横滚', '俯仰', '航向'});
plotErrorTile(nexttile(layout), t, rad2deg(ukfError(4:6, :)), ...
    'UKF 校正后姿态误差', '误差 (°)', {'横滚', '俯仰', '航向'});
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotPoseComparison(benchmark, fileName)
t = benchmark.t;
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1550, 1100]);
layout = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '真实值、测量重建、UKF 先验预测与校正后融合结果', ...
    'FontSize', 17, 'FontWeight', 'bold');
names = {'x 平移', 'y 平移', 'z 平移', '横滚角', '俯仰角', '航向角'};
for axisIndex = 1:6
    ax = nexttile(layout);
    if axisIndex <= 3
        scale = 1e3;
        unit = 'mm';
    else
        scale = 180 / pi;
        unit = 'deg';
    end
    plot(ax, t, scale * benchmark.truth.pose(axisIndex, :), 'k-', 'LineWidth', 1.0);
    hold(ax, 'on');
    plot(ax, t, scale * benchmark.kinematic.pose(axisIndex, :), ...
        'Color', [0.85 0.45 0.15], 'LineWidth', 1.0);
    plot(ax, t, scale * benchmark.ukf.predictedPose(axisIndex, :), ...
        'Color', [0.45 0.65 0.20], 'LineWidth', 1.0);
    plot(ax, t, scale * benchmark.ukf.pose(axisIndex, :), ...
        'Color', [0.10 0.40 0.75], 'LineWidth', 1.0);
    grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, unit); title(ax, names{axisIndex});
    if axisIndex == 1
        legend(ax, '真实值', '测量重建', 'UKF 先验预测', ...
            'UKF 校正后结果', 'Location', 'best');
    end
end
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotVelocityErrors(benchmark, fileName)
t = benchmark.t;
measurementError = benchmark.kinematic.velocity - benchmark.truth.velocity;
predictionError = benchmark.ukf.predictedVelocity - benchmark.truth.velocity;
ukfError = benchmark.ukf.velocity - benchmark.truth.velocity;
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1500, 1100]);
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '测量重建、UKF 先验预测与校正后速度相对真实值的误差', ...
    'FontSize', 17, 'FontWeight', 'bold');
plotErrorTile(nexttile(layout), t, measurementError(1:3, :), ...
    '测量重建差分速度误差', '误差 (m/s)', ...
    {'v_x', 'v_y', 'v_z'});
plotErrorTile(nexttile(layout), t, predictionError(1:3, :), ...
    'UKF 先验预测平移速度误差', '误差 (m/s)', ...
    {'v_x', 'v_y', 'v_z'});
plotErrorTile(nexttile(layout), t, ukfError(1:3, :), ...
    'UKF 校正后平移速度误差', '误差 (m/s)', {'v_x', 'v_y', 'v_z'});
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotControlEvaluation(comparison, report, fileName)
result = comparison.identified;
t = result.t;
trackingError = result.qTrue - result.qReference;
estimationError = result.qFeedback - result.qTrue;
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1550, 1050]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('接入 UKF 反馈后的闭环评估（总体验收通过：%d）', report.passed), ...
    'FontSize', 17, 'FontWeight', 'bold');

plotErrorTile(nexttile(layout), t, 1e3 * estimationError(1:3, :), ...
    '闭环中的 UKF 反馈误差', '误差 (mm)', {'x', 'y', 'z'});
plotErrorTile(nexttile(layout), t, 1e3 * trackingError(1:3, :), ...
    '真实平台跟踪误差', '误差 (mm)', {'x', 'y', 'z'});

ax = nexttile(layout);
plot(ax, t, result.targetForce.', '--', t, result.trueForce.', 'LineWidth', 0.7);
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '力 (N)');
title(ax, sprintf('目标力与真实力跟踪，NRMSE %.4f', report.metrics.identifiedForceTrackingNrmse));

ax = nexttile(layout);
values = [1e3 * report.metrics.estimatorSettledTranslationAxisPeak(:), ...
    1e3 * report.metrics.identifiedTranslationAxisPeak(:)];
bar(ax, values); grid(ax, 'on'); xticklabels(ax, {'x', 'y', 'z'});
ylabel(ax, '峰值误差 (mm)'); legend(ax, 'UKF 反馈误差', '真实跟踪误差', 'Location', 'best');
title(ax, sprintf('旋转跟踪峰值 %.3f°', rad2deg(report.metrics.identifiedRotationPeak)));
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotOverallMetrics(metrics, report, fileName)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1450, 700]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'UKF 与闭环控制总体指标', 'FontSize', 17, 'FontWeight', 'bold');

ax = nexttile(layout);
values = [metrics.measurementTranslationRmsMm, metrics.predictionTranslationRmsMm, ...
    metrics.ukfTranslationRmsMm; metrics.measurementTranslationPeakMm, ...
    metrics.predictionTranslationPeakMm, metrics.ukfTranslationPeakMm];
bar(ax, values); grid(ax, 'on'); xticklabels(ax, {'平移 RMS', '平移峰值'});
ylabel(ax, '误差 (mm)'); legend(ax, '测量重建', ...
    'UKF 先验预测', 'UKF 校正后结果', 'Location', 'best');
title(ax, '离线位置估计');

ax = nexttile(layout);
names = fieldnames(report.acceptance);
values = cell2mat(struct2cell(report.acceptance));
barh(ax, double(values)); grid(ax, 'on'); xlim(ax, [0 1.2]); yticks(ax, 1:numel(names));
yticklabels(ax, acceptanceChineseNames(names)); xlabel(ax, '通过 = 1'); title(ax, '闭环验收项');
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotErrorTile(ax, t, errorValue, plotTitle, yLabel, legendText)
plot(ax, t, errorValue.', 'LineWidth', 1.0);
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel); title(ax, plotTitle);
legend(ax, legendText, 'Location', 'best');
end

function names = acceptanceChineseNames(fieldNames)
mapping = struct( ...
    'finitePassed', '数值有限性', ...
    'forceTrackingPassed', '力跟踪', ...
    'strictTranslationPassed', '严格平移跟踪', ...
    'translationPassed', '平移跟踪', ...
    'estimatorTranslationPassed', '估计器平移误差', ...
    'estimatorVelocityPassed', '估计器速度误差', ...
    'rotationPassed', '旋转跟踪', ...
    'pwmPassed', 'PWM 限幅', ...
    'forceLimitPassed', '力限幅', ...
    'truthIsolationPassed', '真实值隔离');
names = fieldNames;
for index = 1:numel(fieldNames)
    if isfield(mapping, fieldNames{index})
        names{index} = mapping.(fieldNames{index});
    end
end
end

function value = rmsAll(errorValue)
value = sqrt(mean(errorValue.^2, 'all'));
end

function value = peakAll(errorValue)
value = max(abs(errorValue), [], 'all');
end

function writeReadme(outputDir, metrics, perAxisMetrics, report, identifierFile, referenceFile)
fileId = fopen(fullfile(outputDir, 'README.md'), 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '# 当前实物参数 UKF 完整评估\n\n');
fprintf(fileId, '- 辨识器：`%s`\n', identifierFile);
fprintf(fileId, '- 参考轨迹：`%s`\n', referenceFile);
fprintf(fileId, '- 原始 XYZ 位置不可直接测量；测量位姿指相对腿长与姿态的运动学重建结果。\n');
fprintf(fileId, '- 离线误差统计忽略前 0.2 s 启动过程。\n\n');
fprintf(fileId, '## 总体数据\n\n');
fprintf(fileId, '| 指标 | 数值 |\n|---|---:|\n');
fprintf(fileId, '| 测量重建平移 RMS | %.4f mm |\n', metrics.measurementTranslationRmsMm);
fprintf(fileId, '| 测量重建平移峰值 | %.4f mm |\n', metrics.measurementTranslationPeakMm);
fprintf(fileId, '| UKF 先验预测平移 RMS | %.4f mm |\n', metrics.predictionTranslationRmsMm);
fprintf(fileId, '| UKF 先验预测平移峰值 | %.4f mm |\n', metrics.predictionTranslationPeakMm);
fprintf(fileId, '| UKF 平移 RMS | %.4f mm |\n', metrics.ukfTranslationRmsMm);
fprintf(fileId, '| UKF 平移峰值 | %.4f mm |\n', metrics.ukfTranslationPeakMm);
fprintf(fileId, '| 测量重建平移速度 RMS | %.6f m/s |\n', metrics.measurementTranslationVelocityRmsMps);
fprintf(fileId, '| UKF 先验预测平移速度 RMS | %.6f m/s |\n', metrics.predictionTranslationVelocityRmsMps);
fprintf(fileId, '| UKF 平移速度 RMS | %.6f m/s |\n', metrics.ukfTranslationVelocityRmsMps);
fprintf(fileId, '| 闭环真实平移跟踪 RMS | %.4f mm |\n', metrics.controlTrackingTranslationRmsMm);
fprintf(fileId, '| 闭环真实平移跟踪峰值 | %.4f mm |\n', metrics.controlTrackingTranslationPeakMm);
fprintf(fileId, '| 闭环真实旋转跟踪峰值 | %.4f deg |\n', metrics.controlTrackingRotationPeakDeg);
fprintf(fileId, '| 力跟踪 NRMSE | %.5f |\n', metrics.forceTrackingNrmse);
fprintf(fileId, '| 总体验收通过 | %d |\n\n', report.passed);
fprintf(fileId, '## 分轴数据\n\n');
fprintf(fileId, '| 轴 | 单位 | 测量 RMS | 预测 RMS | 校正后 RMS | 闭环跟踪 RMS | 闭环跟踪峰值 |\n');
fprintf(fileId, '|---|---|---:|---:|---:|---:|---:|\n');
for index = 1:height(perAxisMetrics)
    fprintf(fileId, '| %s | %s | %.5f | %.5f | %.5f | %.5f | %.5f |\n', ...
        perAxisMetrics.axis(index), perAxisMetrics.unit(index), ...
        perAxisMetrics.measurementRms(index), perAxisMetrics.predictionRms(index), ...
        perAxisMetrics.ukfRms(index), ...
        perAxisMetrics.controlTrackingRms(index), perAxisMetrics.controlTrackingPeak(index));
end
end
