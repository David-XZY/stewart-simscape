function outputDir = exportUkfPoseGroupMeetingFigures(outputDir)
% exportUkfPoseGroupMeetingFigures - 导出组会汇报使用的 UKF 位姿结果图
arguments
    outputDir string = ""
end

controllerRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));

if strlength(outputDir) == 0
    outputDir = fullfile(projectRoot, 'results', 'reports', 'ukf_group_meeting_20260615');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

evaluationFolders = dir(fullfile(projectRoot, 'results', 'controller', 'ukf_current_evaluation_*'));
evaluationFolders = evaluationFolders([evaluationFolders.isdir]);
if isempty(evaluationFolders)
    error('exportUkfPoseGroupMeetingFigures:MissingEvaluation', '未找到当前 UKF 评估结果。');
end
[~, latestIndex] = max([evaluationFolders.datenum]);
evaluationFile = fullfile(evaluationFolders(latestIndex).folder, ...
    evaluationFolders(latestIndex).name, 'evaluation.mat');
sample = load(evaluationFile, 'benchmark');
benchmark = sample.benchmark;

metrics = poseMetrics(benchmark);
perAxis = perAxisPoseMetrics(benchmark);
ablation = runPoseAblation(benchmark);

writetable(metrics, fullfile(outputDir, '位姿总体误差.csv'));
writetable(perAxis, fullfile(outputDir, '位姿逐轴误差.csv'));
writetable(ablation, fullfile(outputDir, '位姿误差来源消融.csv'));

plotCurrentOverview(benchmark, metrics, fullfile(outputDir, '08_当前UKF位姿误差总览.png'));
plotPerAxis(perAxis, fullfile(outputDir, '09_当前UKF六轴位姿误差.png'));
plotErrorSources(ablation, fullfile(outputDir, '10_UKF位姿误差来源消融.png'));
plotErrorMechanism(metrics, fullfile(outputDir, '11_平台位姿误差来源与传播路径.png'));

fprintf('UKF 位姿组会图已导出：%s\n', outputDir);
end

function metrics = poseMetrics(benchmark)
settled = benchmark.t >= 0.2;
truth = benchmark.truth.pose;
measurement = benchmark.kinematic.pose - truth;
prediction = benchmark.ukf.predictedPose - truth;
correction = benchmark.ukf.pose - truth;
names = ["测量重建"; "UKF 预测"; "UKF 校正"];
errors = {measurement, prediction, correction};
translationRms = zeros(3, 1);
translationPeak = zeros(3, 1);
rotationRms = zeros(3, 1);
rotationPeak = zeros(3, 1);
for index = 1:3
    value = errors{index}(:, settled);
    translationRms(index) = 1e3 * rmsAll(value(1:3, :));
    translationPeak(index) = 1e3 * peakAll(value(1:3, :));
    rotationRms(index) = rad2deg(rmsAll(value(4:6, :)));
    rotationPeak(index) = rad2deg(peakAll(value(4:6, :)));
end
metrics = table(names, translationRms, translationPeak, rotationRms, rotationPeak, ...
    'VariableNames', {'方法', '平移RMS毫米', '平移峰值毫米', '姿态RMS度', '姿态峰值度'});
end

function metrics = perAxisPoseMetrics(benchmark)
settled = benchmark.t >= 0.2;
truth = benchmark.truth.pose;
errors = {benchmark.kinematic.pose - truth, ...
    benchmark.ukf.predictedPose - truth, benchmark.ukf.pose - truth};
axisName = ["x"; "y"; "z"; "横滚"; "俯仰"; "航向"];
unit = ["毫米"; "毫米"; "毫米"; "度"; "度"; "度"];
scale = [1e3; 1e3; 1e3; 180 / pi; 180 / pi; 180 / pi];
measurementRms = axisRms(errors{1}(:, settled), scale);
predictionRms = axisRms(errors{2}(:, settled), scale);
correctionRms = axisRms(errors{3}(:, settled), scale);
measurementPeak = axisPeak(errors{1}(:, settled), scale);
predictionPeak = axisPeak(errors{2}(:, settled), scale);
correctionPeak = axisPeak(errors{3}(:, settled), scale);
metrics = table(axisName, unit, measurementRms, predictionRms, correctionRms, ...
    measurementPeak, predictionPeak, correctionPeak, ...
    'VariableNames', {'轴', '单位', '测量重建RMS', 'UKF预测RMS', 'UKF校正RMS', ...
    '测量重建峰值', 'UKF预测峰值', 'UKF校正峰值'});
end

function result = runPoseAblation(benchmark)
trajectory = struct('t', benchmark.t, 'qTrue', benchmark.truth.pose, ...
    'qdTrue', benchmark.truth.velocity);
model = buildOptModelCustom();
ukfOverrides = struct( ...
    'encoderNoiseStd', 1e-3, ...
    'orientationNoiseStd', deg2rad([0.1; 0.1; 0.5]), ...
    'accelerationNoiseStd', 9.80665e-3, ...
    'angularVelocityNoiseStd', deg2rad(0.07));
names = ["全部实际误差"; "无随机噪声基线"; "仅腿长噪声"; ...
    "仅姿态噪声"; "仅加速度噪声"; "仅角速度噪声"];
cases = { ...
    struct('encoderNoiseStd', 5e-4, 'orientationNoiseStd', deg2rad([0.1; 0.1; 0.5]), ...
        'accelerationNoiseStd', 9.80665e-3, 'angularVelocityNoiseStd', deg2rad(0.07)), ...
    struct('encoderNoiseStd', 1e-9, 'orientationNoiseStd', 0, ...
        'accelerationNoiseStd', 0, 'angularVelocityNoiseStd', 0), ...
    struct('encoderNoiseStd', 5e-4, 'orientationNoiseStd', 0, ...
        'accelerationNoiseStd', 0, 'angularVelocityNoiseStd', 0), ...
    struct('encoderNoiseStd', 1e-9, 'orientationNoiseStd', deg2rad([0.1; 0.1; 0.5]), ...
        'accelerationNoiseStd', 0, 'angularVelocityNoiseStd', 0), ...
    struct('encoderNoiseStd', 1e-9, 'orientationNoiseStd', 0, ...
        'accelerationNoiseStd', 9.80665e-3, 'angularVelocityNoiseStd', 0), ...
    struct('encoderNoiseStd', 1e-9, 'orientationNoiseStd', 0, ...
        'accelerationNoiseStd', 0, 'angularVelocityNoiseStd', deg2rad(0.07))};

translationRms = zeros(numel(cases), 1);
translationPeak = zeros(numel(cases), 1);
rotationRms = zeros(numel(cases), 1);
rotationPeak = zeros(numel(cases), 1);
settled = benchmark.t >= 0.2;
for index = 1:numel(cases)
    measurements = simulatePoseImuSensors(trajectory, model, cases{index});
    candidate = estimatePoseImuUkfSeries(measurements, model, "specificForce", ukfOverrides);
    errorValue = candidate.pose(:, settled) - trajectory.qTrue(:, settled);
    translationRms(index) = 1e3 * rmsAll(errorValue(1:3, :));
    translationPeak(index) = 1e3 * peakAll(errorValue(1:3, :));
    rotationRms(index) = rad2deg(rmsAll(errorValue(4:6, :)));
    rotationPeak(index) = rad2deg(peakAll(errorValue(4:6, :)));
end
result = table(names, translationRms, translationPeak, rotationRms, rotationPeak, ...
    'VariableNames', {'误差工况', '平移RMS毫米', '平移峰值毫米', '姿态RMS度', '姿态峰值度'});
end

function plotCurrentOverview(benchmark, metrics, fileName)
colors = reportColors();
t = benchmark.t;
truth = benchmark.truth.pose;
measurement = benchmark.kinematic.pose - truth;
prediction = benchmark.ukf.predictedPose - truth;
correction = benchmark.ukf.pose - truth;
translationNorm = 1e3 * [vecnorm(measurement(1:3, :)); ...
    vecnorm(prediction(1:3, :)); vecnorm(correction(1:3, :))];
rotationNorm = rad2deg([vecnorm(measurement(4:6, :)); ...
    vecnorm(prediction(4:6, :)); vecnorm(correction(4:6, :))]);

fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50 50 1650 1080]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '当前 UKF 平台位姿误差总览', 'FontSize', 19, 'FontWeight', 'bold');

ax = nexttile(layout);
plot(ax, t, translationNorm.', 'LineWidth', 0.9);
styleAxis(ax, '平移误差随时间变化', '时间（秒）', '三轴合成误差（毫米）');
legend(ax, {'测量重建', 'UKF 预测', 'UKF 校正'}, 'Location', 'northwest');

ax = nexttile(layout);
plot(ax, t, rotationNorm.', 'LineWidth', 0.9);
styleAxis(ax, '姿态误差随时间变化', '时间（秒）', '三轴合成误差（度）');
legend(ax, {'姿态测量', 'UKF 预测', 'UKF 校正'}, 'Location', 'northwest');

ax = nexttile(layout);
bar(ax, [metrics.("平移RMS毫米"), metrics.("平移峰值毫米")].');
styleAxis(ax, '平移误差指标', '', '误差（毫米）');
xticklabels(ax, {'RMS', '峰值'});
legend(ax, metrics.("方法"), 'Location', 'northwest');

ax = nexttile(layout);
bar(ax, [metrics.("姿态RMS度"), metrics.("姿态峰值度")].');
styleAxis(ax, '姿态误差指标', '', '误差（度）');
xticklabels(ax, {'RMS', '峰值'});
legend(ax, metrics.("方法"), 'Location', 'northwest');

colormap(fig, [colors.blue; colors.orange; colors.green]);
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotPerAxis(metrics, fileName)
colors = reportColors();
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50 50 1650 980]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '当前 UKF 六轴位姿误差：逐轴 RMS 与峰值', ...
    'FontSize', 19, 'FontWeight', 'bold');
names = {'测量重建', 'UKF 预测', 'UKF 校正'};

ax = nexttile(layout);
bar(ax, [metrics.("测量重建RMS")(1:3), metrics.("UKF预测RMS")(1:3), metrics.("UKF校正RMS")(1:3)]);
styleAxis(ax, '三轴平移 RMS', '', '误差（毫米）'); xticklabels(ax, metrics.("轴")(1:3));
legend(ax, names, 'Location', 'northwest');

ax = nexttile(layout);
bar(ax, [metrics.("测量重建峰值")(1:3), metrics.("UKF预测峰值")(1:3), metrics.("UKF校正峰值")(1:3)]);
styleAxis(ax, '三轴平移峰值', '', '误差（毫米）'); xticklabels(ax, metrics.("轴")(1:3));
legend(ax, names, 'Location', 'northwest');

ax = nexttile(layout);
bar(ax, [metrics.("测量重建RMS")(4:6), metrics.("UKF预测RMS")(4:6), metrics.("UKF校正RMS")(4:6)]);
styleAxis(ax, '三轴姿态 RMS', '', '误差（度）'); xticklabels(ax, metrics.("轴")(4:6));
legend(ax, names, 'Location', 'northwest');

ax = nexttile(layout);
bar(ax, [metrics.("测量重建峰值")(4:6), metrics.("UKF预测峰值")(4:6), metrics.("UKF校正峰值")(4:6)]);
styleAxis(ax, '三轴姿态峰值', '', '误差（度）'); xticklabels(ax, metrics.("轴")(4:6));
legend(ax, names, 'Location', 'northwest');
colormap(fig, [colors.blue; colors.orange; colors.green]);
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotErrorSources(ablation, fileName)
colors = reportColors();
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50 50 1680 950]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'UKF 位姿随机误差来源：相对无随机噪声基线的误差增量', ...
    'FontSize', 19, 'FontWeight', 'bold');
sourceNames = ["全部随机误差"; "腿长噪声"; "姿态噪声"; "加速度噪声"; "角速度噪声"];
indices = [1, 3, 4, 5, 6];
translationIncrement = max(ablation.("平移RMS毫米")(indices) - ablation.("平移RMS毫米")(2), 0);
rotationIncrement = max(ablation.("姿态RMS度")(indices) - ablation.("姿态RMS度")(2), 0);

ax = nexttile(layout);
bar(ax, translationIncrement, 'FaceColor', colors.blue);
styleAxis(ax, '各类随机误差增加的平移 RMS', '', '相对基线增量（毫米）');
xticks(ax, 1:numel(sourceNames)); xticklabels(ax, sourceNames);
xtickangle(ax, 25);

ax = nexttile(layout);
bar(ax, rotationIncrement, 'FaceColor', colors.orange);
styleAxis(ax, '各类随机误差增加的姿态 RMS', '', '相对基线增量（度）');
xticks(ax, 1:numel(sourceNames)); xticklabels(ax, sourceNames);
xtickangle(ax, 25);

exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotErrorMechanism(metrics, fileName)
colors = reportColors();
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50 50 1750 950]);
ax = axes(fig, 'Position', [0 0 1 1], 'Visible', 'off');
xlim(ax, [0 1]); ylim(ax, [0 1]);
text(ax, 0.04, 0.94, '平台位姿误差来源与传播路径', ...
    'FontSize', 24, 'FontWeight', 'bold', 'Color', colors.dark);
text(ax, 0.04, 0.895, '位姿误差并非来自单一传感器，而是经预测、运动学约束和回零锚点共同传递', ...
    'FontSize', 14.5, 'Color', colors.muted);

diagramBox(ax, [0.04 0.62 0.20 0.20], colors.blueLight, colors.blue, ...
    '六条相对腿长', {'随机噪声与量化误差'; '直接影响平移校正'; '当前测量重建平移误差的主因'});
diagramBox(ax, [0.04 0.34 0.20 0.20], colors.greenLight, colors.green, ...
    '三轴姿态测量', {'直接决定姿态测量精度'; '通过机构几何耦合影响位置'; '航向角权重最保守'});
diagramBox(ax, [0.04 0.08 0.20 0.18], colors.orangeLight, colors.orange, ...
    'IMU 与偏置', {'积分噪声造成短期预测误差'; '残余偏置造成累计漂移'; '腿长与姿态测量持续约束漂移'});

diagramBox(ax, [0.38 0.58 0.25 0.24], colors.indigoLight, colors.indigo, ...
    'UKF 融合过程', {'IMU 预测平台运动'; '相对腿长约束三轴位置'; '姿态测量约束三轴姿态'; '同时在线估计两类偏置'});
diagramBox(ax, [0.38 0.20 0.25 0.22], colors.blueLight, colors.blue, ...
    '回零锚点与几何模型', {'相对编码器不提供绝对腿长'; '锚点偏差转化为绝对位姿偏差'; '几何参数误差形成系统性误差'});

diagramBox(ax, [0.76 0.58 0.20 0.24], colors.greenLight, colors.green, ...
    '相对位姿误差', {'当前仿真重点评估对象'; ...
    sprintf('平移 RMS：%.3f 毫米', metrics.("平移RMS毫米")(3)); ...
    sprintf('姿态 RMS：%.4f 度', metrics.("姿态RMS度")(3))});
diagramBox(ax, [0.76 0.20 0.20 0.22], colors.orangeLight, colors.orange, ...
    '绝对位姿误差', {'除融合误差外'; '还取决于回零锚点标定'; '当前理想锚点仿真未覆盖该风险'});

arrow(ax, [0.24 0.38], [0.72 0.72], colors.blue);
arrow(ax, [0.24 0.38], [0.44 0.65], colors.green);
arrow(ax, [0.24 0.38], [0.17 0.61], colors.orange);
arrow(ax, [0.63 0.76], [0.70 0.70], colors.green);
arrow(ax, [0.63 0.76], [0.31 0.31], colors.orange);
arrow(ax, [0.505 0.505], [0.42 0.58], colors.indigo);

exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function styleAxis(ax, plotTitle, xLabel, yLabel)
grid(ax, 'on'); box(ax, 'on'); title(ax, plotTitle, 'FontWeight', 'bold');
xlabel(ax, xLabel); ylabel(ax, yLabel); ax.FontSize = 11;
end

function value = axisRms(errorValue, scale)
value = scale .* sqrt(mean(errorValue.^2, 2));
end

function value = axisPeak(errorValue, scale)
value = scale .* max(abs(errorValue), [], 2);
end

function value = rmsAll(errorValue)
value = sqrt(mean(errorValue.^2, 'all'));
end

function value = peakAll(errorValue)
value = max(abs(errorValue), [], 'all');
end

function colors = reportColors()
colors.dark = [0.10 0.15 0.23];
colors.muted = [0.34 0.40 0.48];
colors.blue = [0.16 0.43 0.78];
colors.blueLight = [0.91 0.96 1.00];
colors.indigo = [0.36 0.32 0.72];
colors.indigoLight = [0.94 0.93 1.00];
colors.orange = [0.88 0.45 0.14];
colors.orangeLight = [1.00 0.95 0.89];
colors.green = [0.13 0.58 0.38];
colors.greenLight = [0.91 0.98 0.94];
end

function diagramBox(ax, position, faceColor, edgeColor, titleText, lines)
rectangle(ax, 'Position', position, 'Curvature', 0.05, ...
    'FaceColor', faceColor, 'EdgeColor', edgeColor, 'LineWidth', 2);
text(ax, position(1) + 0.015, position(2) + position(4) - 0.035, titleText, ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', edgeColor, 'VerticalAlignment', 'top');
text(ax, position(1) + 0.015, position(2) + position(4) - 0.085, strjoin(lines, newline), ...
    'FontSize', 12.3, 'Color', [0.15 0.19 0.25], 'VerticalAlignment', 'top');
end

function arrow(ax, x, y, color)
annotation(ancestor(ax, 'figure'), 'arrow', x, y, ...
    'LineWidth', 2.2, 'Color', color, 'HeadWidth', 10, 'HeadLength', 10);
end
