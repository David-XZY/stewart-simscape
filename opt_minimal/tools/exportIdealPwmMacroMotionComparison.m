function result = exportIdealPwmMacroMotionComparison(outputDir)
% exportIdealPwmMacroMotionComparison - 对比理想执行器与两条 PWM 控制线的宏观运动
arguments
    outputDir {mustBeTextScalar} = ""
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
resultRoot = fullfile(optRoot, 'results');
idealFile = latestFile(resultRoot, 'result_simscape_pose_force_control_*.mat');
pwmFile = latestFile(resultRoot, 'pwm_pose_force_comparison_*.mat');
ideal = load(idealFile);
pwm = load(pwmFile);
if strlength(string(outputDir)) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(resultRoot, ['ideal_pwm_macro_comparison_', timestamp]);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

runs = alignRuns(ideal, pwm);
metrics = calculateMetrics(runs, ideal.setup.config.characteristicLength);
plotOverview(fullfile(outputDir, '01_macro_motion_overview.png'), runs, metrics);
plotDofErrors(fullfile(outputDir, '02_pose_dof_errors.png'), runs);
writeSummary(fullfile(outputDir, 'summary.md'), idealFile, pwmFile, metrics);
writetable(struct2table(metrics), fullfile(outputDir, 'metrics.csv'));

result = struct('outputDir', string(outputDir), 'idealFile', string(idealFile), ...
    'pwmFile', string(pwmFile), 'runs', runs, 'metrics', metrics);
save(fullfile(outputDir, 'comparison.mat'), 'result');
fprintf('理想执行器与 PWM 宏观运动对比已导出：%s\n', outputDir);
end

function fileName = latestFile(resultRoot, pattern)
files = dir(fullfile(resultRoot, pattern));
if isempty(files)
    error('exportIdealPwmMacroMotionComparison:MissingResult', '未找到结果文件：%s', pattern);
end
[~, index] = max([files.datenum]);
fileName = fullfile(files(index).folder, files(index).name);
end

function runs = alignRuns(ideal, pwm)
time = pwm.comparison.identified.t(:);
q0 = ideal.setup.refs.q0(:).';
idealActual = ideal.report.actualRelativePose + q0;
idealReference = ideal.report.referenceRelativePose + q0;

runs = struct();
runs.time = time;
runs.reference = interp1(ideal.report.poseTime, idealReference, time, 'linear');
runs.ideal = interp1(ideal.report.poseTime, idealActual, time, 'linear');
runs.pwmOracle = pwm.comparison.oracle.qTrue.';
runs.pwmIdentified = pwm.comparison.identified.qTrue.';
runs.errorIdeal = runs.reference - runs.ideal;
runs.errorPwmOracle = runs.reference - runs.pwmOracle;
runs.errorPwmIdentified = runs.reference - runs.pwmIdentified;
runs.labels = {'纯理想力执行器', 'PWM物理对象+真值反馈', 'PWM物理对象+辨识反馈'};
end

function metrics = calculateMetrics(runs, characteristicLength)
names = ["ideal"; "pwm_oracle"; "pwm_identified"];
errors = {runs.errorIdeal, runs.errorPwmOracle, runs.errorPwmIdentified};
metrics = repmat(struct(), 3, 1);
for index = 1:3
    error = errors{index};
    translationNorm = vecnorm(error(:, 1:3), 2, 2);
    rotationNorm = vecnorm(error(:, 4:6), 2, 2);
    equivalentNorm = vecnorm([error(:, 1:3), characteristicLength * error(:, 4:6)], 2, 2);
    metrics(index).name = names(index);
    metrics(index).translationRmsMm = 1e3 * sqrt(mean(translationNorm.^2));
    metrics(index).translationPeakMm = 1e3 * max(translationNorm);
    metrics(index).rotationRmsDeg = rad2deg(sqrt(mean(rotationNorm.^2)));
    metrics(index).rotationPeakDeg = rad2deg(max(rotationNorm));
    metrics(index).endpointTranslationMm = 1e3 * translationNorm(end);
    metrics(index).endpointRotationDeg = rad2deg(rotationNorm(end));
    metrics(index).equivalentPoseRmsMm = 1e3 * sqrt(mean(equivalentNorm.^2));
end
end

function plotOverview(fileName, runs, metrics)
colors = [0.10, 0.45, 0.75; 0.20, 0.65, 0.35; 0.90, 0.35, 0.10];
errors = {runs.errorIdeal, runs.errorPwmOracle, runs.errorPwmIdentified};
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1450, 1050]);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(layout, '纯理想执行器与 PWM 双线宏观运动对比', 'FontWeight', 'bold');

ax = nexttile(layout);
plot3(ax, runs.reference(:, 1), runs.reference(:, 2), runs.reference(:, 3), ...
    'k--', 'LineWidth', 2); hold(ax, 'on');
plot3(ax, runs.ideal(:, 1), runs.ideal(:, 2), runs.ideal(:, 3), ...
    'Color', colors(1, :), 'LineWidth', 1.2);
plot3(ax, runs.pwmOracle(:, 1), runs.pwmOracle(:, 2), runs.pwmOracle(:, 3), ...
    'Color', colors(2, :), 'LineWidth', 1.2);
plot3(ax, runs.pwmIdentified(:, 1), runs.pwmIdentified(:, 2), runs.pwmIdentified(:, 3), ...
    'Color', colors(3, :), 'LineWidth', 1.2);
grid(ax, 'on'); axis(ax, 'equal'); view(ax, 35, 25);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
title(ax, '平台质心空间轨迹');
legend(ax, [{'参考轨迹'}, runs.labels], 'Location', 'best');

ax = nexttile(layout);
for index = 1:3
    plot(ax, runs.time, 1e3 * vecnorm(errors{index}(:, 1:3), 2, 2), ...
        'Color', colors(index, :), 'LineWidth', 1.2); hold(ax, 'on');
end
yline(ax, 1, 'r--', '1 mm 验收线');
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '平移误差范数 (mm)');
title(ax, '宏观平移跟踪误差'); legend(ax, runs.labels, 'Location', 'best');

ax = nexttile(layout);
for index = 1:3
    plot(ax, runs.time, rad2deg(vecnorm(errors{index}(:, 4:6), 2, 2)), ...
        'Color', colors(index, :), 'LineWidth', 1.2); hold(ax, 'on');
end
yline(ax, 1, 'r--', '1 deg 验收线');
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '旋转误差范数 (deg)');
title(ax, '宏观旋转跟踪误差'); legend(ax, runs.labels, 'Location', 'best');

ax = nexttile(layout);
values = [[metrics.translationRmsMm].', [metrics.translationPeakMm].', ...
    [metrics.rotationRmsDeg].', [metrics.rotationPeakDeg].'];
bars = bar(ax, values.', 'grouped');
for index = 1:3
    bars(index).FaceColor = colors(index, :);
end
set(ax, 'XTickLabel', {'平移RMS(mm)', '平移峰值(mm)', '旋转RMS(deg)', '旋转峰值(deg)'});
grid(ax, 'on'); title(ax, '宏观运动指标');
legend(ax, runs.labels, 'Location', 'best');
exportgraphics(fig, fileName, 'Resolution', 200);
close(fig);
end

function plotDofErrors(fileName, runs)
colors = [0.10, 0.45, 0.75; 0.20, 0.65, 0.35; 0.90, 0.35, 0.10];
errors = {runs.errorIdeal, runs.errorPwmOracle, runs.errorPwmIdentified};
titles = {'X 平移误差', 'Y 平移误差', 'Z 平移误差', ...
    'Roll 误差', 'Pitch 误差', 'Yaw 误差'};
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1500, 1000]);
layout = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for dofIndex = 1:6
    ax = nexttile(layout);
    for runIndex = 1:3
        value = errors{runIndex}(:, dofIndex);
        if dofIndex <= 3
            value = 1e3 * value;
        else
            value = rad2deg(value);
        end
        plot(ax, runs.time, value, 'Color', colors(runIndex, :), 'LineWidth', 1);
        hold(ax, 'on');
    end
    grid(ax, 'on'); title(ax, titles{dofIndex}); xlabel(ax, '时间 (s)');
    if dofIndex <= 3
        ylabel(ax, '误差 (mm)');
    else
        ylabel(ax, '误差 (deg)');
    end
end
legend(nexttile(layout, 1), runs.labels, 'Location', 'best');
exportgraphics(fig, fileName, 'Resolution', 200);
close(fig);
end

function writeSummary(fileName, idealFile, pwmFile, metrics)
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# 纯理想执行器与 PWM 双线宏观运动对比\n\n');
fprintf(fid, '- 理想执行器结果：`%s`\n', idealFile);
fprintf(fid, '- PWM 双线结果：`%s`\n\n', pwmFile);
fprintf(fid, '> 注意：纯理想执行器来自 Run02 Simscape 理想力源及其位姿控制器；PWM 两条线来自高保真 PWM 对象及力内环。该对比反映系统级宏观表现，不是仅替换执行器的严格控制变量实验。\n\n');
fprintf(fid, '| 方案 | 平移 RMS (mm) | 平移峰值 (mm) | 旋转 RMS (deg) | 旋转峰值 (deg) | 终点平移 (mm) | 终点旋转 (deg) | 等效位姿 RMS (mm) |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|\n');
labels = {'纯理想力执行器', 'PWM物理对象+真值反馈', 'PWM物理对象+辨识反馈'};
for index = 1:3
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f |\n', ...
        labels{index}, metrics(index).translationRmsMm, metrics(index).translationPeakMm, ...
        metrics(index).rotationRmsDeg, metrics(index).rotationPeakDeg, ...
        metrics(index).endpointTranslationMm, metrics(index).endpointRotationDeg, ...
        metrics(index).equivalentPoseRmsMm);
end
fprintf(fid, '\n- PWM 真值反馈相对理想执行器的平移 RMS 放大倍数：`%.2f`\n', ...
    metrics(2).translationRmsMm / metrics(1).translationRmsMm);
fprintf(fid, '- PWM 辨识反馈相对真值反馈的平移 RMS 放大倍数：`%.2f`\n', ...
    metrics(3).translationRmsMm / metrics(2).translationRmsMm);
fprintf(fid, '- PWM 辨识反馈相对真值反馈的旋转 RMS 放大倍数：`%.2f`\n', ...
    metrics(3).rotationRmsDeg / metrics(2).rotationRmsDeg);
end
