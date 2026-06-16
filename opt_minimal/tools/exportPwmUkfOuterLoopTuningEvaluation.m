function result = exportPwmUkfOuterLoopTuningEvaluation(outputDir)
% exportPwmUkfOuterLoopTuningEvaluation - 导出联合整定前后的完整轨迹多种子对比
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
    outputDir = fullfile(optRoot, 'results', ['pwm_ukf_outer_loop_tuning_', timestamp]);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

identifierFile = latestFile(fullfile(optRoot, 'results'), 'pwm_force_identifier_*.mat');
sample = load(identifierFile, 'teacher', 'identified');
referenceFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
reference = load(referenceFile, 'refs');
seeds = [7, 19, 41, 73, 101];
configs = {legacyOptions(), makePwmUkfOuterLoopTunedOptions()};
names = ["整定前"; "稳健联合整定后"];
metrics = table();
for configIndex = 1:numel(configs)
    for seedIndex = 1:numel(seeds)
        options = configs{configIndex};
        options.sensorRandomSeed = seeds(seedIndex);
        comparison = comparePwmPoseForceControlLines( ...
            reference.refs, sample.teacher, sample.identified, options);
        line = comparison.identified;
        errorValue = line.qTrue - line.qReference;
        row = table(names(configIndex), seeds(seedIndex), ...
            1e3 * sqrt(mean(errorValue(1:3, :).^2, 'all')), ...
            1e3 * max(abs(errorValue(1:3, :)), [], 'all'), ...
            rad2deg(sqrt(mean(errorValue(4:6, :).^2, 'all'))), ...
            rad2deg(max(abs(errorValue(4:6, :)), [], 'all')), ...
            comparison.metrics.identifiedForceTrackingNrmse, ...
            max(abs(line.pwm), [], 'all'), ...
            'VariableNames', {'configuration', 'seed', 'translationRmsMm', ...
            'translationPeakMm', 'rotationRmsDeg', 'rotationPeakDeg', ...
            'forceTrackingNrmse', 'maxAbsPwm'});
        metrics = [metrics; row]; %#ok<AGROW>
    end
end
summary = groupsummary(metrics, 'configuration', {'mean', 'max'}, ...
    {'translationRmsMm', 'translationPeakMm', 'rotationPeakDeg', ...
    'forceTrackingNrmse', 'maxAbsPwm'});
writetable(metrics, fullfile(outputDir, 'multiseed_metrics.csv'));
writetable(summary, fullfile(outputDir, 'summary_metrics.csv'));
plotComparison(metrics, fullfile(outputDir, '01_before_after_multiseed.png'));
writeReadme(outputDir, summary, identifierFile, referenceFile);
result = struct('outputDir', outputDir, 'metrics', metrics, 'summary', summary, ...
    'legacyOptions', configs{1}, 'tunedOptions', configs{2}, ...
    'identifierFile', identifierFile, 'referenceFile', referenceFile);
save(fullfile(outputDir, 'tuning_evaluation.mat'), 'result');
fprintf('UKF 与外环联合整定评估已导出：%s\n', outputDir);
end

function options = legacyOptions()
options = struct();
options.ukfAccelerationNoiseStd = 9.80665e-3;
options.translationGain = [8.0e4; 8.0e4; 2.4e5];
options.translationRateGain = [2.4e4; 2.4e4; 6.0e4];
options.translationIntegralGain = [3.0e5; 3.0e5; 8.0e5];
options.rotationGain = [3.0e3; 3.0e3; 3.0e3];
options.rotationRateGain = [8.0e2; 8.0e2; 8.0e2];
options.forceCorrectionLimit = 2000;
end

function fileName = latestFile(folder, pattern)
files = dir(fullfile(folder, pattern));
if isempty(files)
    error('exportPwmUkfOuterLoopTuningEvaluation:MissingResult', '未找到结果文件：%s', pattern);
end
[~, index] = max([files.datenum]);
fileName = fullfile(files(index).folder, files(index).name);
end

function plotComparison(metrics, fileName)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1500, 850]);
layout = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
plotMetric(nexttile(layout), metrics, 'translationRmsMm', '平移 RMS (mm)');
plotMetric(nexttile(layout), metrics, 'translationPeakMm', '平移峰值 (mm)');
plotMetric(nexttile(layout), metrics, 'rotationPeakDeg', '旋转峰值 (deg)');
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function plotMetric(ax, metrics, fieldName, yLabel)
names = unique(metrics.configuration, 'stable');
values = zeros(numel(names), 5);
for index = 1:numel(names)
    values(index, :) = metrics{metrics.configuration == names(index), fieldName}.';
end
bar(ax, values); grid(ax, 'on'); ylabel(ax, yLabel); xticklabels(ax, names);
legend(ax, compose('seed %d', [7, 19, 41, 73, 101]), 'Location', 'best');
end

function writeReadme(outputDir, summary, identifierFile, referenceFile)
fid = fopen(fullfile(outputDir, 'README.md'), 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# UKF 与外环稳健联合整定结果\n\n');
fprintf(fid, '- 辨识模型：`%s`\n', identifierFile);
fprintf(fid, '- 参考轨迹：`%s`\n', referenceFile);
fprintf(fid, '- 实际传感器噪声、偏置和回零残差保持不变。\n');
fprintf(fid, '- 使用完整轨迹与五个随机种子进行稳健性复核。\n\n');
fprintf(fid, '## 汇总\n\n');
fprintf(fid, '| 配置 | 平移 RMS 均值 (mm) | 平移峰值最差值 (mm) | 旋转峰值最差值 (deg) | 力跟踪 NRMSE 均值 | PWM 最差峰值 |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|\n');
for index = 1:height(summary)
    fprintf(fid, '| %s | %.4f | %.4f | %.4f | %.5f | %.1f |\n', ...
        summary.configuration(index), summary.mean_translationRmsMm(index), ...
        summary.max_translationPeakMm(index), summary.max_rotationPeakDeg(index), ...
        summary.mean_forceTrackingNrmse(index), summary.max_maxAbsPwm(index));
end
fprintf(fid, '\n## 固化参数\n\n');
fprintf(fid, '- UKF 加速度过程噪声标准差：`0.02 m/s^2`\n');
fprintf(fid, '- 平移位置/速度/积分增益比例：`0.70 / 0.30 / 0.25`\n');
fprintf(fid, '- 旋转位置/速度增益比例：`0.75 / 1.25`\n');
fprintf(fid, '- 外环单腿修正力限幅：`750 N`\n');
fprintf(fid, '\n原始逐种子结果见 `multiseed_metrics.csv`。\n');
end
