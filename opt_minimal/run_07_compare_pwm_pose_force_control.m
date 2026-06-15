%% run_07_compare_pwm_pose_force_control - 比较真值基准线与辨识反馈线
clearvars -except pwmIdentifierFile pwmTrajectoryFile pwmComparisonOverrides;
close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));
resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
if ~exist('pwmIdentifierFile', 'var')
    pwmIdentifierFile = "";
end
if ~exist('pwmTrajectoryFile', 'var') || strlength(string(pwmTrajectoryFile)) == 0
    pwmTrajectoryFile = fullfile(optRoot, 'examples', ...
        'ihsid_40x20_limited_memory', 'simscape_references.mat');
end
if ~exist('pwmComparisonOverrides', 'var')
    pwmComparisonOverrides = struct();
end

identifierFile = resolveLatestFile(pwmIdentifierFile, resultDir, 'pwm_force_identifier_*.mat');
if strlength(identifierFile) == 0
    teacher = makeHighFidelityPwmActuator();
    dataset = generatePwmIdentificationDataset(teacher);
    identified = trainGrayNarxForceIdentifier(dataset, teacher);
else
    sample = load(identifierFile, 'teacher', 'identified');
    teacher = sample.teacher;
    identified = sample.identified;
end
trajectory = load(pwmTrajectoryFile, 'refs');
comparison = comparePwmPoseForceControlLines( ...
    trajectory.refs, teacher, identified, pwmComparisonOverrides);
report = evaluatePwmPoseForceControlComparison(comparison, teacher);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultFile = fullfile(resultDir, ['pwm_pose_force_comparison_', timestamp, '.mat']);
plotFile = fullfile(resultDir, ['pwm_pose_force_comparison_', timestamp, '.png']);
save(resultFile, 'teacher', 'identified', 'comparison', 'report', ...
    'identifierFile', 'pwmTrajectoryFile', 'pwmComparisonOverrides');
plotComparison(plotFile, comparison);
fprintf('\nPWM 双线控制比较结果：%s\n诊断图：%s\n', resultFile, plotFile);
fprintf('辨识线力 NRMSE：%.4f，平移峰值：%.3f mm，旋转峰值：%.3f deg，通过：%d\n', ...
    report.metrics.identifiedForceTrackingNrmse, ...
    1e3 * report.metrics.identifiedTranslationPeak, ...
    rad2deg(report.metrics.identifiedRotationPeak), report.passed);
if ~report.passed
    error('run_07_compare_pwm_pose_force_control:AcceptanceFailed', 'PWM 辨识反馈线未通过完整轨迹验收。');
end

function fileName = resolveLatestFile(requestedFile, resultDir, pattern)
fileName = string(requestedFile);
if strlength(fileName) > 0
    if ~isfile(fileName)
        error('run_07_compare_pwm_pose_force_control:FileNotFound', '未找到辨识模型文件：%s', fileName);
    end
    return;
end
candidates = dir(fullfile(resultDir, pattern));
if isempty(candidates)
    fileName = "";
    return;
end
[~, index] = max([candidates.datenum]);
fileName = string(fullfile(candidates(index).folder, candidates(index).name));
end

function plotComparison(fileName, comparison)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1200, 900]);
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile;
plot(comparison.identified.t, 1e3 * ...
    (comparison.identified.qTrue(1:3, :) - comparison.identified.qReference(1:3, :)).');
grid on; ylabel('平移误差 (mm)'); title('辨识反馈线位姿跟踪');
nexttile;
plot(comparison.identified.t, comparison.identified.targetForce.', '--', ...
    comparison.identified.t, comparison.identified.trueForce.', 'LineWidth', 0.8);
grid on; ylabel('力 (N)'); title('目标力与高保真真力');
nexttile;
plot(comparison.identified.t(2:end), comparison.identified.estimatedForce(:, 2:end).' - ...
    comparison.identified.trueForce(:, 1:end - 1).', 'LineWidth', 0.8);
grid on; xlabel('时间 (s)'); ylabel('估计误差 (N)'); title('灰箱加残差 NARX 力估计误差');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end
