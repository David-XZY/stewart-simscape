%% run_07_compare_pwm_pose_force_control - 比较三种 PWM 闭环反馈配置
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
fprintf('\nPWM 三线控制比较结果：%s\n诊断图：%s\n', resultFile, plotFile);
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
plot(comparison.identified.t, 1e3 * vecnorm( ...
    comparison.identifiedTruthFeedback.qTrue(1:3, :) - ...
    comparison.identifiedTruthFeedback.qReference(1:3, :), 2, 1), 'LineWidth', 1.1);
hold on;
plot(comparison.identified.t, 1e3 * vecnorm( ...
    comparison.identified.qTrue(1:3, :) - comparison.identified.qReference(1:3, :), 2, 1), ...
    'LineWidth', 1.1);
grid on; ylabel('平移误差范数 (mm)'); title('辨识闭环位姿反馈源消融');
legend('真值位姿反馈', 'UKF反馈', 'Location', 'best');
nexttile;
plot(comparison.identified.t(2:end), ...
    vecnorm(comparison.identifiedTruthFeedback.estimatedForce(:, 2:end) - ...
    comparison.identifiedTruthFeedback.trueForce(:, 1:end - 1), 2, 1), 'LineWidth', 1.1);
hold on;
plot(comparison.identified.t(2:end), ...
    vecnorm(comparison.identified.estimatedForce(:, 2:end) - ...
    comparison.identified.trueForce(:, 1:end - 1), 2, 1), 'LineWidth', 1.1);
grid on; ylabel('力估计误差范数 (N)'); title('对齐后的辨识力误差');
legend('真值位姿反馈', 'UKF反馈', 'Location', 'best');
nexttile;
plot(comparison.identified.t, comparison.identifiedTruthFeedback.pwm.', 'LineWidth', 0.7);
hold on;
plot(comparison.identified.t, comparison.identified.pwm.', '--', 'LineWidth', 0.7);
grid on; xlabel('时间 (s)'); ylabel('PWM'); title('两种辨识闭环的 PWM 命令');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end
