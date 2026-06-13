%% run_04_simscape_pose_length_control - 位姿反馈纯腿长输入控制入口
% 在 Run03 长度串级控制基础上，先低通平台位姿误差，再通过随参考位姿
% 变化的雅可比转换为腿长参考修正；最终执行器仍为理想腿长输入。
clearvars -except poseLengthRunMode poseLengthTrajectoryFile poseLengthConfigOverrides;
close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(optRoot, 'integration'));

if ~exist('poseLengthRunMode', 'var')
    poseLengthRunMode = 'auto';
end
if ~exist('poseLengthTrajectoryFile', 'var')
    poseLengthTrajectoryFile = "";
end
if ~exist('poseLengthConfigOverrides', 'var')
    poseLengthConfigOverrides = struct();
end
poseLengthRunMode = validatestring(poseLengthRunMode, {'auto', 'manual'});

setup = prepareSimscapePoseLengthControl( ...
    poseLengthTrajectoryFile, poseLengthConfigOverrides);

fprintf('\n===== Simscape 位姿反馈纯腿长输入控制 =====\n');
fprintf('运行模式：%s\n', poseLengthRunMode);
fprintf('参考轨迹：%s\n', setup.trajectoryFile);
fprintf('仿真时长：%.3f s\n', setup.refs.t(end));
fprintf('位置环/速度环增益缩放：%.3f / %.3f\n', ...
    setup.design.positionGainScale, setup.design.velocityGainScale);
fprintf('位姿反馈增益/腿长修正限幅：%.3f / %.3f mm\n', ...
    setup.design.poseFeedbackGain, 1e3 * setup.design.poseCorrectionLimit);
fprintf('位姿反馈低通截止频率：%.3f Hz\n', setup.design.poseFeedbackFilterHz);
fprintf('重力启用：%d\n', setup.config.gravityEnabled);

if strcmp(poseLengthRunMode, 'manual')
    open_system(setup.modelName);
    fprintf('\n模型已完成位姿反馈纯腿长输入配置并打开，可直接点击 Simulink 运行。\n');
    return;
end

modelCleanup = onCleanup(@() closeModelWithoutSaving(setup.modelName));
resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
tag = ['simscape_pose_length_control_', timestamp];
simulationOutput = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapePoseLengthControl( ...
    simulationOutput.get('simout'), setup.refs, setup.model, setup.design, setup.config);
printSummary(report, setup.design);

resultFile = fullfile(resultDir, ['result_', tag, '.mat']);
summaryFile = fullfile(resultDir, ['summary_', tag, '.txt']);
plotFile = fullfile(resultDir, ['tracking_', tag, '.png']);
save(resultFile, 'setup', 'report', 'poseLengthTrajectoryFile', ...
    'poseLengthConfigOverrides');
writeSummary(summaryFile, resultFile, setup, report);
plotTracking(plotFile, report);

fprintf('\n结果 MAT：%s\n摘要：%s\n检查图：%s\n', resultFile, summaryFile, plotFile);
fprintf('位姿反馈纯腿长输入控制验收通过：%d\n', report.passed);
clear modelCleanup;
if ~report.passed
    error('run_04_simscape_pose_length_control:AcceptanceFailed', ...
        '位姿反馈纯腿长输入控制未通过硬验收，诊断结果已保存。');
end

function printSummary(report, design)
fprintf('自动整定稳定=%d，位姿映射秩=%d\n', design.stable, design.poseLengthRank);
fprintf('峰值误差：腿长=%.6e m，平移=%.6e m，转角=%.6e rad\n', ...
    report.metrics.maxLengthTrackingPeak, report.metrics.maxTranslationPeak, ...
    report.metrics.maxRotationPeak);
fprintf('最大腿速/腿加速度/位姿腿长修正：%.6e m/s，%.6e m/s^2，%.6e m\n', ...
    report.metrics.maxAbsLegSpeed, report.metrics.maxAbsLegAcceleration, ...
    report.metrics.maxAbsPoseLengthCorrection);
fprintf('硬验收通过：%d\n', report.passed);
end

function writeSummary(fileName, resultFile, setup, report)
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Simscape pose-feedback pure length-input control summary\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'trajectoryFile: %s\n', setup.trajectoryFile);
fprintf(fid, 'poseFeedbackGain: %.12g\n', setup.design.poseFeedbackGain);
fprintf(fid, 'poseCorrectionLimit: %.12g\n', setup.design.poseCorrectionLimit);
fprintf(fid, 'poseFeedbackFilterHz: %.12g\n', setup.design.poseFeedbackFilterHz);
fprintf(fid, 'positionGainScale: %.12g\n', setup.design.positionGainScale);
fprintf(fid, 'velocityGainScale: %.12g\n', setup.design.velocityGainScale);
fprintf(fid, 'gravityEnabled: %d\n', setup.config.gravityEnabled);
fprintf(fid, 'passed: %d\n', report.passed);
writeFields(fid, report.acceptance);
writeFields(fid, report.metrics);
end

function writeFields(fid, values)
names = fieldnames(values);
for index = 1:numel(names)
    value = values.(names{index});
    if isnumeric(value) || islogical(value)
        fprintf(fid, '%s: %s\n', names{index}, mat2str(value, 12));
    end
end
end

function plotTracking(fileName, report)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1200, 900]);
tiledlayout(4, 1, 'TileSpacing', 'compact');
nexttile; plot(report.time, report.lengthError * 1e3, 'LineWidth', 1);
grid on; ylabel('腿长误差 (mm)'); title('六腿长度跟踪误差');
nexttile; plot(report.poseTime, report.poseError(:, 1:3) * 1e3, 'LineWidth', 1);
grid on; ylabel('平移误差 (mm)'); title('平台平移误差');
nexttile; plot(report.poseTime, rad2deg(report.poseError(:, 4:6)), 'LineWidth', 1);
grid on; ylabel('转角误差 (deg)'); title('平台转角误差');
nexttile; plot(report.poseLengthCorrectionTime, report.poseLengthCorrection * 1e3, 'LineWidth', 1);
grid on; xlabel('时间 (s)'); ylabel('修正量 (mm)'); title('位姿反馈生成的腿长参考修正');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function closeModelWithoutSaving(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
