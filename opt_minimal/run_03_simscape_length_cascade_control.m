%% run_03_simscape_length_cascade_control - 纯长度串级控制入口
% 仅使用轨迹中的 t/q/qd，在仿真前逐腿自动整定位置 P 与速度 PIDF。
% 设置 lengthCascadeRunMode='manual' 后，本脚本只准备变量并打开模型。
clearvars -except lengthCascadeRunMode lengthCascadeTrajectoryFile lengthCascadeConfigOverrides;
close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(optRoot, 'integration'));

if ~exist('lengthCascadeRunMode', 'var')
    lengthCascadeRunMode = 'auto';
end
if ~exist('lengthCascadeTrajectoryFile', 'var')
    lengthCascadeTrajectoryFile = "";
end
if ~exist('lengthCascadeConfigOverrides', 'var')
    lengthCascadeConfigOverrides = struct();
end
lengthCascadeRunMode = validatestring(lengthCascadeRunMode, {'auto', 'manual'});

setup = prepareSimscapeLengthCascadeControl( ...
    lengthCascadeTrajectoryFile, lengthCascadeConfigOverrides);

fprintf('\n===== Simscape 纯长度串级控制 =====\n');
fprintf('运行模式：%s\n', lengthCascadeRunMode);
fprintf('参考轨迹：%s\n', setup.trajectoryFile);
fprintf('仿真时长：%.3f s\n', setup.refs.t(end));
fprintf('采样周期：%.4f s\n', setup.config.sampleTime);
fprintf('内环/外环带宽：%.3f / %.3f Hz\n', ...
    setup.design.innerBandwidthHz, setup.design.outerBandwidthHz);
fprintf('位置环/速度环增益缩放：%.3f / %.3f\n', ...
    setup.design.positionGainScale, setup.design.velocityGainScale);
fprintf('重力启用：%d\n', setup.config.gravityEnabled);

if strcmp(lengthCascadeRunMode, 'manual')
    open_system(setup.modelName);
    fprintf('\n模型已完成纯长度配置并打开；可直接点击 Simulink 运行。\n');
    return;
end

modelCleanup = onCleanup(@() closeModelWithoutSaving(setup.modelName));
resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
tag = ['simscape_length_cascade_', timestamp];
simulationOutput = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
simout = simulationOutput.get('simout');
report = evaluateSimscapeLengthCascadeControl( ...
    simout, setup.refs, setup.model, setup.design, setup.config);
printSummary(report, setup.design);

resultFile = fullfile(resultDir, ['result_', tag, '.mat']);
summaryFile = fullfile(resultDir, ['summary_', tag, '.txt']);
plotFile = fullfile(resultDir, ['tracking_', tag, '.png']);
save(resultFile, 'setup', 'report', 'lengthCascadeTrajectoryFile', ...
    'lengthCascadeConfigOverrides');
writeSummary(summaryFile, resultFile, setup, report);
plotTracking(plotFile, report);

fprintf('\n结果 MAT：%s\n摘要：%s\n检查图：%s\n', ...
    resultFile, summaryFile, plotFile);
fprintf('纯长度串级控制验收通过：%d\n', report.passed);
clear modelCleanup;
if ~report.passed
    error('run_03_simscape_length_cascade_control:AcceptanceFailed', ...
        '纯长度串级控制未通过硬验收，诊断结果已保存。');
end

function printSummary(report, design)
% printSummary - 输出不含力指标的纯长度验收摘要
fprintf('自动整定稳定=%d\n', design.stable);
fprintf('峰值误差：腿长=%.6e m，平移=%.6e m，转角=%.6e rad\n', ...
    report.metrics.maxLengthTrackingPeak, ...
    report.metrics.maxTranslationPeak, report.metrics.maxRotationPeak);
fprintf('最大腿速=%.6e m/s，最大腿加速度=%.6e m/s^2\n', ...
    report.metrics.maxAbsLegSpeed, report.metrics.maxAbsLegAcceleration);
fprintf('硬验收：finite=%d, length=%d, speed=%d, acceleration=%d, tracking=%d, passed=%d\n', ...
    report.acceptance.finitePassed, report.acceptance.lengthPassed, ...
    report.acceptance.speedPassed, report.acceptance.accelerationPassed, ...
    report.acceptance.trackingPassed, report.passed);
end

function writeSummary(fileName, resultFile, setup, report)
% writeSummary - 保存纯长度串级控制摘要
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Simscape pure length cascade control summary\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'trajectoryFile: %s\n', setup.trajectoryFile);
fprintf(fid, 'sampleTime: %.12g\n', setup.config.sampleTime);
fprintf(fid, 'innerBandwidthHz: %.12g\n', setup.design.innerBandwidthHz);
fprintf(fid, 'outerBandwidthHz: %.12g\n', setup.design.outerBandwidthHz);
fprintf(fid, 'positionGainScale: %.12g\n', setup.design.positionGainScale);
fprintf(fid, 'velocityGainScale: %.12g\n', setup.design.velocityGainScale);
fprintf(fid, 'gravityEnabled: %d\n', setup.config.gravityEnabled);
fprintf(fid, 'passed: %d\n', report.passed);
writeFields(fid, report.acceptance);
writeFields(fid, report.metrics);
end

function writeFields(fid, values)
% writeFields - 将结构体字段逐项写入摘要
names = fieldnames(values);
for index = 1:numel(names)
    fprintf(fid, '%s: %s\n', names{index}, mat2str(values.(names{index}), 12));
end
end

function plotTracking(fileName, report)
% plotTracking - 绘制长度、速度和位姿误差
figureHandle = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1200, 900]);
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile;
plot(report.time, report.Lref, '--', report.time, report.Lactual, 'LineWidth', 1);
grid on; ylabel('\DeltaL (m)'); title('腿长参考与实际值');
nexttile;
plot(report.time, report.Ldref, '--', report.time, report.LdCmd, 'LineWidth', 1);
grid on; ylabel('腿速 (m/s)'); title('腿速参考与限幅命令');
nexttile;
plot(report.poseTime, report.poseError, 'LineWidth', 1);
grid on; xlabel('时间 (s)'); ylabel('位姿误差'); title('相对位姿跟踪误差');
exportgraphics(figureHandle, fileName, 'Resolution', 180);
close(figureHandle);
end

function closeModelWithoutSaving(modelName)
% closeModelWithoutSaving - 丢弃运行期配置并关闭模型
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
