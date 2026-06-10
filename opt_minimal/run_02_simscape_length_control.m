%% run_02_simscape_length_control - IHSID 轨迹的 Simscape 稳定跟踪入口
% 默认自动运行 10 Hz 力前馈加长度反馈闭环。
% 设置 simscapeRunMode='manual' 后，本脚本只准备并打开模型，由用户点击 Simulink 运行。
clearvars -except simscapeRunMode simscapeTrajectoryFile simscapeBandwidthHz useForceFeedforward;
close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(optRoot, 'integration'));

if ~exist('simscapeRunMode', 'var')
    simscapeRunMode = 'auto';
end
if ~exist('simscapeTrajectoryFile', 'var')
    simscapeTrajectoryFile = "";
end
if ~exist('simscapeBandwidthHz', 'var')
    simscapeBandwidthHz = 10;
end
if ~exist('useForceFeedforward', 'var')
    useForceFeedforward = true;
end
simscapeRunMode = validatestring(simscapeRunMode, {'auto', 'manual'});

setup = prepareSimscapeLengthControl( ...
    simscapeTrajectoryFile, simscapeBandwidthHz, useForceFeedforward);
modelName = setup.modelName;
refs = setup.refs;
references = setup.references;
model = setup.model;
scene = setup.scene;
simscapeData = setup.simscapeData;
design = setup.design;

fprintf('\n===== Simscape 力前馈加长度反馈稳定跟踪 =====\n');
fprintf('运行模式：%s\n', simscapeRunMode);
fprintf('参考轨迹：%s\n', setup.trajectoryFile);
fprintf('仿真时长：%.3f s\n', refs.t(end));
fprintf('反馈带宽：%.3f Hz\n', design.bandwidthHz);
fprintf('力前馈启用：%d\n', useForceFeedforward);

if strcmp(simscapeRunMode, 'manual')
    open_system(modelName);
    fprintf('\n模型已完成配置并打开。可调整基础工作区中的 references、Kl 等变量，\n');
    fprintf('随后点击 Simulink 运行按钮。模型停止时间已设为 %.3f s。\n', refs.t(end));
    fprintf('关闭模型时请勿保存运行期配置，以保持通用 SLX 不变。\n');
    return;
end

modelCleanup = onCleanup(@() closeModelWithoutSaving(modelName));
resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
tag = ['simscape_length_control_', timestamp];
diaryFile = fullfile(resultDir, ['console_', tag, '.txt']);
diary(diaryFile);
diaryCleanup = onCleanup(@() diary('off'));

simulationOutput = sim(modelName, ...
    'StopTime', num2str(refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
simout = simulationOutput.get('simout');
report = evaluateSimscapeLengthControl(simout, refs, model, design);
printSummary(report, design);

resultFile = fullfile(resultDir, ['result_', tag, '.mat']);
summaryFile = fullfile(resultDir, ['summary_', tag, '.txt']);
plotFile = fullfile(resultDir, ['tracking_', tag, '.png']);
save(resultFile, 'model', 'scene', 'simscapeData', 'refs', 'references', ...
    'design', 'report', 'useForceFeedforward', 'simscapeBandwidthHz', ...
    'simscapeTrajectoryFile');
writeSummary(summaryFile, resultFile, report, design, simscapeData, ...
    setup.trajectoryFile, useForceFeedforward);
plotTracking(plotFile, report);

fprintf('\n结果 MAT：%s\n', resultFile);
fprintf('摘要：%s\n', summaryFile);
fprintf('检查图：%s\n', plotFile);
fprintf('完整跟踪验收通过：%d\n', report.passed);
clear modelCleanup;

if ~report.passed
    error('run_02_simscape_length_control:AcceptanceFailed', ...
        'Simscape 完整跟踪未通过验收，诊断结果已保存。');
end

function printSummary(report, design)
% printSummary - 输出完整跟踪验收摘要
fprintf('线性闭环稳定=%d，低频秩=%d\n', design.stable, design.lowFrequencyRank);
fprintf('腿长范围=[%.6f, %.6f] m，最大总控制力=%.3f N\n', ...
    report.metrics.minAbsoluteLength, report.metrics.maxAbsoluteLength, ...
    report.metrics.maxAbsControlForce);
fprintf('峰值误差：腿长=%.6e m，平移=%.6e m，转角=%.6e rad\n', ...
    report.metrics.maxLengthTrackingPeak, report.metrics.maxTranslationPeak, ...
    report.metrics.maxRotationPeak);
fprintf('硬验收：finite=%d, length=%d, force=%d, tracking=%d, passed=%d\n', ...
    report.acceptance.finitePassed, report.acceptance.lengthPassed, ...
    report.acceptance.forcePassed, report.acceptance.trackingPassed, report.passed);
end

function writeSummary(fileName, resultFile, report, design, simscapeData, ...
        trajectoryFile, useForceFeedforward)
% writeSummary - 保存稳定跟踪摘要
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'IHSID Simscape stable tracking summary\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'trajectoryFile: %s\n', trajectoryFile);
fprintf(fid, 'useForceFeedforward: %d\n', useForceFeedforward);
fprintf(fid, 'bandwidthHz: %.12g\n', design.bandwidthHz);
fprintf(fid, 'stable: %d\n', design.stable);
fprintf(fid, 'lowFrequencyRank: %d\n', design.lowFrequencyRank);
fprintf(fid, 'passed: %d\n', report.passed);
acceptanceFields = fieldnames(report.acceptance);
for fieldIndex = 1:numel(acceptanceFields)
    fieldName = acceptanceFields{fieldIndex};
    fprintf(fid, '%s: %d\n', fieldName, report.acceptance.(fieldName));
end
metricFields = fieldnames(report.metrics);
for fieldIndex = 1:numel(metricFields)
    fieldName = metricFields{fieldIndex};
    fprintf(fid, '%s: %s\n', fieldName, mat2str(report.metrics.(fieldName), 12));
end
fprintf(fid, 'mappingTotalMassError: %.12e\n', simscapeData.mapping.totalMassError);
fprintf(fid, 'mappingComError: %.12e\n', simscapeData.mapping.comError);
fprintf(fid, 'mappingInertiaError: %.12e\n', simscapeData.mapping.inertiaError);
fprintf(fid, 'closedLoopPoles: %s\n', mat2str(design.closedLoopPoles, 12));
end

function plotTracking(fileName, report)
% plotTracking - 输出长度、位姿和控制力检查图
figureHandle = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1200, 900]);
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(report.time, report.referenceRelativeLength, '--', 'LineWidth', 1);
hold on;
plot(report.time, report.actualRelativeLength, 'LineWidth', 1);
grid on;
ylabel('\DeltaL (m)');
title('腿长参考与实际值');

nexttile;
plot(report.poseTime, report.poseError, 'LineWidth', 1);
grid on;
ylabel('位姿误差');
title('相对位姿跟踪误差');

nexttile;
plot(report.forceTime, report.controlForce, 'LineWidth', 1);
hold on;
yline(2000, 'r--');
yline(-2000, 'r--');
grid on;
xlabel('时间 (s)');
ylabel('控制力 (N)');
title('总控制力（前馈+反馈）');

exportgraphics(figureHandle, fileName, 'Resolution', 180);
close(figureHandle);
end

function closeModelWithoutSaving(modelName)
% closeModelWithoutSaving - 丢弃运行期配置并关闭通用模型
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
