%% run_02_simscape_length_control - 力输入位姿轨迹跟踪入口
% 控制结构：u = references.uFF + uFeedback。
% references.uFF 为 IHSID 逆动力学前馈力，uFeedback 由位姿误差动态反馈生成。
% 位姿参考由节点 q/qd 经 10 ms 分段三次 Hermite 重建。
clearvars -except simscapeRunMode simscapeTrajectoryFile poseForceConfigOverrides poseForceBaselineFile;
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
if ~exist('poseForceConfigOverrides', 'var')
    poseForceConfigOverrides = struct( ...
        'bandwidthHz', 15, 'gainScale', 0.45, 'rotationGainScale', 1.4);
end
if ~exist('poseForceBaselineFile', 'var')
    poseForceBaselineFile = "";
end
simscapeRunMode = validatestring(simscapeRunMode, {'auto', 'manual'});

setup = prepareSimscapePoseForceControl(simscapeTrajectoryFile, poseForceConfigOverrides);
fprintf('\n===== Simscape 力输入位姿轨迹跟踪 =====\n');
fprintf('运行模式：%s\n', simscapeRunMode);
fprintf('参考轨迹：%s\n', setup.trajectoryFile);
fprintf('仿真时长：%.3f s\n', setup.refs.t(end));
fprintf('反馈带宽：%.3f Hz\n', setup.design.bandwidthHz);
fprintf('总增益/转动增益缩放：%.3f / %.3f\n', ...
    setup.design.gainScale, setup.design.rotationGainScale);
fprintf('重力启用：%d\n', setup.config.gravityEnabled);

if strcmp(simscapeRunMode, 'manual')
    open_system(setup.modelName);
    fprintf('\n模型已按力输入位姿跟踪配置并打开，可直接点击 Simulink 运行。\n');
    return;
end

modelCleanup = onCleanup(@() closeModelWithoutSaving(setup.modelName));
resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
tag = ['simscape_pose_force_control_', timestamp];
simulationOutput = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapePoseForceControl( ...
    simulationOutput.get('simout'), setup.refs, setup.model, setup.design, setup.config);

[baselineFile, baseline] = loadRun03Baseline(poseForceBaselineFile, resultDir);
if isempty(baseline)
    comparison = struct();
    comparisonPassed = false;
else
    comparison = comparePoseTrackingPerformance( ...
        report, baseline.report, setup.config.characteristicLength);
    comparisonPassed = comparison.passed;
end
overallPassed = report.passed && comparisonPassed;
printSummary(report, comparison, baselineFile, overallPassed);

resultFile = fullfile(resultDir, ['result_', tag, '.mat']);
summaryFile = fullfile(resultDir, ['summary_', tag, '.txt']);
plotFile = fullfile(resultDir, ['tracking_', tag, '.png']);
save(resultFile, 'setup', 'report', 'comparison', 'baselineFile', ...
    'poseForceConfigOverrides', 'simscapeTrajectoryFile');
writeSummary(summaryFile, resultFile, setup, report, comparison, baselineFile, overallPassed);
plotTracking(plotFile, report);

fprintf('\n结果 MAT：%s\n摘要：%s\n检查图：%s\n', resultFile, summaryFile, plotFile);
fprintf('力输入位姿轨迹跟踪与 Run03 对比验收通过：%d\n', overallPassed);
clear modelCleanup;
if ~overallPassed
    error('run_02_simscape_length_control:AcceptanceFailed', ...
        '力输入位姿轨迹跟踪或 Run03 对比未通过，诊断结果已保存。');
end

function printSummary(report, comparison, baselineFile, overallPassed)
fprintf('峰值误差：腿长=%.6e m，平移=%.6e m，转角=%.6e rad\n', ...
    report.metrics.maxLengthTrackingPeak, report.metrics.maxTranslationPeak, ...
    report.metrics.maxRotationPeak);
fprintf('最大腿速/腿加速度/总力：%.6e m/s，%.6e m/s^2，%.3f N\n', ...
    report.metrics.maxAbsLegSpeed, report.metrics.maxAbsLegAcceleration, ...
    report.metrics.maxAbsControlForce);
fprintf('硬验收通过：%d\n', report.passed);
if isempty(fieldnames(comparison))
    fprintf('Run03 默认基准：未找到\n');
else
    fprintf('Run03 默认基准：%s\n', baselineFile);
    fprintf('位姿综合分=%.6f，平移峰值比=%.6f，转角峰值比=%.6f，对比通过=%d\n', ...
        comparison.score, comparison.translationPeakRatio, ...
        comparison.rotationPeakRatio, comparison.passed);
end
fprintf('总体通过：%d\n', overallPassed);
end

function writeSummary(fileName, resultFile, setup, report, comparison, baselineFile, overallPassed)
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Simscape force-input pose tracking summary\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'trajectoryFile: %s\n', setup.trajectoryFile);
fprintf(fid, 'baselineFile: %s\n', baselineFile);
fprintf(fid, 'bandwidthHz: %.12g\n', setup.design.bandwidthHz);
fprintf(fid, 'gainScale: %.12g\n', setup.design.gainScale);
fprintf(fid, 'rotationGainScale: %.12g\n', setup.design.rotationGainScale);
fprintf(fid, 'gravityEnabled: %d\n', setup.config.gravityEnabled);
fprintf(fid, 'overallPassed: %d\n', overallPassed);
writeFields(fid, report.acceptance);
writeFields(fid, report.metrics);
if ~isempty(fieldnames(comparison))
    writeFields(fid, comparison);
end
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
grid on; ylabel('平移误差 (mm)'); title('位姿平移误差');
nexttile; plot(report.poseTime, rad2deg(report.poseError(:, 4:6)), 'LineWidth', 1);
grid on; ylabel('转角误差 (deg)'); title('位姿转角误差');
nexttile; plot(report.forceTime, report.controlForce, 'LineWidth', 1);
hold on; yline(2000, 'r--'); yline(-2000, 'r--');
grid on; xlabel('时间 (s)'); ylabel('驱动力 (N)'); title('六腿总驱动力');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function [baselineFile, baseline] = loadRun03Baseline(requestedFile, resultDir)
baseline = [];
if strlength(string(requestedFile)) > 0
    candidates = dir(char(requestedFile));
else
    candidates = dir(fullfile(resultDir, 'result_simscape_length_cascade_*.mat'));
    [~, order] = sort([candidates.datenum], 'descend');
    candidates = candidates(order);
end
baselineFile = "";
for index = 1:numel(candidates)
    if isfield(candidates, 'folder')
        candidateFile = fullfile(candidates(index).folder, candidates(index).name);
    else
        candidateFile = candidates(index).name;
    end
    candidate = load(candidateFile);
    if ~isfield(candidate, 'setup') || ~isfield(candidate, 'report') || ~candidate.report.passed
        continue;
    end
    design = candidate.setup.design;
    if isfield(design, 'positionGainScale') && isfield(design, 'velocityGainScale') && ...
            abs(design.positionGainScale - 1) < 1e-12 && abs(design.velocityGainScale - 0.7) < 1e-12
        baseline = candidate;
        baselineFile = string(candidateFile);
        return;
    end
end
end

function closeModelWithoutSaving(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
