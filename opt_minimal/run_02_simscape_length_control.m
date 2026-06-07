%% run_02_simscape_length_control - IHSID 轨迹的 Simscape 纯长度反馈闭环
% 控制器只使用 references.rL-dLm，不使用优化力前馈或位姿反馈。
% 几何与刚体参数读取当前优化模型；杆件每段 1e-3 kg 仅为数值正则质量。
clear; close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

sampleFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
sample = load(sampleFile, 'refs');
refs = sample.refs;

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
simscapeData = buildSimscapeLengthControlData(model, scene);
[~, references] = exportTrajectoryToSimscape(trajectoryFromRefs(refs), scene, []);

stewart = simscapeData.stewart;
payload = simscapeData.payload;
ground = simscapeData.ground;
disturbances = simscapeData.disturbances;
controller = initializeController('type', 'open-loop');
Kl = ss(zeros(6));

modelName = 'stewart_platform_model';
modelFile = fullfile(projectRoot, 'matlab', [modelName, '.slx']);
load_system(modelFile);
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

fprintf('\n===== Simscape 纯长度反馈闭环 =====\n');
fprintf('标准轨迹：%s\n', sampleFile);
fprintf('仿真时长：%.3f s\n', refs.t(end));
fprintf('腿部被动刚度/阻尼：K=0, C=0\n');
fprintf('杆件每段数值正则质量：%.3e kg\n', ...
    simscapeData.approximations.strutSegmentRegularizationMass);

bandwidthCandidates = [0.5, 0.25, 0.125];
attempts = repmat(struct('bandwidthHz', [], 'design', [], 'report', [], ...
    'errorMessage', ''), 1, numel(bandwidthCandidates));
selectedIndex = [];

for candidateIndex = 1:numel(bandwidthCandidates)
    bandwidthHz = bandwidthCandidates(candidateIndex);
    attempts(candidateIndex).bandwidthHz = bandwidthHz;
    fprintf('\n--- 尝试闭环带宽 %.3f Hz ---\n', bandwidthHz);
    try
        configureSimscapeGravity(modelName, simscapeData.gravity, 'enabled', false);
        controller = initializeController('type', 'open-loop');
        design = designSimscapeLengthController(modelName, bandwidthHz);
        Kl = design.Kl;
        controller = simscapeData.controller;
        configureSimscapeGravity(modelName, simscapeData.gravity);
        simulationOutput = sim(modelName, ...
            'StopTime', num2str(refs.t(end), 16), ...
            'ReturnWorkspaceOutputs', 'on');
        simout = simulationOutput.get('simout');
        report = evaluateSimscapeLengthControl(simout, refs, model, design);
        attempts(candidateIndex).design = design;
        attempts(candidateIndex).report = report;
        printAttemptSummary(report, design);
        if report.passed
            selectedIndex = candidateIndex;
            break;
        end
    catch exception
        attempts(candidateIndex).errorMessage = getReport(exception, 'extended', ...
            'hyperlinks', 'off');
        fprintf('带宽 %.3f Hz 失败：%s\n', bandwidthHz, exception.message);
    end
end

if isempty(selectedIndex)
    completed = find(arrayfun(@(item) ~isempty(item.report), attempts), 1, 'last');
    if isempty(completed)
        save(fullfile(resultDir, ['failed_', tag, '.mat']), ...
            'model', 'scene', 'simscapeData', 'refs', 'references', 'attempts');
        error('run_02_simscape_length_control:NoSimulationCompleted', ...
            '所有候选带宽均未完成仿真。');
    end
    selectedIndex = completed;
end

selected = attempts(selectedIndex);
design = selected.design;
report = selected.report;
resultFile = fullfile(resultDir, ['result_', tag, '.mat']);
summaryFile = fullfile(resultDir, ['summary_', tag, '.txt']);
plotFile = fullfile(resultDir, ['tracking_', tag, '.png']);
save(resultFile, 'model', 'scene', 'simscapeData', 'refs', 'references', ...
    'attempts', 'selectedIndex', 'design', 'report');
writeSummary(summaryFile, resultFile, report, design, simscapeData);
plotTracking(plotFile, report);

fprintf('\n结果 MAT：%s\n', resultFile);
fprintf('摘要：%s\n', summaryFile);
fprintf('检查图：%s\n', plotFile);
fprintf('硬验收通过：%d\n', report.passed);

if ~report.passed
    error('run_02_simscape_length_control:AcceptanceFailed', ...
        '候选带宽均未通过硬验收，已保存最后一次完整仿真结果。');
end

function traj = trajectoryFromRefs(refs)
% trajectoryFromRefs - 将标准样例原始数组恢复为导出接口输入
traj = struct('t', refs.t, 'Q', refs.q, 'V', refs.qd, ...
    'Unode', refs.Fleg, 'L', refs.L);
end

function printAttemptSummary(report, design)
% printAttemptSummary - 输出单次闭环尝试摘要
fprintf('线性闭环稳定=%d，低频秩=%d\n', design.stable, design.lowFrequencyRank);
fprintf('腿长范围=[%.6f, %.6f] m，最大控制力=%.3f N\n', ...
    report.metrics.minAbsoluteLength, report.metrics.maxAbsoluteLength, ...
    report.metrics.maxAbsControlForce);
fprintf('长度误差 RMS 最大值=%.6e m，峰值最大值=%.6e m\n', ...
    max(report.metrics.lengthRms), max(report.metrics.lengthPeak));
fprintf('硬验收：finite=%d, length=%d, force=%d, passed=%d\n', ...
    report.acceptance.finitePassed, report.acceptance.lengthPassed, ...
    report.acceptance.forcePassed, report.passed);
end

function writeSummary(fileName, resultFile, report, design, simscapeData)
% writeSummary - 保存纯长度反馈闭环摘要
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'IHSID Simscape pure length feedback summary\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'bandwidthHz: %.12g\n', design.bandwidthHz);
fprintf(fid, 'stable: %d\n', design.stable);
fprintf(fid, 'lowFrequencyRank: %d\n', design.lowFrequencyRank);
fprintf(fid, 'passed: %d\n', report.passed);
fprintf(fid, 'finitePassed: %d\n', report.acceptance.finitePassed);
fprintf(fid, 'lengthPassed: %d\n', report.acceptance.lengthPassed);
fprintf(fid, 'forcePassed: %d\n', report.acceptance.forcePassed);
fprintf(fid, 'minAbsoluteLength: %.12e\n', report.metrics.minAbsoluteLength);
fprintf(fid, 'maxAbsoluteLength: %.12e\n', report.metrics.maxAbsoluteLength);
fprintf(fid, 'maxAbsControlForce: %.12e\n', report.metrics.maxAbsControlForce);
fprintf(fid, 'maxAbsOptimizedForce: %.12e\n', report.metrics.maxAbsOptimizedForce);
fprintf(fid, 'lengthRms: %s\n', mat2str(report.metrics.lengthRms, 12));
fprintf(fid, 'lengthPeak: %s\n', mat2str(report.metrics.lengthPeak, 12));
fprintf(fid, 'poseRms: %s\n', mat2str(report.metrics.poseRms, 12));
fprintf(fid, 'posePeak: %s\n', mat2str(report.metrics.posePeak, 12));
fprintf(fid, 'maxAbsLegSpeed: %.12e\n', report.metrics.maxAbsLegSpeed);
fprintf(fid, 'maxAbsLegAcceleration: %.12e\n', report.metrics.maxAbsLegAcceleration);
fprintf(fid, 'mappingTotalMassError: %.12e\n', simscapeData.mapping.totalMassError);
fprintf(fid, 'mappingComError: %.12e\n', simscapeData.mapping.comError);
fprintf(fid, 'mappingInertiaError: %.12e\n', simscapeData.mapping.inertiaError);
fprintf(fid, 'strutSegmentRegularizationMass: %.12e\n', ...
    simscapeData.approximations.strutSegmentRegularizationMass);
fprintf(fid, 'closedLoopPoles: %s\n', mat2str(design.closedLoopPoles, 12));
end

function plotTracking(fileName, report)
% plotTracking - 输出长度、位姿和控制力检查图
figureHandle = figure('Color', 'w', 'Position', [100, 100, 1200, 900]);
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
title('纯长度反馈控制力');

exportgraphics(figureHandle, fileName, 'Resolution', 180);
close(figureHandle);
end

function closeModelWithoutSaving(modelName)
% closeModelWithoutSaving - 丢弃重力等运行期配置并关闭通用模型
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
