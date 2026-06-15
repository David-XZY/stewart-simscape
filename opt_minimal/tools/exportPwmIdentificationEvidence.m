function evidence = exportPwmIdentificationEvidence(outputDir)
% exportPwmIdentificationEvidence - 导出 PWM 物理模型、辨识与控制全链路证据
arguments
    outputDir {mustBeTextScalar} = ""
end

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
resultDir = fullfile(optRoot, 'results');
addpath(fullfile(optRoot, 'actuator_identification'));
addpath(fullfile(optRoot, 'core'));

identifierFile = latestFile(resultDir, 'pwm_force_identifier_*.mat');
comparisonFile = latestFile(resultDir, 'pwm_pose_force_comparison_*.mat');
identifier = load(identifierFile);
control = load(comparisonFile);
if strlength(string(outputDir)) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(resultDir, ['pwm_evidence_', timestamp]);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

physical = buildPhysicalEvidence(identifier.teacher);
switching = validateAveragePwmActuatorAgainstSwitching(identifier.teacher, struct( ...
    'axisIndex', 1, 'duty', 0.7, 'duration', 0.15, 'legSpeed', 0.05));
switchingGrid = validateAveragePwmActuatorAcrossGrid(identifier.teacher);
identification = buildIdentificationEvidence(identifier.report, identifier.dataset);
controlMetrics = buildControlEvidence(control.comparison, control.report, identifier.teacher);
poseBenchmark = benchmarkPwmPoseEstimators(control.comparison.identified);

plotPhysical(fullfile(outputDir, '01_physical_actuator_characteristics.png'), physical, identifier.teacher);
plotDataset(fullfile(outputDir, '02_identification_dataset.png'), identifier.dataset);
plotIdentification(fullfile(outputDir, '03_gray_narx_validation.png'), ...
    identifier.report, identifier.dataset, identification);
plotSwitching(fullfile(outputDir, '04_average_switching_validation.png'), switching);
plotControl(fullfile(outputDir, '05_dual_line_full_tracking.png'), control.comparison);
plotPoseBenchmark(fullfile(outputDir, '06_pose_estimator_benchmark.png'), poseBenchmark);
writeSummary(fullfile(outputDir, 'summary.md'), identifierFile, comparisonFile, ...
    identifier, control, physical, switching, identification, controlMetrics, poseBenchmark);
writeMetricsCsv(fullfile(outputDir, 'metrics.csv'), identifier, control, switching, ...
    switchingGrid, identification, controlMetrics);

evidence = struct('outputDir', string(outputDir), 'identifierFile', string(identifierFile), ...
    'comparisonFile', string(comparisonFile), 'physical', physical, ...
    'switching', switching, 'switchingGrid', switchingGrid, 'identification', identification, ...
    'control', controlMetrics, 'poseBenchmark', poseBenchmark.metrics);
save(fullfile(outputDir, 'evidence.mat'), 'evidence');
fprintf('PWM 全链路证据已导出：%s\n', outputDir);
end

function fileName = latestFile(resultDir, pattern)
files = dir(fullfile(resultDir, pattern));
if isempty(files)
    error('exportPwmIdentificationEvidence:MissingResult', '未找到结果文件：%s', pattern);
end
[~, index] = max([files.datenum]);
fileName = fullfile(files(index).folder, files(index).name);
end

function physical = buildPhysicalEvidence(teacher)
commands = linspace(-teacher.pwmMax, teacher.pwmMax, 121);
speeds = [0, 0.15, 0.30];
force = zeros(numel(commands), numel(speeds));
for speedIndex = 1:numel(speeds)
    for commandIndex = 1:numel(commands)
        state = initializeHighFidelityPwmActuatorState(teacher);
        command = zeros(6, 1);
        command(1) = commands(commandIndex);
        legSpeed = zeros(6, 1);
        legSpeed(1) = speeds(speedIndex);
        for settleIndex = 1:100
            [state, output] = stepHighFidelityPwmActuator(state, command, legSpeed, teacher);
        end
        force(commandIndex, speedIndex) = output.force(1);
    end
end
physical = struct('commands', commands, 'speeds', speeds, 'force', force, ...
    'deadzoneFraction', teacher.deadzonePwm / teacher.pwmMax);
end

function identification = buildIdentificationEvidence(report, dataset)
testMask = dataset.split.test;
axisCount = size(dataset.trueForce, 2);
grayNrmse = zeros(axisCount, 1);
narxNrmse = zeros(axisCount, 1);
bias = zeros(axisCount, 1);
for axisIndex = 1:axisCount
    truth = dataset.trueForce(testMask, axisIndex);
    grayNrmse(axisIndex) = sqrt(mean((report.grayForce(testMask, axisIndex) - truth).^2)) / 4800;
    narxNrmse(axisIndex) = sqrt(mean((report.estimatedForce(testMask, axisIndex) - truth).^2)) / 4800;
    bias(axisIndex) = mean(report.estimatedForce(testMask, axisIndex) - truth) / 4800;
end
identification = struct('grayAxisNrmse', grayNrmse, 'narxAxisNrmse', narxNrmse, ...
    'axisBias', bias, 'improvementPercent', ...
    100 * (report.metrics.grayTestNrmse - report.metrics.narxTestNrmse) / report.metrics.grayTestNrmse);
end

function metrics = buildControlEvidence(comparison, report, teacher)
identified = comparison.identified;
oracle = comparison.oracle;
oraclePoseError = oracle.qTrue - oracle.qReference;
sameSampleForceEstimateError = identified.estimatedForce - identified.trueForce;
forceEstimateError = identified.estimatedForce(:, 2:end) - identified.trueForce(:, 1:end - 1);
poseFeedbackError = identified.qFeedback - identified.qTrue;
legSpeedError = identified.legSpeed - identified.trueLegSpeed;
metrics = report.metrics;
metrics.oracleTranslationPeak = max(abs(oraclePoseError(1:3, :)), [], 'all');
metrics.oracleRotationPeak = max(abs(oraclePoseError(4:6, :)), [], 'all');
metrics.forceEstimateNrmse = sqrt(mean(forceEstimateError.^2, 'all')) / (2 * max(teacher.forceLimit));
metrics.forceEstimateBias = max(abs(mean(forceEstimateError, 2))) / (2 * max(teacher.forceLimit));
metrics.forceEstimateRms = sqrt(mean(forceEstimateError.^2, 'all'));
metrics.forceEstimatePeak = max(abs(forceEstimateError), [], 'all');
metrics.sameSampleForceEstimateRms = sqrt(mean(sameSampleForceEstimateError.^2, 'all'));
metrics.poseFeedbackTranslationRms = sqrt(mean(poseFeedbackError(1:3, :).^2, 'all'));
metrics.poseFeedbackTranslationPeak = max(abs(poseFeedbackError(1:3, :)), [], 'all');
metrics.poseFeedbackRotationRms = sqrt(mean(poseFeedbackError(4:6, :).^2, 'all'));
metrics.poseFeedbackRotationPeak = max(abs(poseFeedbackError(4:6, :)), [], 'all');
metrics.encoderLegSpeedErrorRms = sqrt(mean(legSpeedError.^2, 'all'));
metrics.encoderLegSpeedErrorPeak = max(abs(legSpeedError), [], 'all');
metrics.pwmUtilization = metrics.identifiedMaxAbsPwm / teacher.pwmMax;
metrics.forceUtilization = metrics.identifiedMaxAbsTrueForce / max(teacher.forceLimit);
metrics.translationRmsRatioToOracle = ...
    comparison.metrics.identifiedPoseTranslationRms / comparison.metrics.oraclePoseTranslationRms;
end

function plotPhysical(fileName, physical, teacher)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1300, 760]);
tiledlayout(1, 2, 'TileSpacing', 'compact');
nexttile;
plot(physical.commands, physical.force, 'LineWidth', 1.6);
hold on; xline(teacher.deadzonePwm(1), 'k--'); xline(-teacher.deadzonePwm(1), 'k--');
yline(teacher.forceLimit(1), 'r:'); yline(-teacher.forceLimit(1), 'r:');
grid on; xlabel('PWM 命令'); ylabel('稳态输出力 (N)');
title('轴1 PWM-力静态特性及速度影响');
legend(compose('腿速 %.2f m/s', physical.speeds), 'Location', 'northwest');
nexttile;
bar(1:6, teacher.deadzonePwm);
grid on; xlabel('执行器轴号'); ylabel('死区 PWM');
title('六轴参数离散性');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function plotDataset(fileName, dataset)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1300, 980]);
tiledlayout(4, 1, 'TileSpacing', 'compact');
time = dataset.time;
trainEnd = time(find(dataset.split.train, 1, 'last'));
validationEnd = time(find(dataset.split.validation, 1, 'last'));
nexttile; plot(time, dataset.pwm(:, 1), 'LineWidth', 0.8);
grid on; ylabel('PWM'); title('轴1 辨识激励'); markSplit(trainEnd, validationEnd);
nexttile; plot(time, dataset.legSpeed(:, 1), 'LineWidth', 0.8);
grid on; ylabel('腿速 (m/s)'); title('编码器可观测运动状态'); markSplit(trainEnd, validationEnd);
nexttile; plot(time, dataset.trueForce(:, 1), 'LineWidth', 0.8);
grid on; ylabel('教师真力 (N)'); title('仅训练标签与评价使用的真力'); markSplit(trainEnd, validationEnd);
nexttile; histogram(dataset.pwm(:), 50);
grid on; xlabel('PWM'); ylabel('样本数'); title('六轴 PWM 覆盖分布');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function markSplit(trainEnd, validationEnd)
xline(trainEnd, 'k--', '训练/验证'); xline(validationEnd, 'k--', '验证/测试');
end

function plotIdentification(fileName, report, dataset, identification)
testIndices = find(dataset.split.test);
time = dataset.time(testIndices);
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1300, 980]);
tiledlayout(2, 2, 'TileSpacing', 'compact');
nexttile;
plot(time, dataset.trueForce(testIndices, 1), 'k', ...
    time, report.grayForce(testIndices, 1), '--', ...
    time, report.estimatedForce(testIndices, 1), 'LineWidth', 1);
grid on; xlabel('时间 (s)'); ylabel('力 (N)'); title('轴1 测试段自由运行预测');
legend('教师真力', '灰箱', '灰箱+NARX', 'Location', 'best');
nexttile;
plot(time, report.grayForce(testIndices, 1) - dataset.trueForce(testIndices, 1), '--', ...
    time, report.estimatedForce(testIndices, 1) - dataset.trueForce(testIndices, 1), 'LineWidth', 1);
grid on; xlabel('时间 (s)'); ylabel('误差 (N)'); title('轴1 测试段误差');
legend('灰箱误差', '灰箱+NARX误差', 'Location', 'best');
nexttile;
bar(1:6, 100 * [identification.grayAxisNrmse, identification.narxAxisNrmse]);
grid on; xlabel('执行器轴号'); ylabel('NRMSE (%)'); title('各轴测试 NRMSE');
legend('灰箱', '灰箱+NARX', 'Location', 'best');
nexttile;
truth = dataset.trueForce(testIndices, :);
estimate = report.estimatedForce(testIndices, :);
scatter(truth(:), estimate(:), 5, 'filled'); hold on;
limit = max(abs(truth), [], 'all'); plot([-limit, limit], [-limit, limit], 'r--');
axis equal; grid on; xlabel('教师真力 (N)'); ylabel('估计力 (N)');
title('六轴测试集真值-估计值一致性');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function plotSwitching(fileName, switching)
averageTime = linspace(0, 0.15, numel(switching.averageForce));
switchingTime = linspace(0, 0.15, numel(switching.switchingForce));
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1300, 760]);
tiledlayout(1, 2, 'TileSpacing', 'compact');
nexttile;
plot(switchingTime, switching.switchingForce, 'Color', [0.75, 0.75, 0.75]); hold on;
plot(averageTime, switching.averageForce, 'b-o', 'LineWidth', 1.3);
grid on; xlabel('时间 (s)'); ylabel('力 (N)');
title('5 kHz 开关级与 10 ms 平均值模型');
legend('开关级瞬时力', '平均值模型', 'Location', 'southeast');
nexttile;
bar([switching.metrics.meanSwitchingForce, switching.metrics.meanAverageForce]);
grid on; ylabel('尾段平均力 (N)'); xticklabels({'开关级', '平均值'});
title(sprintf('平均力相对误差 %.3f%%', 100 * switching.metrics.relativeMeanForceError));
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function plotControl(fileName, comparison)
identified = comparison.identified;
oracle = comparison.oracle;
t = identified.t;
identifiedError = identified.qTrue - identified.qReference;
oracleError = oracle.qTrue - oracle.qReference;
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1350, 1100]);
tiledlayout(3, 2, 'TileSpacing', 'compact');
nexttile;
plot(t, 1e3 * vecnorm(oracleError(1:3, :), 2, 1), '--', ...
    t, 1e3 * vecnorm(identifiedError(1:3, :), 2, 1), 'LineWidth', 1.2);
grid on; ylabel('平移误差范数 (mm)'); title('双线平移跟踪'); legend('真值基准线', '辨识反馈线');
nexttile;
plot(t, rad2deg(vecnorm(oracleError(4:6, :), 2, 1)), '--', ...
    t, rad2deg(vecnorm(identifiedError(4:6, :), 2, 1)), 'LineWidth', 1.2);
grid on; ylabel('旋转误差范数 (deg)'); title('双线旋转跟踪'); legend('真值基准线', '辨识反馈线');
nexttile;
plot(t, identified.targetForce(1, :), '--', t, identified.trueForce(1, :), 'LineWidth', 1);
grid on; ylabel('力 (N)'); title('辨识线轴1目标力与物理真力'); legend('目标力', '物理真力');
nexttile;
plot(t(2:end), identified.estimatedForce(1, 2:end) - identified.trueForce(1, 1:end - 1), ...
    'LineWidth', 1);
grid on; ylabel('力估计误差 (N)'); title('辨识线轴1在线力估计误差');
nexttile;
plot(t, identified.pwm.', 'LineWidth', 0.8);
grid on; xlabel('时间 (s)'); ylabel('PWM'); title('辨识线六轴 PWM 命令');
nexttile;
plot(t, (identified.legSpeed - identified.trueLegSpeed).', 'LineWidth', 0.8);
grid on; xlabel('时间 (s)'); ylabel('腿速误差 (m/s)'); title('融合位姿腿速与仿真真腿速差异');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function plotPoseBenchmark(fileName, benchmark)
time = benchmark.t;
kinematicError = benchmark.kinematic.pose - benchmark.truth.pose;
ukfError = benchmark.ukf.pose - benchmark.truth.pose;
metrics = benchmark.metrics;
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [100, 100, 1350, 900]);
tiledlayout(2, 2, 'TileSpacing', 'compact');
nexttile;
plot(time, 1e3 * vecnorm(kinematicError(1:3, :), 2, 1), '--', ...
    time, 1e3 * vecnorm(ukfError(1:3, :), 2, 1), 'LineWidth', 1);
grid on; ylabel('平移估计误差范数 (mm)'); title('真实位姿与融合估计差异');
legend('运动学重建', '偏置状态 UKF');
nexttile;
plot(time, rad2deg(vecnorm(kinematicError(4:6, :), 2, 1)), '--', ...
    time, rad2deg(vecnorm(ukfError(4:6, :), 2, 1)), 'LineWidth', 1);
grid on; ylabel('旋转估计误差范数 (deg)'); title('姿态估计误差');
legend('运动学重建', '偏置状态 UKF');
nexttile;
bar([metrics.kinematicTranslationRms, metrics.ukfTranslationRms; ...
    metrics.kinematicTranslationPeak, metrics.ukfTranslationPeak] * 1e3);
grid on; ylabel('误差 (mm)'); xticklabels({'平移 RMS', '平移峰值'});
legend('运动学重建', '偏置状态 UKF'); title('位姿误差指标');
nexttile;
bar([metrics.kinematicVelocityRms, metrics.ukfVelocityRms; ...
    metrics.kinematicLegSpeedRms, metrics.ukfLegSpeedRms]);
grid on; ylabel('误差速度 (m/s)'); xticklabels({'等效位姿速度 RMS', '腿速 RMS'});
legend('运动学重建', '偏置状态 UKF'); title('速度链误差指标');
exportgraphics(fig, fileName, 'Resolution', 180);
close(fig);
end

function writeSummary(fileName, identifierFile, comparisonFile, identifier, control, ...
        physical, switching, identification, controlMetrics, poseBenchmark)
dataset = identifier.dataset;
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# PWM 执行器辨识与控制结果证据\n\n');
fprintf(fid, '- 辨识结果：`%s`\n', identifierFile);
fprintf(fid, '- 控制结果：`%s`\n\n', comparisonFile);
fprintf(fid, '## 1. 高保真物理执行器\n\n');
fprintf(fid, '- 六轴死区 PWM：`%s`\n', mat2str(identifier.teacher.deadzonePwm.'));
fprintf(fid, '- 死区占满量程比例：`%.2f%% - %.2f%%`\n', ...
    100 * min(physical.deadzoneFraction), 100 * max(physical.deadzoneFraction));
fprintf(fid, '- 力限幅：`±%.0f N`\n\n', max(identifier.teacher.forceLimit));
fprintf(fid, '## 2. 辨识数据\n\n');
fprintf(fid, '- 样本数/时长/采样周期：`%d / %.2f s / %.3f s`\n', ...
    numel(dataset.time), dataset.time(end), dataset.sampleTime);
fprintf(fid, '- PWM 范围：`[%.1f, %.1f]`\n', min(dataset.pwm, [], 'all'), max(dataset.pwm, [], 'all'));
fprintf(fid, '- 腿速范围：`[%.4f, %.4f] m/s`\n\n', ...
    min(dataset.legSpeed, [], 'all'), max(dataset.legSpeed, [], 'all'));
fprintf(fid, '## 3. 灰箱加残差 NARX\n\n');
fprintf(fid, '- 灰箱测试 NRMSE：`%.4f%%`\n', 100 * identifier.report.metrics.grayTestNrmse);
fprintf(fid, '- NARX 测试 NRMSE：`%.4f%%`\n', 100 * identifier.report.metrics.narxTestNrmse);
fprintf(fid, '- NRMSE 相对改善：`%.2f%%`\n', identification.improvementPercent);
fprintf(fid, '- 测试偏差：`%.4f%%`\n\n', 100 * identifier.report.metrics.testBias);
fprintf(fid, '| 轴号 | 灰箱 NRMSE (%%) | 灰箱+NARX NRMSE (%%) | NARX 偏差 (%%) |\n');
fprintf(fid, '|---:|---:|---:|---:|\n');
for axisIndex = 1:6
    fprintf(fid, '| %d | %.4f | %.4f | %.4f |\n', axisIndex, ...
        100 * identification.grayAxisNrmse(axisIndex), ...
        100 * identification.narxAxisNrmse(axisIndex), ...
        100 * identification.axisBias(axisIndex));
end
fprintf(fid, '\n');
fprintf(fid, '- 灰箱参数优化目标：`%.6g -> %.6g`\n', ...
    identifier.identified.training.grayParameterFit.objectiveBefore, ...
    identifier.identified.training.grayParameterFit.objectiveAfter);
fprintf(fid, '- 独立闭环精炼样本数：`%d`\n\n', ...
    identifier.identified.training.closedLoopRefinement.addedSampleCount);
fprintf(fid, '## 4. 平均值模型与开关级校验\n\n');
fprintf(fid, '- 开关级尾段平均力：`%.3f N`\n', switching.metrics.meanSwitchingForce);
fprintf(fid, '- 平均值模型尾段平均力：`%.3f N`\n', switching.metrics.meanAverageForce);
fprintf(fid, '- 平均力相对误差：`%.4f%%`\n\n', 100 * switching.metrics.relativeMeanForceError);
fprintf(fid, '## 5. 完整轨迹双线控制\n\n');
fprintf(fid, '- 辨识线力跟踪 NRMSE：`%.4f%%`\n', 100 * control.report.metrics.identifiedForceTrackingNrmse);
fprintf(fid, '- 基准线力跟踪 NRMSE：`%.4f%%`\n', 100 * control.report.metrics.oracleForceTrackingNrmse);
fprintf(fid, '- 在线力估计 NRMSE：`%.4f%%`\n', 100 * controlMetrics.forceEstimateNrmse);
fprintf(fid, '- 在线力估计 RMS/峰值：`%.3f N / %.3f N`\n', ...
    controlMetrics.forceEstimateRms, controlMetrics.forceEstimatePeak);
fprintf(fid, '- 未对齐同拍力估计 RMS：`%.3f N`\n', controlMetrics.sameSampleForceEstimateRms);
fprintf(fid, '- 辨识线平移峰值：`%.3f mm`\n', 1e3 * controlMetrics.identifiedTranslationPeak);
fprintf(fid, '- 辨识线旋转峰值：`%.3f deg`\n', rad2deg(controlMetrics.identifiedRotationPeak));
fprintf(fid, '- 基准线平移/旋转峰值：`%.3f mm / %.3f deg`\n', ...
    1e3 * controlMetrics.oracleTranslationPeak, rad2deg(controlMetrics.oracleRotationPeak));
fprintf(fid, '- 辨识线/基准线平移 RMS 比：`%.3f`\n', controlMetrics.translationRmsRatioToOracle);
fprintf(fid, '- 融合位姿腿速误差 RMS/峰值：`%.5f / %.5f m/s`\n', ...
    controlMetrics.encoderLegSpeedErrorRms, controlMetrics.encoderLegSpeedErrorPeak);
fprintf(fid, '- 闭环 UKF 位姿平移 RMS/峰值：`%.4f / %.4f mm`\n', ...
    1e3 * controlMetrics.poseFeedbackTranslationRms, 1e3 * controlMetrics.poseFeedbackTranslationPeak);
fprintf(fid, '- PWM 利用率：`%.2f%%`\n', 100 * controlMetrics.pwmUtilization);
fprintf(fid, '- 真力限幅利用率：`%.2f%%`\n', 100 * controlMetrics.forceUtilization);
fprintf(fid, '- 真值隔离验收：`%d`\n', control.report.acceptance.truthIsolationPassed);
fprintf(fid, '- 完整轨迹验收：`%d`\n', control.report.passed);
fprintf(fid, '\n## 6. 位姿估计器独立基准\n\n');
fprintf(fid, '- 运动学重建平移 RMS/峰值：`%.4f / %.4f mm`\n', ...
    1e3 * poseBenchmark.metrics.kinematicTranslationRms, ...
    1e3 * poseBenchmark.metrics.kinematicTranslationPeak);
fprintf(fid, '- 偏置状态 UKF 平移 RMS/峰值：`%.4f / %.4f mm`\n', ...
    1e3 * poseBenchmark.metrics.ukfTranslationRms, 1e3 * poseBenchmark.metrics.ukfTranslationPeak);
fprintf(fid, '- 运动学重建/UKF 腿速 RMS：`%.5f / %.5f m/s`\n', ...
    poseBenchmark.metrics.kinematicLegSpeedRms, poseBenchmark.metrics.ukfLegSpeedRms);
end

function writeMetricsCsv(fileName, identifier, control, switching, switchingGrid, identification, controlMetrics)
names = ["gray_test_nrmse"; "narx_test_nrmse"; "narx_test_bias"; ...
    "narx_improvement_percent"; "switching_relative_mean_force_error"; ...
    "switching_grid_max_relative_error"; "switching_grid_max_normalized_error"; ...
    "identified_force_tracking_nrmse"; "oracle_force_tracking_nrmse"; ...
    "online_force_estimate_nrmse"; "online_force_estimate_rms_n"; ...
    "online_force_estimate_peak_n"; "same_sample_force_estimate_rms_n"; ...
    "pose_feedback_translation_rms_m"; "pose_feedback_translation_peak_m"; ...
    "identified_translation_peak_m"; "identified_rotation_peak_rad"; ...
    "pwm_utilization"; "force_utilization"];
values = [identifier.report.metrics.grayTestNrmse; identifier.report.metrics.narxTestNrmse; ...
    identifier.report.metrics.testBias; identification.improvementPercent; ...
    switching.metrics.relativeMeanForceError; switchingGrid.metrics.maxRelativeForceError; ...
    switchingGrid.metrics.maxNormalizedForceError; control.report.metrics.identifiedForceTrackingNrmse; ...
    control.report.metrics.oracleForceTrackingNrmse; controlMetrics.forceEstimateNrmse; ...
    controlMetrics.forceEstimateRms; controlMetrics.forceEstimatePeak; ...
    controlMetrics.sameSampleForceEstimateRms; controlMetrics.poseFeedbackTranslationRms; ...
    controlMetrics.poseFeedbackTranslationPeak; ...
    controlMetrics.identifiedTranslationPeak; controlMetrics.identifiedRotationPeak; ...
    controlMetrics.pwmUtilization; controlMetrics.forceUtilization];
writetable(table(names, values), fileName);
end
