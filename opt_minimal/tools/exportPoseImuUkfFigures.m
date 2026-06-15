function outputDir = exportPoseImuUkfFigures(resultFile, outputDir)
% exportPoseImuUkfFigures - 导出相对编码器与 IMU UKF 流程和结果图
arguments
    resultFile string = ""
    outputDir string = ""
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));

if strlength(resultFile) == 0
    candidates = dir(fullfile(optRoot, 'results', 'pwm_pose_force_comparison_*.mat'));
    [~, index] = max([candidates.datenum]);
    resultFile = string(fullfile(candidates(index).folder, candidates(index).name));
end
if strlength(outputDir) == 0
    outputDir = fullfile(optRoot, 'results', 'ukf_visuals_20260615');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

sample = load(resultFile, 'comparison', 'report');
makeFlowFigure(fullfile(outputDir, '01_relative_encoder_imu_ukf_flow.png'));
makeLatestResultFigure(sample.comparison, sample.report, ...
    fullfile(outputDir, '02_latest_ukf_closed_loop_result.png'));
makeNoiseBenchmarkFigure(sample.comparison.identified, ...
    fullfile(outputDir, '03_encoder_noise_benchmark.png'));
copyfile(strrep(resultFile, '.mat', '.png'), ...
    fullfile(outputDir, '04_original_pwm_closed_loop_result.png'));
fprintf('UKF 图已导出：%s\n', outputDir);
end

function makeFlowFigure(fileName)
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1550, 850]);
annotation(fig, 'textbox', [0.04 0.86 0.92 0.08], 'String', ...
    '回零锚定、相对编码器与 IMU 辅助 UKF', 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'FontSize', 20, 'FontWeight', 'bold');

boxes = {
    [0.04 0.58 0.18 0.18], {'回零静止 0.2 s'; '固定最短限位'; '一次性 anchorPose / anchorLength'}, [0.88 0.94 1.00]
    [0.29 0.64 0.18 0.12], {'六腿相对编码器'; 'ΔL = L(q)-L0'}, [0.91 0.97 0.94]
    [0.29 0.46 0.18 0.12], {'三轴姿态'; '仅姿态，不含 x/y/z'}, [0.91 0.97 0.94]
    [0.29 0.28 0.18 0.12], {'三轴加速度 + 角速度'; '原始比力 / 世界系加速度'}, [0.91 0.97 0.94]
    [0.55 0.55 0.18 0.18], {'18 状态 UKF'; '[q; qd; b_a; b_g]'; 'IMU 预测 + 几何校正'}, [1.00 0.95 0.84]
    [0.80 0.55 0.16 0.18], {'估计输出'; '位姿 q'; '速度 qd'; 'IMU 残余偏置'}, [0.95 0.91 0.98]
    [0.55 0.25 0.18 0.14], {'闭环控制与力估计'; '使用估计值'; '禁止读取位置真值'}, [1.00 0.91 0.91]
    };
for index = 1:size(boxes, 1)
    annotation(fig, 'textbox', boxes{index, 1}, 'String', boxes{index, 2}, ...
        'BackgroundColor', boxes{index, 3}, 'EdgeColor', [0.30 0.35 0.40], ...
        'LineWidth', 1.2, 'FontSize', 12, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FitBoxToText', 'off');
end
arrow(fig, [0.22 0.55], [0.67 0.67]);
arrow(fig, [0.47 0.55], [0.70 0.67]);
arrow(fig, [0.47 0.55], [0.52 0.62]);
arrow(fig, [0.47 0.55], [0.34 0.58]);
arrow(fig, [0.73 0.80], [0.64 0.64]);
arrow(fig, [0.64 0.64], [0.55 0.39]);
annotation(fig, 'textbox', [0.04 0.05 0.92 0.10], 'String', ...
    '绝对位置精度由回零锚点决定；运行中恢复相对运动与速度。旧的完整位姿测量 UKF 已停用。', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 12, 'FontAngle', 'italic', 'Color', [0.35 0.35 0.35]);
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function arrow(fig, x, y)
annotation(fig, 'arrow', x, y, 'Color', [0.25 0.35 0.45], ...
    'LineWidth', 1.8, 'HeadLength', 9, 'HeadWidth', 9);
end

function makeLatestResultFigure(comparison, report, fileName)
identified = comparison.identified;
t = identified.t;
poseError = identified.qFeedback - identified.qTrue;
velocityError = identified.qdFeedback - identified.qdTrue;
trackingError = identified.qTrue - identified.qReference;
plotIndices = unique(round(linspace(1, numel(t), min(numel(t), 2500))));
tPlot = t(plotIndices);

fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1500, 1000]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '相对编码器与 IMU UKF：最新闭环结果', 'FontSize', 17, 'FontWeight', 'bold');

ax = nexttile(layout);
plot(ax, tPlot, 1e3 * poseError(1:3, plotIndices).', 'LineWidth', 1.1);
yline(ax, 2, '--', '2 mm 实物门槛'); yline(ax, -2, '--');
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '估计误差 (mm)');
title(ax, 'UKF 平移估计误差'); legend(ax, 'x', 'y', 'z', 'Location', 'best');

ax = nexttile(layout);
plot(ax, tPlot, velocityError(1:3, plotIndices).', 'LineWidth', 1.1);
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '速度误差 (m/s)');
title(ax, 'UKF 平移速度误差'); legend(ax, 'v_x', 'v_y', 'v_z', 'Location', 'best');

ax = nexttile(layout);
plot(ax, tPlot, 1e3 * trackingError(1:3, plotIndices).', 'LineWidth', 1.1);
yline(ax, 1, ':', '严格旧门槛 ±1 mm'); yline(ax, -1, ':');
yline(ax, 5, '--', '实物闭环门槛 ±5 mm'); yline(ax, -5, '--');
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '跟踪误差 (mm)');
title(ax, '真实平台平移跟踪误差'); legend(ax, 'x', 'y', 'z', 'Location', 'best');

ax = nexttile(layout);
values = [1e3 * report.metrics.estimatorSettledTranslationAxisPeak(:), ...
    1e3 * report.metrics.identifiedTranslationAxisPeak(:)];
bar(ax, values); grid(ax, 'on');
xticklabels(ax, {'x', 'y', 'z'}); ylabel(ax, '峰值误差 (mm)');
legend(ax, 'UKF 稳态估计峰值', '真实闭环跟踪峰值', 'Location', 'northwest');
title(ax, sprintf('速度 RMS %.4f m/s；严格 1 mm 门槛：%d', ...
    report.metrics.estimatorVelocityRms, report.acceptance.strictTranslationPassed));
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function makeNoiseBenchmarkFigure(trajectory, fileName)
sampleCount = min(numel(trajectory.t), 500);
indices = 1:sampleCount;
shortTrajectory = struct('t', trajectory.t(indices), ...
    'qTrue', trajectory.qTrue(:, indices), 'qdTrue', trajectory.qdTrue(:, indices));
benchmark1 = benchmarkPwmPoseEstimators(shortTrajectory, struct('encoderNoiseStd', 1e-3));
benchmark05 = benchmarkPwmPoseEstimators(shortTrajectory, struct('encoderNoiseStd', 5e-4));
metrics1 = benchmark1.metrics;
metrics05 = benchmark05.metrics;

fig = figure('Color', 'w', 'Visible', 'off', 'Position', [80, 80, 1450, 620]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, '编码器噪声对 UKF 极限精度的影响', 'FontSize', 17, 'FontWeight', 'bold');

ax = nexttile(layout);
values = 1e3 * [metrics1.ukfSettledRelativeTranslationRms, metrics05.ukfSettledRelativeTranslationRms; ...
    metrics1.ukfSettledRelativeTranslationPeak, metrics05.ukfSettledRelativeTranslationPeak];
bar(ax, values); grid(ax, 'on');
xticklabels(ax, {'稳态相对平移 RMS', '稳态相对平移峰值'});
ylabel(ax, '误差 (mm)'); legend(ax, '1 mm RMS 编码器', '0.5 mm RMS 编码器');
yline(ax, 0.25, ':', 'RMS 目标 0.25 mm', 'HandleVisibility', 'off');
yline(ax, 1.0, '--', '峰值目标 1 mm', 'HandleVisibility', 'off');
title(ax, '位置恢复精度');

ax = nexttile(layout);
values = [metrics1.ukfVelocityRms, metrics05.ukfVelocityRms; ...
    metrics1.ukfLegSpeedRms, metrics05.ukfLegSpeedRms];
bar(ax, values); grid(ax, 'on');
xticklabels(ax, {'等效位姿速度 RMS', '腿速度 RMS'});
ylabel(ax, '速度误差 (m/s)'); legend(ax, '1 mm RMS 编码器', '0.5 mm RMS 编码器');
yline(ax, 0.005, '--', '速度目标 0.005 m/s', 'HandleVisibility', 'off');
title(ax, '速度链精度');
exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end
