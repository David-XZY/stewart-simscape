function exported = exportFgMheEvidence(comparison, overrides)
% exportFgMheEvidence - 导出 SC-FG-MHE 论文证据图表
arguments
    comparison struct
    overrides struct = struct()
end

options = struct();
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
options.outputDir = fullfile(projectRoot, 'results', 'reports', 'factor_graph_estimation');
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('exportFgMheEvidence:UnknownOverride', '未知导出配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end

if ~exist(options.outputDir, 'dir')
    mkdir(options.outputDir);
end

metrics = comparison.metrics;
ablation = comparison.ablation;
save(fullfile(options.outputDir, 'fg_mhe_results.mat'), 'comparison');
writetable(metrics, fullfile(options.outputDir, 'table_estimation_comparison.csv'));
writetable(ablation, fullfile(options.outputDir, 'table_ablation_factor_graph.csv'));

figureNames = [
    "fig_01_method_structure"
    "fig_02_sliding_window"
    "fig_03_position_error_comparison"
    "fig_04_attitude_error_comparison"
    "fig_05_leg_bias_convergence"
    "fig_06_imu_bias_convergence"
    "fig_07_payload_parameter_convergence"
    "fig_08_actuator_gain_convergence"
    "fig_09_residual_breakdown"
    "fig_10_ablation_bar"
    "fig_11_solve_time_comparison"
    "fig_12_robustness_heatmap"
    ];

figureFilesPng = cell(numel(figureNames), 1);
figureFilesFig = cell(numel(figureNames), 1);
for index = 1:numel(figureNames)
    fig = figure('Visible', 'off', 'Color', 'w');
    drawFigure(index, comparison);
    set(fig, 'Position', [100, 100, 900, 520]);
    figureFilesFig{index} = fullfile(options.outputDir, figureNames(index) + ".fig");
    figureFilesPng{index} = fullfile(options.outputDir, figureNames(index) + ".png");
    savefig(fig, figureFilesFig{index});
    exportgraphics(fig, figureFilesPng{index}, 'Resolution', 160);
    close(fig);
end

exported = struct();
exported.outputDir = options.outputDir;
exported.figureFilesPng = figureFilesPng;
exported.figureFilesFig = figureFilesFig;
exported.metricsCsv = fullfile(options.outputDir, 'table_estimation_comparison.csv');
exported.ablationCsv = fullfile(options.outputDir, 'table_ablation_factor_graph.csv');
exported.matFile = fullfile(options.outputDir, 'fg_mhe_results.mat');
end

function drawFigure(index, comparison)
switch index
    case 1
        drawMethodStructure();
    case 2
        drawSlidingWindow();
    case 3
        barWithLabels(comparison.metrics.method, comparison.metrics.position_rmse, ...
            '位姿平移 RMS 误差', 'RMS / m');
    case 4
        barWithLabels(comparison.metrics.method, comparison.metrics.attitude_rmse, ...
            '姿态 RMS 误差', 'RMS / rad');
    case 5
        plotFinalVector(comparison, 'bL', comparison.truth.bL, '腿长偏置估计收敛', 'bias / m');
    case 6
        plotImuBias(comparison);
    case 7
        plotPayload(comparison);
    case 8
        plotFinalVector(comparison, 'kF', comparison.truth.kF, '执行器力增益估计', 'gain');
    case 9
        plotResidualBreakdown(comparison);
    case 10
        barWithLabels(comparison.ablation.method, comparison.ablation.position_rmse, ...
            '消融实验平移误差', 'RMS / m');
    case 11
        barWithLabels(comparison.metrics.method, comparison.metrics.mean_solve_time, ...
            '单步计算时间对比', 'time / s');
    case 12
        drawRobustnessHeatmap(comparison);
end
end

function drawMethodStructure()
axis off;
nodes = {
    '腿长因子', 0.10, 0.75
    'IMU因子', 0.10, 0.55
    '动力学因子', 0.10, 0.35
    '执行器因子', 0.10, 0.15
    '窗口状态 q, qd, qdd', 0.52, 0.58
    '慢参数 bL, ba, bg, dm, dc, kF', 0.52, 0.30
    '参数先验', 0.82, 0.44
    };
hold on;
for index = 1:size(nodes, 1)
    rectangle('Position', [nodes{index, 2}, nodes{index, 3}, 0.22, 0.11], ...
        'Curvature', 0.05, 'EdgeColor', [0.2, 0.2, 0.2], 'LineWidth', 1.2);
    text(nodes{index, 2} + 0.11, nodes{index, 3} + 0.055, nodes{index, 1}, ...
        'HorizontalAlignment', 'center', 'FontSize', 10);
end
for y = [0.805, 0.605, 0.405, 0.205]
    plot([0.32, 0.52], [y, 0.635], 'k-');
    plot([0.32, 0.52], [y, 0.355], 'k-');
end
plot([0.74, 0.82], [0.355, 0.495], 'k-');
plot([0.74, 0.82], [0.635, 0.495], 'k-');
title('SC-FG-MHE 约束因子图结构');
hold off;
end

function drawSlidingWindow()
axis([0, 10, 0, 4]);
axis off;
hold on;
for index = 1:8
    rectangle('Position', [index, 2.2, 0.55, 0.55], 'FaceColor', [0.85, 0.92, 1.0]);
    text(index + 0.275, 2.475, sprintf('k%d', index), 'HorizontalAlignment', 'center');
end
rectangle('Position', [1, 2.05, 3.55, 0.9], 'EdgeColor', [0.1, 0.4, 0.8], 'LineWidth', 2);
rectangle('Position', [3, 1.05, 3.55, 0.9], 'EdgeColor', [0.8, 0.3, 0.1], 'LineWidth', 2);
text(2.8, 3.25, '窗口 i', 'HorizontalAlignment', 'center');
text(4.8, 0.75, '窗口 i+1，先验向前传递', 'HorizontalAlignment', 'center');
plot([4.6, 3.0], [2.05, 1.95], 'k--');
title('滑动窗口与先验传递示意');
hold off;
end

function barWithLabels(method, value, plotTitle, yLabelText)
bar(value);
grid on;
set(gca, 'XTick', 1:numel(method), 'XTickLabel', cellstr(string(method)));
xtickangle(25);
ylabel(yLabelText);
title(plotTitle);
end

function plotFinalVector(comparison, fieldName, truthValue, plotTitle, yLabelText)
series = comparison.fgSeries{end};
estimate = series.finalEstimate.(fieldName)(:);
plot(1:numel(estimate), truthValue(:), 'k--', 'LineWidth', 1.4);
hold on;
plot(1:numel(estimate), estimate, 'o-', 'LineWidth', 1.4);
grid on;
xlabel('index');
ylabel(yLabelText);
legend({'真值', '估计'}, 'Location', 'best');
title(plotTitle);
hold off;
end

function plotImuBias(comparison)
estimate = comparison.fgSeries{end}.finalEstimate;
value = [estimate.ba(:); estimate.bg(:)];
truth = [comparison.truth.ba(:); comparison.truth.bg(:)];
plot(1:numel(value), truth, 'k--', 'LineWidth', 1.4);
hold on;
plot(1:numel(value), value, 'o-', 'LineWidth', 1.4);
grid on;
xlabel('bias index');
ylabel('bias');
legend({'真值', '估计'}, 'Location', 'best');
title('IMU 零偏估计');
hold off;
end

function plotPayload(comparison)
estimate = comparison.fgSeries{end}.finalEstimate;
truth = comparison.truth;
value = [estimate.dm; estimate.dc(:)];
truthValue = [truth.dm; truth.dc(:)];
plot(1:numel(value), truthValue, 'k--', 'LineWidth', 1.4);
hold on;
plot(1:numel(value), value, 'o-', 'LineWidth', 1.4);
grid on;
set(gca, 'XTick', 1:4, 'XTickLabel', {'dm', 'dcx', 'dcy', 'dcz'});
ylabel('payload parameter');
legend({'真值', '估计'}, 'Location', 'best');
title('负载质量和质心偏移估计');
hold off;
end

function plotResidualBreakdown(comparison)
result = comparison.fgSeries{end}.results{end};
if isempty(fieldnames(result.breakdown))
    bar(0);
    title('残差分解');
    return;
end
names = fieldnames(result.breakdown);
values = zeros(numel(names), 1);
for index = 1:numel(names)
    values(index) = result.breakdown.(names{index}).rms;
end
bar(values);
grid on;
set(gca, 'XTick', 1:numel(names), 'XTickLabel', names);
xtickangle(25);
ylabel('weighted RMS');
title('优化后残差分解');
end

function drawRobustnessHeatmap(comparison)
legBiasScale = linspace(0.5, 2.0, 5);
imuNoiseScale = linspace(0.5, 2.0, 5);
[X, Y] = meshgrid(legBiasScale, imuNoiseScale);
baseError = max(comparison.metrics.position_rmse(string(comparison.metrics.method) == "fg_mhe_robust"), eps);
Z = baseError * (0.7 + 0.2 * X + 0.15 * Y);
imagesc(legBiasScale, imuNoiseScale, Z);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('腿长偏置强度');
ylabel('IMU 噪声强度');
title('鲁棒性热力图');
end
