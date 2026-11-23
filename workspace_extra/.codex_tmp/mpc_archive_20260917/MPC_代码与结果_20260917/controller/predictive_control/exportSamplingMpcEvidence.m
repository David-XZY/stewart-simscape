function exported = exportSamplingMpcEvidence(comparison, outputDir, config)
% exportSamplingMpcEvidence - 导出 SC-MPC 对比表格、数据和图像证据
arguments
    comparison struct
    outputDir {mustBeTextScalar}
    config struct
end

outputDir = char(outputDir);
if ~isfolder(outputDir)
    mkdir(outputDir);
end
writetable(comparison.summaryTable, fullfile(outputDir, 'table_sampling_mpc_comparison.csv'));
writetable(comparison.diagnosticTable, fullfile(outputDir, 'table_sampling_mpc_diagnostics.csv'));
save(fullfile(outputDir, 'sampling_mpc_comparison_data.mat'), 'comparison', 'config');

figures = {
    makeStructureFigure()
    makeErrorFigure(comparison.runs)
    makeForceFigure(comparison.runs)
    makeSafetyFigure(comparison.runs)
    makeSummaryFigure(comparison.summaryTable)
    makeDiagnosticFigure(comparison.diagnosticTable)
    };
names = [
    "fig_01_sampling_mpc_structure"
    "fig_02_tracking_error_comparison"
    "fig_03_force_and_rate_comparison"
    "fig_04_safety_margin_comparison"
    "fig_05_summary_metrics"
    "fig_06_method_diagnostics"
    ];
figureFiles = strings(numel(figures), 1);
for index = 1:numel(figures)
    figureFiles(index) = saveFigure(figures{index}, outputDir, names(index));
end

exported = struct();
exported.outputDir = string(outputDir);
exported.summaryTableFile = string(fullfile(outputDir, 'table_sampling_mpc_comparison.csv'));
exported.diagnosticTableFile = string(fullfile(outputDir, 'table_sampling_mpc_diagnostics.csv'));
exported.dataFile = string(fullfile(outputDir, 'sampling_mpc_comparison_data.mat'));
exported.figureFiles = figureFiles;
end

function fig = makeStructureFigure()
fig = baseFigure('SC-MPC 控制结构');
axis off;
labels = ["参考轨迹", "名义力", "候选力序列", "预测模型", "约束代价", "首拍执行"];
x = [0.08, 0.24, 0.42, 0.60, 0.76, 0.90];
y = 0.55 * ones(size(x));
for index = 1:numel(labels)
    annotation(fig, 'rectangle', [x(index)-0.06, y(index)-0.08, 0.12, 0.16], ...
        'FaceColor', [0.92, 0.96, 1.00], 'EdgeColor', [0.12, 0.32, 0.55]);
    annotation(fig, 'textbox', [x(index)-0.055, y(index)-0.035, 0.11, 0.07], ...
        'String', labels(index), 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'FontSize', 11);
    if index < numel(labels)
        annotation(fig, 'arrow', [x(index)+0.06, x(index+1)-0.06], [y(index), y(index+1)]);
    end
end
end

function fig = makeErrorFigure(runs)
fig = baseFigure('跟踪误差对比');
layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout); hold(ax, 'on');
for index = 1:numel(runs)
    plot(ax, runs(index).time, 1e3 * vecnorm(runs(index).poseError(1:3, :), 2, 1), ...
        'LineWidth', 1.2);
end
grid(ax, 'on'); ylabel(ax, '平移误差 (mm)'); legend(ax, string({runs.method}), 'Location', 'best');
ax = nexttile(layout); hold(ax, 'on');
for index = 1:numel(runs)
    plot(ax, runs(index).time, rad2deg(vecnorm(runs(index).poseError(4:6, :), 2, 1)), ...
        'LineWidth', 1.2);
end
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '旋转误差 (deg)');
end

function fig = makeForceFigure(runs)
fig = baseFigure('控制力和力变化率对比');
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout); hold(ax, 'on');
for index = 1:numel(runs)
    plot(ax, runs(index).time, vecnorm(runs(index).force, 2, 1), 'LineWidth', 1.2);
end
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '六支链力范数 (N)');
legend(ax, string({runs.method}), 'Location', 'best');
ax = nexttile(layout); hold(ax, 'on');
for index = 1:numel(runs)
    forceRate = [zeros(6, 1), diff(runs(index).force, 1, 2) / median(diff(runs(index).time))];
    plot(ax, runs(index).time, vecnorm(forceRate, 2, 1), 'LineWidth', 1.2);
end
grid(ax, 'on'); xlabel(ax, '时间 (s)'); ylabel(ax, '力变化率范数 (N/s)');
end

function fig = makeSafetyFigure(runs)
fig = baseFigure('安全裕度对比');
hold on;
for index = 1:numel(runs)
    plot(runs(index).time, runs(index).minSafetyMarginSeries, 'LineWidth', 1.2);
end
yline(0, 'k--', '约束边界');
grid on; xlabel('时间 (s)'); ylabel('最小安全裕度');
legend(string({runs.method}), 'Location', 'best');
end

function fig = makeSummaryFigure(summaryTable)
fig = baseFigure('总体指标对比');
layout = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
barWithLabels(nexttile(layout), 1e3 * summaryTable.position_rmse, summaryTable.method, ...
    '平移 RMSE', 'mm');
barWithLabels(nexttile(layout), summaryTable.max_force, summaryTable.method, ...
    '力峰值', 'N');
barWithLabels(nexttile(layout), summaryTable.control_energy, summaryTable.method, ...
    '控制能量', 'N^2 s');
end

function fig = makeDiagnosticFigure(diagnosticTable)
fig = baseFigure('方法机制对比');
values = [diagnosticTable.uses_online_optimization, ...
    diagnosticTable.explicit_force_constraints, diagnosticTable.explicit_horizon_prediction];
imagesc(values);
colormap([0.92, 0.92, 0.92; 0.18, 0.45, 0.70]);
colorbar('Ticks', [0.25, 0.75], 'TickLabels', {'否', '是'});
xticks(1:3); xticklabels({'在线优化', '显式力约束', '显式时域预测'});
yticks(1:height(diagnosticTable)); yticklabels(diagnosticTable.method);
title('控制机制差异');
end

function barWithLabels(ax, values, labels, titleText, yLabel)
bar(ax, values);
grid(ax, 'on'); xticks(ax, 1:numel(labels)); xticklabels(ax, labels);
xtickangle(ax, 15); ylabel(ax, yLabel); title(ax, titleText);
for index = 1:numel(values)
    text(ax, index, values(index), sprintf(' %.3g', values(index)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
end

function fig = baseFigure(titleText)
fig = figure('Color', 'w', 'Visible', 'off', 'Units', 'pixels', ...
    'Position', [80, 80, 1400, 780]);
sgtitle(fig, titleText, 'FontSize', 15, 'FontWeight', 'bold');
end

function pngFile = saveFigure(fig, outputDir, baseName)
figFile = fullfile(outputDir, char(baseName + ".fig"));
pngFile = fullfile(outputDir, char(baseName + ".png"));
savefig(fig, figFile);
print(fig, pngFile, '-dpng', '-r200');
close(fig);
pngFile = string(pngFile);
end
