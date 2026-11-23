function paths = export_fresh_baseline_thesis_figures(resultFile, outputDir)
% Re-export the locked baseline result with thesis-scale typography.
arguments
    resultFile {mustBeFile}
    outputDir {mustBeTextScalar}
end

loaded = load(resultFile, 'report');
report = loaded.report;
outputDir = char(outputDir);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

colors = lines(6);
paths = strings(3, 1);
paths(1) = fullfile(outputDir, 'fresh_baseline_tracking.png');
paths(2) = fullfile(outputDir, 'fresh_baseline_leg_motion.png');
paths(3) = fullfile(outputDir, 'fresh_baseline_force.png');

figureHandle = thesisFigure(6.8, 5.3);
layout = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(report.poseTime, report.poseError(:, 1:3)*1e3, 'LineWidth', 1.15);
grid on;
ylim([-0.85, 0.85]);
ylabel('平移误差/mm');
title('位姿跟踪误差', 'FontWeight', 'normal');
legend({'x', 'y', 'z'}, 'Location', 'northwest', 'NumColumns', 3, ...
    'FontSize', 10.5);
formatAxes(gca);
nexttile;
plot(report.poseTime, rad2deg(report.poseError(:, 4:6)), 'LineWidth', 1.15);
grid on;
ylim([-0.05, 0.16]);
ylabel('姿态误差/(°)');
legend({'\alpha', '\beta', '\gamma'}, 'Location', 'northwest', ...
    'NumColumns', 3, 'Interpreter', 'tex', 'FontSize', 10.5);
formatAxes(gca);
nexttile;
plot(report.poseTime, report.lengthError*1e3, 'LineWidth', 1.1);
grid on;
ylim([-0.6, 2.0]);
xlabel('时间/s');
ylabel('腿长误差/mm');
formatAxes(gca);
exportgraphics(figureHandle, paths(1), 'Resolution', 240);
close(figureHandle);

figureHandle = thesisFigure(6.8, 4.6);
layout = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); %#ok<NASGU>
nexttile;
plot(report.derivativeTime, report.legSpeed, 'LineWidth', 1.15);
grid on;
yline(0.60, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
yline(-0.60, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
ylim([-0.65, 0.65]);
ylabel('腿速度/(m\cdot s^{-1})', 'Interpreter', 'tex');
title('六支链速度与中期约束边界', 'FontWeight', 'normal');
formatAxes(gca);
nexttile;
plot(report.derivativeTime, report.legAcceleration, 'LineWidth', 1.15);
grid on;
yline(1.20, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
yline(-1.20, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
ylim([-1.30, 1.30]);
xlabel('时间/s');
ylabel('腿加速度/(m\cdot s^{-2})', 'Interpreter', 'tex');
title('六支链加速度与中期约束边界', 'FontWeight', 'normal');
formatAxes(gca);
exportgraphics(figureHandle, paths(2), 'Resolution', 240);
close(figureHandle);

figureHandle = thesisFigure(6.8, 4.6);
layout = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); %#ok<NASGU>
nexttile;
plot(report.forceTime, report.controlForce, 'LineWidth', 1.05);
grid on;
yline(2000, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
yline(-2000, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
ylabel('指令力/N');
title('六支链总驱动力', 'FontWeight', 'normal');
formatAxes(gca);
nexttile;
forceError = report.controlForce - report.actualForce;
plot(report.forceTime, forceError, 'LineWidth', 0.9);
grid on;
xlabel('时间/s');
ylabel('指令-实际力/N');
title('非理想执行器力跟踪误差', 'FontWeight', 'normal');
formatAxes(gca);
exportgraphics(figureHandle, paths(3), 'Resolution', 240);
close(figureHandle);
end

function figureHandle = thesisFigure(widthIn, heightIn)
figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Units', 'inches', 'Position', [1, 1, widthIn, heightIn]);
end

function formatAxes(axisHandle)
set(axisHandle, 'FontName', 'Microsoft YaHei', 'FontSize', 12, ...
    'LineWidth', 0.8, 'Box', 'on', 'ColorOrder', lines(6));
end
