function generate_simscape_tracking_supplement()
% generate_simscape_tracking_supplement - 生成中期报告中的跟踪控制结构图与结果图

resultFile = 'D:\NPU\Matlab_code\stewart-simscape\opt_minimal\results\result_simscape_length_control_20260609_102452.mat';
outputDir = 'D:\NPU\并联机器人\中期\Simscape跟踪控制补充';

loaded = load(resultFile, 'report', 'refs', 'design', 'useForceFeedforward');
assert(isfield(loaded, 'report') && loaded.report.passed, '指定 Simscape 结果未通过验收。');
assert(abs(loaded.design.bandwidthHz - 10) < 1e-12, '指定结果并非 10 Hz 控制器。');
assert(loaded.useForceFeedforward, '指定结果未启用驱动力前馈。');

report = loaded.report;
refs = loaded.refs;
assert(report.metrics.maxTranslationPeak <= 10e-3, '平移误差超出 10 mm。');
assert(report.metrics.maxRotationPeak <= deg2rad(1), '姿态误差超出 1°。');
assert(report.metrics.maxLengthTrackingPeak <= 5e-3, '腿长误差超出 5 mm。');
assert(report.metrics.maxAbsControlForce <= 2000, '控制力超出 ±2000 N。');

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

writeArchitectureFigure(outputDir);
writeTrackingFigure(outputDir, report, refs);
writeMetrics(outputDir, resultFile, report, loaded.design);
fprintf('Simscape 跟踪控制补充图已生成：%s\n', outputDir);
end

function writeArchitectureFigure(outputDir)
fig = makeFigure('图7 跟踪控制结构', [70 70 1600 850]);
blue = [0.10 0.35 0.72];
blueLight = [0.90 0.95 1.00];
orange = [0.90 0.42 0.12];
gray = [0.35 0.38 0.42];
grayLight = [0.96 0.97 0.98];
greenLight = [0.91 0.97 0.93];

diagramBox(fig, [0.035 0.61 0.16 0.20], 'IHSID轨迹优化', ...
    {'参考位姿 q_{ref}', '参考腿长 L_{ref}', '参考驱动力 F_{FF}'}, blueLight, blue);
diagramBox(fig, [0.245 0.63 0.17 0.16], '参考信号', ...
    {'q_{ref}，L_{ref}，F_{FF}', '完整轨迹时长：7.5 s'}, grayLight, gray);
sumCircle(fig, [0.590 0.680 0.040 0.075], '+', '+', blue);
diagramBox(fig, [0.675 0.61 0.17 0.20], 'Simscape平台模型', ...
    {'六自由度并联机构', '力驱动执行器'}, greenLight, [0.18 0.48 0.29]);
diagramBox(fig, [0.875 0.61 0.105 0.20], '测量输出', ...
    {'实际腿长 L', '实际位姿 q'}, grayLight, gray);

sumCircle(fig, [0.325 0.315 0.040 0.075], '+', '−', orange);
diagramBox(fig, [0.415 0.275 0.17 0.16], '腿长反馈控制器', ...
    {'e_L = L_{ref} − L', '反馈带宽：10 Hz'}, [1.00 0.95 0.89], orange);
diagramBox(fig, [0.675 0.245 0.17 0.14], '跟踪效果评估', ...
    {'位姿误差与约束验收', '不参与闭环控制'}, grayLight, gray);

arrow(fig, [0.195 0.245], [0.71 0.71], '-', blue);
arrow(fig, [0.415 0.590], [0.71 0.71], '-', blue);
label(fig, [0.465 0.745 0.10 0.04], 'F_{FF}', blue, 'center');
arrow(fig, [0.630 0.675], [0.71 0.71], '-', blue);
label(fig, [0.625 0.755 0.06 0.04], 'F = F_{FF}+F_{FB}', blue, 'center');
arrow(fig, [0.845 0.875], [0.71 0.71], '-', [0.18 0.48 0.29]);

arrow(fig, [0.330 0.330], [0.63 0.390], '-', orange);
label(fig, [0.265 0.485 0.10 0.04], 'L_{ref}', orange, 'center');
arrow(fig, [0.365 0.415], [0.352 0.352], '-', orange);
arrow(fig, [0.585 0.610], [0.352 0.680], '-', orange);
label(fig, [0.585 0.455 0.08 0.05], 'F_{FB}', orange, 'center');

lineArrow(fig, [0.928 0.928 0.345 0.345], [0.61 0.19 0.19 0.315], '-', orange);
label(fig, [0.825 0.145 0.10 0.04], 'L（闭环反馈）', orange, 'center');

lineArrow(fig, [0.930 0.930 0.845], [0.61 0.455 0.315], '--', gray);
label(fig, [0.855 0.425 0.12 0.04], 'q（仅用于评估）', gray, 'center');
lineArrow(fig, [0.300 0.300 0.675], [0.63 0.505 0.315], '--', gray);
label(fig, [0.470 0.475 0.15 0.04], 'q_{ref}（评估参考）', gray, 'center');

annotation(fig, 'textbox', [0.09 0.885 0.82 0.065], ...
    'String', '优化轨迹驱动力前馈与腿长误差反馈的 Simscape 跟踪控制结构', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold', ...
    'Color', [0.12 0.15 0.20]);
annotation(fig, 'textbox', [0.16 0.055 0.68 0.07], ...
    'String', '实线表示控制信号链路；虚线表示仅用于结果分析的位姿测量链路。', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 11, 'Color', gray);

saveFigure(fig, outputDir, '图7_Simscape跟踪控制结构');
close(fig);
end

function writeTrackingFigure(outputDir, report, refs)
fig = makeFigure('图8 跟踪控制仿真结果', [60 60 1500 1050]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
blue = [0.10 0.35 0.72];
orange = [0.90 0.42 0.12];
red = [0.75 0.12 0.12];
gray = [0.35 0.38 0.42];
axisColors = [0.10 0.35 0.72; 0.90 0.42 0.12; 0.20 0.60 0.35];
forceColors = lines(6);
stageTime = refs.tStageBoundary;

referencePosition = 1000 * report.referenceRelativePose(:, 1:3);
actualPosition = 1000 * report.actualRelativePose(:, 1:3);
[~, stagePoseIndex] = min(abs(report.poseTime - stageTime));

ax = nexttile(layout);
plot3(ax, referencePosition(:, 1), referencePosition(:, 2), referencePosition(:, 3), ...
    '--', 'Color', orange, 'LineWidth', 2.3);
hold(ax, 'on');
plot3(ax, actualPosition(:, 1), actualPosition(:, 2), actualPosition(:, 3), ...
    '-', 'Color', blue, 'LineWidth', 1.4);
scatter3(ax, referencePosition(1, 1), referencePosition(1, 2), referencePosition(1, 3), ...
    42, [0.20 0.60 0.35], 'filled', 'DisplayName', '起点');
scatter3(ax, referencePosition(stagePoseIndex, 1), referencePosition(stagePoseIndex, 2), ...
    referencePosition(stagePoseIndex, 3), 46, gray, 'filled', 'diamond', 'DisplayName', '阶段切换');
scatter3(ax, referencePosition(end, 1), referencePosition(end, 2), referencePosition(end, 3), ...
    42, red, 'filled', 'DisplayName', '终点');
grid(ax, 'on'); axis(ax, 'equal'); view(ax, 38, 24);
xlabel(ax, '\Delta x (mm)'); ylabel(ax, '\Delta y (mm)'); zlabel(ax, '\Delta z (mm)');
title(ax, '(a) 平台参考与实际三维运动轨迹');
legend(ax, {'参考轨迹', '实际轨迹', '起点', '阶段切换', '终点'}, ...
    'Location', 'best', 'FontSize', 9);

ax = nexttile(layout);
positionError = 1000 * report.poseError(:, 1:3);
plotThreeSignals(ax, report.poseTime, positionError, axisColors, {'x', 'y', 'z'});
addThresholds(ax, 10, stageTime, red, gray);
xlabel(ax, '时间 (s)'); ylabel(ax, '平移误差 (mm)');
title(ax, '(b) 三轴平移跟踪误差');
ylim(ax, [-11.5 11.5]);
text(ax, 0.72, 0.92, sprintf('峰值 %.3f mm', 1000 * report.metrics.maxTranslationPeak), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'Color', blue, 'BackgroundColor', 'white');

ax = nexttile(layout);
rotationError = rad2deg(report.poseError(:, 4:6));
plotThreeSignals(ax, report.poseTime, rotationError, axisColors, {'滚转', '俯仰', '偏航'});
addThresholds(ax, 1, stageTime, red, gray);
xlabel(ax, '时间 (s)'); ylabel(ax, '姿态误差 (°)');
title(ax, '(c) 三轴姿态跟踪误差');
ylim(ax, [-1.15 1.15]);
text(ax, 0.72, 0.92, sprintf('峰值 %.3f°', rad2deg(report.metrics.maxRotationPeak)), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'Color', blue, 'BackgroundColor', 'white');

ax = nexttile(layout);
hold(ax, 'on');
for legIndex = 1:6
    plot(ax, report.forceTime, report.controlForce(:, legIndex), ...
        'Color', forceColors(legIndex, :), 'LineWidth', 1.1, ...
        'DisplayName', sprintf('支腿%d', legIndex));
end
yline(ax, 2000, '--', 'Color', red, 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(ax, -2000, '--', 'Color', red, 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(ax, stageTime, ':', 'Color', gray, 'LineWidth', 1.1, 'HandleVisibility', 'off');
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '时间 (s)'); ylabel(ax, '总驱动力 (N)');
title(ax, '(d) 六条支腿总驱动力');
ylim(ax, [-2200 2200]);
legend(ax, 'Location', 'southoutside', 'NumColumns', 3, 'FontSize', 9);
text(ax, 0.72, 0.92, sprintf('峰值 %.0f N', report.metrics.maxAbsControlForce), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'Color', blue, 'BackgroundColor', 'white');

title(layout, 'Simscape参考轨迹跟踪控制仿真结果', 'FontSize', 15, 'FontWeight', 'bold');
saveFigure(fig, outputDir, '图8_Simscape跟踪控制仿真结果');
close(fig);
end

function writeMetrics(outputDir, resultFile, report, design)
fileName = fullfile(outputDir, '跟踪结果指标核对.txt');
fid = fopen(fileName, 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '数据来源：%s\n', resultFile);
fprintf(fid, '完整验收通过：%d\n', report.passed);
fprintf(fid, '反馈带宽：%.3f Hz\n', design.bandwidthHz);
fprintf(fid, '最大腿长跟踪误差：%.3f mm\n', 1000 * report.metrics.maxLengthTrackingPeak);
fprintf(fid, '最大平移跟踪误差：%.3f mm\n', 1000 * report.metrics.maxTranslationPeak);
fprintf(fid, '最大姿态跟踪误差：%.3f°\n', rad2deg(report.metrics.maxRotationPeak));
fprintf(fid, '最大总驱动力：%.3f N\n', report.metrics.maxAbsControlForce);
fprintf(fid, '实际腿长范围：[%.3f, %.3f] m\n', ...
    report.metrics.minAbsoluteLength, report.metrics.maxAbsoluteLength);
end

function fig = makeFigure(name, position)
fig = figure('Name', name, 'Color', 'w', 'Visible', 'off', 'Position', position);
set(fig, 'DefaultAxesFontName', 'Microsoft YaHei', 'DefaultTextFontName', 'Microsoft YaHei', ...
    'DefaultAxesFontSize', 11, 'DefaultTextFontSize', 11);
end

function saveFigure(fig, outputDir, baseName)
axesHandles = findall(fig, 'Type', 'axes');
for axesIndex = 1:numel(axesHandles)
    axesHandles(axesIndex).Toolbar.Visible = 'off';
end
exportgraphics(fig, fullfile(outputDir, [baseName '.png']), ...
    'Resolution', 300, 'BackgroundColor', 'white');
exportgraphics(fig, fullfile(outputDir, [baseName '.pdf']), ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
end

function plotThreeSignals(ax, time, values, colors, names)
hold(ax, 'on');
for signalIndex = 1:3
    plot(ax, time, values(:, signalIndex), 'Color', colors(signalIndex, :), ...
        'LineWidth', 1.2, 'DisplayName', names{signalIndex});
end
grid(ax, 'on'); box(ax, 'on');
legend(ax, 'Location', 'best', 'FontSize', 9);
end

function addThresholds(ax, threshold, stageTime, red, gray)
yline(ax, threshold, '--', 'Color', red, 'LineWidth', 1.1, 'HandleVisibility', 'off');
yline(ax, -threshold, '--', 'Color', red, 'LineWidth', 1.1, 'HandleVisibility', 'off');
xline(ax, stageTime, ':', 'Color', gray, 'LineWidth', 1.1, 'HandleVisibility', 'off');
end

function diagramBox(fig, position, titleText, bodyLines, fillColor, edgeColor)
annotation(fig, 'rectangle', position, 'FaceColor', fillColor, ...
    'EdgeColor', edgeColor, 'LineWidth', 1.5);
annotation(fig, 'textbox', [position(1), position(2) + 0.61 * position(4), position(3), 0.31 * position(4)], ...
    'String', titleText, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'FontName', 'Microsoft YaHei', ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', edgeColor);
annotation(fig, 'textbox', [position(1) + 0.01, position(2) + 0.05 * position(4), position(3) - 0.02, 0.54 * position(4)], ...
    'String', strjoin(bodyLines, newline), 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 10.5, 'Color', [0.16 0.18 0.22]);
end

function sumCircle(fig, position, upperSign, lowerSign, edgeColor)
annotation(fig, 'ellipse', position, 'FaceColor', 'white', 'EdgeColor', edgeColor, 'LineWidth', 1.6);
annotation(fig, 'textbox', position, 'String', [upperSign newline lowerSign], ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 11, 'FontWeight', 'bold', 'Color', edgeColor);
end

function arrow(fig, x, y, lineStyle, color)
annotation(fig, 'arrow', x, y, 'LineStyle', lineStyle, 'Color', color, ...
    'LineWidth', 1.5, 'HeadLength', 8, 'HeadWidth', 8);
end

function lineArrow(fig, x, y, lineStyle, color)
for index = 1:(numel(x) - 2)
    annotation(fig, 'line', x(index:index + 1), y(index:index + 1), ...
        'LineStyle', lineStyle, 'Color', color, 'LineWidth', 1.3);
end
annotation(fig, 'arrow', x(end - 1:end), y(end - 1:end), ...
    'LineStyle', lineStyle, 'Color', color, 'LineWidth', 1.3, ...
    'HeadLength', 8, 'HeadWidth', 8);
end

function label(fig, position, textValue, color, alignment)
annotation(fig, 'textbox', position, 'String', textValue, 'EdgeColor', 'none', ...
    'HorizontalAlignment', alignment, 'VerticalAlignment', 'middle', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 10, 'Color', color);
end
