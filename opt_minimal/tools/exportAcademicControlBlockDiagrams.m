function manifest = exportAcademicControlBlockDiagrams(outputDir)
% exportAcademicControlBlockDiagrams - 导出 Run02/03/04 学术控制框图
arguments
    outputDir {mustBeTextScalar} = ""
end

optRoot = fileparts(fileparts(mfilename('fullpath')));
if strlength(string(outputDir)) == 0
    outputDir = fullfile(optRoot, 'docs', 'figures', 'control_block_diagrams');
end
outputDir = char(outputDir);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

names = ["run02_pose_force_control", "run03_length_cascade_control", ...
    "run04_pose_length_cascade_control"];
makers = {@makeRun02, @makeRun03, @makeRun04};
records = strings(numel(names), 4);
for index = 1:numel(names)
    fig = makers{index}();
    pngFile = fullfile(outputDir, char(names(index) + ".png"));
    pdfFile = fullfile(outputDir, char(names(index) + ".pdf"));
    svgFile = fullfile(outputDir, char(names(index) + ".svg"));
    exportgraphics(fig, pngFile, 'Resolution', 300);
    exportgraphics(fig, pdfFile, 'ContentType', 'vector');
    exportgraphics(fig, svgFile, 'ContentType', 'vector');
    close(fig);
    records(index, :) = [names(index), string(pngFile), string(pdfFile), string(svgFile)];
end
manifest = table(records(:, 1), records(:, 2), records(:, 3), records(:, 4), ...
    'VariableNames', {'name', 'png', 'pdf', 'svg'});
writetable(manifest, fullfile(outputDir, 'manifest.csv'));
fprintf('学术控制框图已导出：%s\n', outputDir);
end

function fig = makeRun02()
[fig, ax, c] = baseCanvas('Run02  理想六腿力输入的位姿反馈控制');
y = 0.60;

inputLabel(ax, 0.02, y, '$q_{\rm ref}$', c.ref);
sumNode(ax, [0.13, y], '+', '-', c);
block(ax, [0.19, y-0.075, 0.15, 0.15], ...
    {'笛卡尔 PIDF', '$K_x(s)$'}, c.feedbackFill);
block(ax, [0.39, y-0.075, 0.13, 0.15], ...
    {'固定力映射', '$J_v(q_0)^{-T}$'}, c.mappingFill);
sumNode(ax, [0.61, y], '+', '+', c);
block(ax, [0.68, y-0.075, 0.12, 0.15], ...
    {'理想力执行器', '$F_{\rm cmd}$'}, c.actuator);
block(ax, [0.85, y-0.075, 0.13, 0.15], ...
    {'Stewart 平台', '$G_{F\rightarrow q}$'}, c.plant);

arrow(ax, [0.055, y], [0.115, y], c.ref);
arrow(ax, [0.145, y], [0.19, y], c.signal, '$e_q$');
arrow(ax, [0.34, y], [0.39, y], c.feedback, '$\Delta W$');
arrow(ax, [0.52, y], [0.595, y], c.feedback, '$F_{\rm fb}$');
arrow(ax, [0.625, y], [0.68, y], c.signal, '$F_{\rm cmd}$');
arrow(ax, [0.80, y], [0.85, y], c.signal);
outputArrow(ax, [0.98, y], '$q_{\rm act}$', c.signal);

inputLabel(ax, 0.45, 0.83, '$F_{\rm FF}$', c.feedforward);
arrowPath(ax, [0.49, 0.81; 0.61, 0.81; 0.61, 0.615], c.feedforward);

branchDot(ax, [0.965, y], c.feedback);
arrowPath(ax, [0.965, y; 0.965, 0.25; 0.13, 0.25; 0.13, 0.585], c.feedback);
text(ax, 0.57, 0.205, '位姿反馈', 'HorizontalAlignment', 'center', ...
    'FontSize', 9, 'Color', c.feedback);

noteBox(ax, [0.18, 0.04, 0.66, 0.10], ...
    '$F_{\rm cmd}=F_{\rm FF}+J_v(q_0)^{-T}K_x(s)(q_{\rm ref}-q_{\rm act})$', c);
end

function fig = makeRun03()
[fig, ax, c] = baseCanvas('Run03  理想腿长串级控制');
y = 0.64;

inputLabel(ax, 0.015, y, '$L_{\rm ref}$', c.ref);
sumNode(ax, [0.105, y], '+', '-', c);
block(ax, [0.155, y-0.065, 0.105, 0.13], ...
    {'位置 P', '$K_{\rm pos}$'}, c.feedbackFill);
sumNode(ax, [0.315, y], '+', '+', c);
block(ax, [0.365, y-0.065, 0.12, 0.13], ...
    {'速度/加速度', '指令限制'}, c.limit);
sumNode(ax, [0.54, y], '+', '-', c);
block(ax, [0.59, y-0.065, 0.12, 0.13], ...
    {'腿速 PIDF', '$K_v(z)$'}, c.feedbackFill);
block(ax, [0.76, y-0.065, 0.10, 0.13], ...
    {'伺服加速度', '限制'}, c.limit);
block(ax, [0.90, y-0.105, 0.085, 0.21], ...
    {'理想运动执行器', '一阶速度响应', '$\int \dot L\,dt$', 'Prismatic Joint'}, c.actuator);

arrow(ax, [0.05, y], [0.09, y], c.ref);
arrow(ax, [0.12, y], [0.155, y], c.signal, '$e_L$');
arrow(ax, [0.26, y], [0.30, y], c.feedback, '$\Delta\dot L$');
arrow(ax, [0.33, y], [0.365, y], c.signal, '$\dot L_{\rm cmd}$');
arrow(ax, [0.485, y], [0.525, y], c.signal);
arrow(ax, [0.555, y], [0.59, y], c.signal, '$e_{\dot L}$');
arrow(ax, [0.71, y], [0.76, y], c.feedback);
arrow(ax, [0.86, y], [0.90, y], c.signal, '$u_{\rm servo}$');
outputArrow(ax, [0.985, y], '$L_{\rm act}$', c.signal);

inputLabel(ax, 0.285, 0.86, '$\dot L_{\rm ref}$', c.feedforward);
arrowPath(ax, [0.315, 0.83; 0.315, 0.655], c.feedforward);

branchDot(ax, [0.975, y], c.feedback);
arrowPath(ax, [0.975, y; 0.975, 0.31; 0.105, 0.31; 0.105, 0.625], c.feedback);
text(ax, 0.31, 0.27, '$L_{\rm act}$', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', c.feedback);

branchDot(ax, [0.93, 0.535], c.feedback);
arrowPath(ax, [0.93, 0.535; 0.93, 0.19; 0.54, 0.19; 0.54, 0.625], c.feedback);
text(ax, 0.75, 0.15, '$\dot L_{\rm act}$', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', c.feedback);

noteBox(ax, [0.18, 0.01, 0.64, 0.10], ...
    '$\dot L_{\rm cmd}=\dot L_{\rm ref}+K_{\rm pos}(L_{\rm ref}-L_{\rm act})$', c);
end

function fig = makeRun04()
[fig, ax, c] = baseCanvas('Run04  位姿外环修正的理想腿长串级控制');

% 上层：位姿误差到腿长修正
yTop = 0.78;
inputLabel(ax, 0.015, yTop, '$q_{\rm ref}$', c.ref);
sumNode(ax, [0.10, yTop], '+', '-', c);
block(ax, [0.15, yTop-0.06, 0.10, 0.12], {'低通滤波', 'LPF'}, c.filter);
block(ax, [0.30, yTop-0.06, 0.13, 0.12], ...
    {'时变参考 Jacobian', '$J_q(q_{\rm ref})$'}, c.mappingFill);
block(ax, [0.48, yTop-0.06, 0.10, 0.12], ...
    {'位姿反馈增益', '$K_{\rm pose}$'}, c.feedbackFill);
block(ax, [0.63, yTop-0.06, 0.11, 0.12], ...
    {'腿长修正限幅', 'sat'}, c.limit);
arrow(ax, [0.05, yTop], [0.085, yTop], c.ref);
arrow(ax, [0.115, yTop], [0.15, yTop], c.signal, '$e_q$');
arrow(ax, [0.25, yTop], [0.30, yTop], c.signal);
arrow(ax, [0.43, yTop], [0.48, yTop], c.mapping);
arrow(ax, [0.58, yTop], [0.63, yTop], c.feedback);

% 下层：修正腿长参考后复用 Run03
y = 0.43;
inputLabel(ax, 0.015, y, '$L_{\rm ref}$', c.ref);
sumNode(ax, [0.10, y], '+', '+', c);
sumNode(ax, [0.21, y], '+', '-', c);
block(ax, [0.26, y-0.055, 0.085, 0.11], {'位置 P', '$K_{\rm pos}$'}, c.feedbackFill);
sumNode(ax, [0.39, y], '+', '+', c);
block(ax, [0.44, y-0.055, 0.105, 0.11], {'速度/加速度', '指令限制'}, c.limit);
sumNode(ax, [0.59, y], '+', '-', c);
block(ax, [0.64, y-0.055, 0.10, 0.11], {'腿速 PIDF', '$K_v(z)$'}, c.feedbackFill);
block(ax, [0.79, y-0.055, 0.085, 0.11], {'伺服加速度', '限制'}, c.limit);
block(ax, [0.91, y-0.085, 0.075, 0.17], ...
    {'理想运动', '执行器', '$\int\dot L\,dt$'}, c.actuator);

arrow(ax, [0.05, y], [0.085, y], c.ref);
arrow(ax, [0.115, y], [0.195, y], c.signal, '$L_{\rm corr}$');
arrow(ax, [0.225, y], [0.26, y], c.signal, '$e_L$');
arrow(ax, [0.345, y], [0.375, y], c.feedback);
arrow(ax, [0.405, y], [0.44, y], c.signal);
arrow(ax, [0.545, y], [0.575, y], c.signal);
arrow(ax, [0.605, y], [0.64, y], c.signal);
arrow(ax, [0.74, y], [0.79, y], c.feedback);
arrow(ax, [0.875, y], [0.91, y], c.signal);
outputArrow(ax, [0.985, y], '$L_{\rm act}$', c.signal);

arrowPath(ax, [0.685, 0.72; 0.685, 0.62; 0.10, 0.62; 0.10, 0.445], c.feedback);
text(ax, 0.39, 0.635, '$\Delta L_{\rm pose}$', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', c.feedback);
inputLabel(ax, 0.36, 0.58, '$\dot L_{\rm ref}$', c.feedforward);
arrowPath(ax, [0.39, 0.555; 0.39, 0.445], c.feedforward);

branchDot(ax, [0.975, y], c.feedback);
arrowPath(ax, [0.975, y; 0.975, 0.20; 0.21, 0.20; 0.21, 0.415], c.feedback);
branchDot(ax, [0.94, 0.345], c.feedback);
arrowPath(ax, [0.94, 0.345; 0.94, 0.13; 0.59, 0.13; 0.59, 0.415], c.feedback);
branchDot(ax, [0.955, 0.515], c.feedback);
arrowPath(ax, [0.955, 0.515; 0.955, 0.90; 0.10, 0.90; 0.10, 0.795], c.feedback);
text(ax, 0.76, 0.10, '$\dot L_{\rm act}$', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', c.feedback);
text(ax, 0.53, 0.915, '$q_{\rm act}$', 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', c.feedback);

noteBox(ax, [0.20, 0.005, 0.60, 0.085], ...
    '$L_{\rm corr}=L_{\rm ref}+\mathrm{sat}\{K_{\rm pose}J_q(q_{\rm ref})\mathrm{LPF}(q_{\rm ref}-q_{\rm act})\}$', c);
end

function [fig, ax, c] = baseCanvas(titleText)
c = colors();
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [60, 60, 1900, 900]);
ax = axes(fig, 'Position', [0.025, 0.06, 0.95, 0.86]);
axis(ax, [0, 1.08, 0, 1]);
axis(ax, 'off');
hold(ax, 'on');
text(ax, 0.01, 0.97, titleText, 'FontName', 'Microsoft YaHei', ...
    'FontWeight', 'bold', 'FontSize', 14, 'Color', c.text, ...
    'VerticalAlignment', 'top');
end

function c = colors()
c.text = [0.12, 0.15, 0.18];
c.signal = [0.20, 0.24, 0.28];
c.ref = [0.10, 0.38, 0.67];
c.feedback = [0.05, 0.52, 0.42];
c.feedbackFill = [0.91, 0.97, 0.95];
c.feedforward = [0.88, 0.47, 0.12];
c.mapping = [0.45, 0.34, 0.67];
c.mappingFill = [0.95, 0.93, 0.98];
c.border = [0.35, 0.39, 0.43];
c.box = [0.97, 0.98, 0.99];
c.plant = [0.90, 0.94, 0.97];
c.actuator = [0.94, 0.92, 0.98];
c.limit = [0.99, 0.95, 0.87];
c.filter = [0.91, 0.96, 0.94];
end

function block(ax, pos, labels, fillColor)
rectangle(ax, 'Position', pos, 'Curvature', 0.05, 'FaceColor', fillColor, ...
    'EdgeColor', [0.35, 0.39, 0.43], 'LineWidth', 1.15);
centerX = pos(1) + pos(3) / 2;
centerY = pos(2) + pos(4) / 2;
for index = 1:numel(labels)
    offset = (numel(labels) + 1 - 2 * index) * 0.021;
    interpreter = 'none';
    if contains(labels{index}, '$')
        interpreter = 'latex';
    end
    text(ax, centerX, centerY + offset, labels{index}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontName', 'Microsoft YaHei', 'FontSize', 9.5, ...
        'FontWeight', ternary(index == 1, 'bold', 'normal'), ...
        'Interpreter', interpreter, 'Color', [0.12, 0.15, 0.18]);
end
end

function sumNode(ax, center, leftSign, bottomSign, c)
r = 0.015;
rectangle(ax, 'Position', [center(1)-r, center(2)-r, 2*r, 2*r], ...
    'Curvature', [1, 1], 'FaceColor', 'w', 'EdgeColor', c.border, 'LineWidth', 1.2);
text(ax, center(1)-0.006, center(2)+0.008, leftSign, ...
    'FontSize', 8, 'Color', c.text, 'HorizontalAlignment', 'center');
text(ax, center(1)+0.001, center(2)-0.010, bottomSign, ...
    'FontSize', 8, 'Color', c.text, 'HorizontalAlignment', 'center');
end

function inputLabel(ax, x, y, label, color)
text(ax, x, y+0.028, label, 'Interpreter', 'latex', 'FontSize', 11, ...
    'Color', color, 'HorizontalAlignment', 'left');
end

function outputArrow(ax, startPoint, label, color)
arrow(ax, startPoint, [1.045, startPoint(2)], color);
text(ax, 1.05, startPoint(2)+0.028, label, 'Interpreter', 'latex', ...
    'FontSize', 11, 'Color', color, 'HorizontalAlignment', 'left');
end

function arrow(ax, startPoint, endPoint, color, label)
if nargin < 5
    label = '';
end
quiver(ax, startPoint(1), startPoint(2), endPoint(1)-startPoint(1), ...
    endPoint(2)-startPoint(2), 0, 'Color', color, 'LineWidth', 1.35, ...
    'MaxHeadSize', 0.55, 'AutoScale', 'off');
if ~isempty(label)
    text(ax, mean([startPoint(1), endPoint(1)]), startPoint(2)+0.03, label, ...
        'Interpreter', 'latex', 'FontSize', 9.5, 'Color', color, ...
        'HorizontalAlignment', 'center');
end
end

function arrowPath(ax, points, color)
for index = 1:size(points, 1)-2
    plot(ax, points(index:index+1, 1), points(index:index+1, 2), ...
        'Color', color, 'LineWidth', 1.25);
end
arrow(ax, points(end-1, :), points(end, :), color);
end

function branchDot(ax, point, color)
plot(ax, point(1), point(2), 'o', 'MarkerSize', 4.5, ...
    'MarkerFaceColor', color, 'MarkerEdgeColor', color);
end

function noteBox(ax, pos, label, c)
rectangle(ax, 'Position', pos, 'Curvature', 0.04, ...
    'FaceColor', [0.975, 0.978, 0.982], 'EdgeColor', [0.78, 0.80, 0.82], ...
    'LineWidth', 0.8, 'LineStyle', '-');
text(ax, pos(1)+pos(3)/2, pos(2)+pos(4)/2, label, ...
    'Interpreter', 'latex', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'FontSize', 10.5, 'Color', c.text);
end

function result = ternary(condition, yesValue, noValue)
if condition
    result = yesValue;
else
    result = noValue;
end
end
