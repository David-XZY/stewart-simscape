function draw_stewart_platform_diagram(outputDir)
%DRAW_STEWART_PLATFORM_DIAGRAM 绘制 Stewart 平台结构与上下平台铰点示意图。
%   draw_stewart_platform_diagram() 默认将高清 PNG/PDF 导出到原 JPG 所在目录。
%   draw_stewart_platform_diagram(outputDir) 可指定导出目录。

if nargin < 1 || isempty(outputDir)
    outputDir = 'D:\NPU\并联机器人\visio绘图';
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

pngFile = fullfile(outputDir, 'stewart平台简图_matlab清晰版.png');
pdfFile = fullfile(outputDir, 'stewart平台简图_matlab清晰版.pdf');

fig = figure('Color', 'w', ...
    'Units', 'pixels', ...
    'Position', [100, 100, 1600, 540], ...
    'Name', 'Stewart 平台示意图');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, 'off');
xlim(ax, [-0.25, 15.4]);
ylim(ax, [-0.2, 5.1]);
set(ax, 'YDir', 'normal');

style.line = 0.75;
style.thin = 0.58;
style.dash = [5, 4];
style.fontSize = 11.5;
style.smallFontSize = 9.5;
style.chineseFontSize = 10.5;
style.markerSize = 5.8;
style.black = [0, 0, 0];
style.gray = [0.45, 0.45, 0.45];

drawSideView(ax, 0.15, 0.2, 1.05, style);
drawAPlatformView(ax, 7.0, 0.55, 1.28, style);
drawBPlatformView(ax, 12.25, 1.05, 1.02, style);

drawnow;
exportgraphics(fig, pngFile, 'Resolution', 600);
exportgraphics(fig, pdfFile, 'ContentType', 'vector');

fprintf('已导出 PNG: %s\n', pngFile);
fprintf('已导出 PDF: %s\n', pdfFile);
end

function drawSideView(ax, ox, oy, s, style)
% 左侧斜视图：上下椭圆平台、六条驱动腿、铰点与中文说明。

lw = style.line;

baseCenter = tr(ox, oy, s, [1.85, 1.05]);
topCenter = tr(ox, oy, s, [1.85, 3.78]);
drawEllipse(ax, baseCenter, 1.75 * s, 0.66 * s, 0, lw);
drawEllipse(ax, topCenter, 1.20 * s, 0.55 * s, 0, lw);

A = [
    3.08, 0.50;  % A1
    3.60, 1.00;  % A2
    2.52, 1.72;  % A3
    1.18, 1.72;  % A4
    0.35, 1.00;  % A5
    0.78, 0.50]; % A6
B = [
    2.35, 3.24;  % B1
    3.12, 3.95;  % B2
    2.75, 4.37;  % B3
    1.32, 4.37;  % B4
    0.72, 3.95;  % B5
    1.72, 3.24]; % B6
A = tr(ox, oy, s, A);
B = tr(ox, oy, s, B);

% 透视图中按原图保留两条虚线腿，其余为实线腿。
drawSegment(ax, A(5,:), B(5,:), '-', lw);
drawSegment(ax, A(2,:), B(2,:), '-', lw);
drawSegment(ax, A(6,:), B(6,:), '-', lw);
drawSegment(ax, A(1,:), B(1,:), '-', lw);
drawSegment(ax, A(4,:), B(4,:), '--', 0.8);
drawSegment(ax, A(3,:), B(3,:), '--', 0.8);

drawOpenPoints(ax, A, style.markerSize, lw);
drawOpenPoints(ax, B, style.markerSize, lw);

labelOffsetA = [
    0.18, -0.16;
    0.12, -0.02;
    0.06, 0.12;
   -0.34, 0.12;
   -0.38, -0.03;
   -0.25, -0.18] * s;
labelOffsetB = [
   -0.05, 0.13;
    0.08, 0.08;
    0.08, 0.17;
   -0.24, 0.17;
   -0.36, 0.08;
    0.02, 0.13] * s;
for k = 1:6
    drawLatexText(ax, A(k,:) + labelOffsetA(k,:), sprintf('$A_%d$', k), style.fontSize);
    drawLatexText(ax, B(k,:) + labelOffsetB(k,:), sprintf('$B_%d$', k), style.fontSize);
end

% 中文引线标注。
leader1 = tr(ox, oy, s, [2.78, 4.35; 3.35, 4.78; 4.05, 4.78]);
plot(ax, leader1(:,1), leader1(:,2), 'k-', 'LineWidth', 0.8);
text(ax, leader1(end,1) + 0.05 * s, leader1(end,2), '动平台', ...
    'FontName', 'SimSun', 'FontSize', style.chineseFontSize, ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

leader2 = tr(ox, oy, s, [3.28, 2.28; 3.80, 2.95; 4.18, 2.95]);
plot(ax, leader2(:,1), leader2(:,2), 'k-', 'LineWidth', 0.8);
text(ax, leader2(end,1) + 0.05 * s, leader2(end,2), '驱动腿', ...
    'FontName', 'SimSun', 'FontSize', style.chineseFontSize, ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

leader3 = tr(ox, oy, s, [3.08, 1.48; 3.65, 2.05; 4.18, 2.05]);
plot(ax, leader3(:,1), leader3(:,2), 'k-', 'LineWidth', 0.8);
text(ax, leader3(end,1) + 0.05 * s, leader3(end,2), '定平台', ...
    'FontName', 'SimSun', 'FontSize', style.chineseFontSize, ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
end

function drawAPlatformView(ax, ox, oy, s, style)
% 中间定平台平面图：A 铰点、半径、坐标轴和角度标注。

c = [ox, oy + 1.75 * s];
R = 1.35 * s;
theta = deg2rad([-58, -22, 72, 108, 202, 238]);
A = c + R * [cos(theta(:)), sin(theta(:))];

drawEllipse(ax, c, R, R, 0, style.line);

pairIdx = [4, 3; 3, 1; 1, 6; 6, 5; 5, 4; 2, 3];
for i = 1:size(pairIdx, 1)
    drawSegment(ax, A(pairIdx(i,1),:), A(pairIdx(i,2),:), '-', style.thin);
end

drawSegment(ax, c, A(3,:), '-', style.line);
drawSegment(ax, c, A(5,:), '-', style.line);
drawSegment(ax, c, A(1,:), '--', 0.75);
drawSegment(ax, c + [0, R], c + [0, -1.38 * R], '-', style.thin);
drawAxisArrow(ax, c, [1.28 * R, 0], style.line);
drawAxisArrow(ax, c, [0, -1.35 * R], style.line);

drawFilledPoints(ax, A, 4.2, style.line);
labelOffset = s * [
    0.09, -0.11;
    0.12, -0.02;
    0.02, 0.11;
   -0.27, 0.11;
   -0.35, -0.02;
   -0.20, -0.11];
for k = 1:6
    drawLatexText(ax, A(k,:) + labelOffset(k,:), sprintf('$A_%d$', k), style.fontSize);
end

drawLatexText(ax, c + [0.38 * R, 0.35 * R], '$R_a$', style.fontSize);
drawLatexText(ax, c + [1.25 * R, 0.08 * R], '$Y_a$', style.fontSize);
drawLatexText(ax, c + [0.07 * R, -1.43 * R], '$X_a$', style.fontSize);

drawArc(ax, c, 0.30 * R, 90, 202, '-', style.thin);
drawArc(ax, c, 0.36 * R, -58, -90, '--', 0.75);
drawLatexText(ax, c + [-0.58 * R, 0.12 * R], '$120^{\circ}$', style.smallFontSize);
drawLatexText(ax, c + [0.25 * R, -0.42 * R], '$\theta_a$', style.fontSize);
end

function drawBPlatformView(ax, ox, oy, s, style)
% 右侧动平台平面图：B 铰点、半径、坐标轴和角度标注。

c = [ox, oy + 1.15 * s];
R = 0.95 * s;
theta = deg2rad([-90, 18, 62, 132, 178, 238]);
B = c + R * [cos(theta(:)), sin(theta(:))];

drawEllipse(ax, c, R, R, 0, style.line);

pairIdx = [4, 3; 3, 1; 1, 6; 5, 4; 2, 3; 2, 1];
for i = 1:size(pairIdx, 1)
    drawSegment(ax, B(pairIdx(i,1),:), B(pairIdx(i,2),:), '-', style.thin);
end

drawSegment(ax, c, B(3,:), '-', style.line);
drawSegment(ax, c, B(1,:), '--', 0.75);
drawAxisArrow(ax, c, [1.25 * R, 0], style.line);
drawAxisArrow(ax, c, [0, -1.30 * R], style.line);

drawFilledPoints(ax, B, 3.8, style.line);
labelOffset = s * [
    0.03, -0.14;
    0.13, 0.00;
    0.04, 0.11;
   -0.23, 0.10;
   -0.25, -0.03;
   -0.16, -0.12];
for k = 1:6
    drawLatexText(ax, B(k,:) + labelOffset(k,:), sprintf('$B_%d$', k), style.fontSize);
end

drawLatexText(ax, c + [0.30 * R, 0.36 * R], '$R_b$', style.fontSize);
drawLatexText(ax, c + [1.18 * R, -0.15 * R], '$Y_b$', style.fontSize);
drawLatexText(ax, c + [0.08 * R, -1.30 * R], '$X_b$', style.fontSize);

drawArc(ax, c, 0.34 * R, -90, -106, '--', 0.75);
drawLatexText(ax, c + [0.12 * R, -0.78 * R], '$\theta_b$', style.smallFontSize);
end

function p = tr(ox, oy, s, p0)
% 将局部坐标变换到整张图的坐标系。
p = [ox, oy] + s * p0;
end

function drawEllipse(ax, c, rx, ry, phi, lw)
t = linspace(0, 2*pi, 500);
xy = [rx * cos(t); ry * sin(t)];
rot = [cos(phi), -sin(phi); sin(phi), cos(phi)];
xy = rot * xy + c(:);
plot(ax, xy(1,:), xy(2,:), 'k-', 'LineWidth', lw);
end

function drawSegment(ax, p1, p2, lineStyle, lw)
plot(ax, [p1(1), p2(1)], [p1(2), p2(2)], lineStyle, ...
    'Color', 'k', 'LineWidth', lw);
end

function drawOpenPoints(ax, p, markerSize, lw)
plot(ax, p(:,1), p(:,2), 'o', ...
    'MarkerSize', markerSize, ...
    'MarkerFaceColor', 'w', ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', lw);
end

function drawFilledPoints(ax, p, markerSize, lw)
plot(ax, p(:,1), p(:,2), 'o', ...
    'MarkerSize', markerSize, ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', lw);
end

function drawLatexText(ax, p, str, fontSize)
text(ax, p(1), p(2), str, ...
    'Interpreter', 'latex', ...
    'FontSize', fontSize, ...
    'FontName', 'Times New Roman', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle');
end

function drawAxisArrow(ax, p, v, lw)
quiver(ax, p(1), p(2), v(1), v(2), 0, ...
    'Color', 'k', ...
    'LineWidth', lw, ...
    'MaxHeadSize', 0.22, ...
    'AutoScale', 'off');
end

function drawArc(ax, c, r, a1, a2, lineStyle, lw)
a = linspace(deg2rad(a1), deg2rad(a2), 80);
x = c(1) + r * cos(a);
y = c(2) + r * sin(a);
plot(ax, x, y, lineStyle, 'Color', 'k', 'LineWidth', lw);
end
