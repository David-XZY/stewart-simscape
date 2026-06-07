function [figFile, pngFile] = exportStewartMountingDiagram(traj, model, scene, resultDir, timestamp, nodeIndex)
% exportStewartMountingDiagram - 基于动画绘制内容导出挂装静态示意图
%
% 文件用途：
%   复用 animateStewartTrajectory 中的平台、支链、圆柱、长方体罩体和路径元素，
%   仅去掉标题、网格、坐标轴和坐标标签，用于论文或汇报中的挂装示意图。
%
% 输入参数：
%   traj/model/scene - 与 animateStewartTrajectory 相同的数据结构。
%   resultDir/timestamp - 图像保存目录和文件标签。
%   nodeIndex - 需要展示的轨迹节点；默认导出初始节点和最终挂装目标节点。
%
% 输出参数：
%   figFile/pngFile - 保存的 MATLAB FIG 和 PNG 文件路径。
if nargin < 4 || isempty(resultDir)
    resultDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if nargin < 5 || isempty(timestamp)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if nargin < 6 || isempty(nodeIndex)
    nodeIndex = [1, numel(traj.t)];
end
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

nodeIndex = max(1, min(numel(traj.t), nodeIndex(:).'));
stateSuffix = makeStateSuffix(nodeIndex, numel(traj.t));
figFile = cell(1, numel(nodeIndex));
pngFile = cell(1, numel(nodeIndex));

for stateIndex = 1:numel(nodeIndex)
figFile{stateIndex} = fullfile(resultDir, ['mounting_diagram_', timestamp, '_', stateSuffix{stateIndex}, '.fig']);
pngFile{stateIndex} = fullfile(resultDir, ['mounting_diagram_', timestamp, '_', stateSuffix{stateIndex}, '.png']);
fig = figure('Name', 'mounting_diagram', 'Color', 'w', ...
    'Units', 'pixels', 'Position', [100, 100, 1400, 950]);
ax = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, 'off');
grid(ax, 'off');
set(ax, 'Visible', 'off');

q = traj.Q(:, nodeIndex(stateIndex));
R = rpy2rotmZYX(q(4:6));
pTop = q(1:3) + R * model.B(:, model.legMap);

drawPlatformDisk(ax, [0; 0; 0], eye(3), model.rA, [0.62 0.69 0.76], 0.34, [0.35 0.41 0.48]);
drawPlatformDisk(ax, q(1:3), R, model.rB, [0.20 0.48 0.82], 0.42, [0.08 0.22 0.42]);
plot3(ax, model.A(1, :), model.A(2, :), model.A(3, :), 'ko', ...
    'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 5);
plot3(ax, pTop(1, :), pTop(2, :), pTop(3, :), 'bo', ...
    'MarkerFaceColor', 'b', 'MarkerSize', 5);
for legIndex = 1:6
    plot3(ax, [model.A(1, legIndex), pTop(1, legIndex)], ...
        [model.A(2, legIndex), pTop(2, legIndex)], ...
        [model.A(3, legIndex), pTop(3, legIndex)], 'b-', 'LineWidth', 1.2);
end

drawBox(ax, scene.box.center_S, scene.box.R_S, scene.box.halfSize, [0.85 0.72 0.28], 0.14, [0.45 0.35 0.1]);
for obstacleIndex = 1:numel(scene.hood.obstacles)
    obstacle = scene.hood.obstacles(obstacleIndex);
    drawBox(ax, obstacle.center_S, obstacle.R_S, obstacle.halfSize, [0.85 0.18 0.18], 0.25, [0.45 0.05 0.05]);
end

drawCylinder(ax, q, scene);

view(ax, 45, 25);
hoodCenters = reshape([scene.hood.obstacles.center_S], 3, []);
allPoints = [model.A, pTop, traj.Q(1:3, :), scene.box.center_S, hoodCenters, scene.qWaypoint(1:3), scene.qGoal(1:3)];
pad = 0.35;
xlim(ax, [min(allPoints(1, :))-pad, max(allPoints(1, :))+pad]);
ylim(ax, [min(allPoints(2, :))-pad, max(allPoints(2, :))+pad]);
zlim(ax, [min(allPoints(3, :))-pad, max(allPoints(3, :))+pad]);

camlight(ax, 'headlight');
lighting(ax, 'gouraud');
drawnow;
savefig(fig, figFile{stateIndex});
exportgraphics(fig, pngFile{stateIndex}, 'Resolution', 300);
fprintf('挂装示意 FIG 已保存：%s\n', figFile{stateIndex});
fprintf('挂装示意 PNG 已保存：%s\n', pngFile{stateIndex});
close(fig);
end
if isscalar(figFile)
    figFile = figFile{1};
    pngFile = pngFile{1};
end
end

function stateSuffix = makeStateSuffix(nodeIndex, numNodes)
stateSuffix = cell(1, numel(nodeIndex));
for index = 1:numel(nodeIndex)
    if nodeIndex(index) == 1
        stateSuffix{index} = 'initial';
    elseif nodeIndex(index) == numNodes
        stateSuffix{index} = 'final';
    else
        stateSuffix{index} = sprintf('node_%03d', nodeIndex(index));
    end
end
end

function drawPlatformDisk(ax, center, R, radius, faceColor, faceAlpha, edgeColor)
angles = linspace(0, 2*pi, 48);
circleLocal = radius * [cos(angles); sin(angles); zeros(1, numel(angles))];
circleWorld = center(:) + R * circleLocal;
patch(ax, circleWorld(1, :), circleWorld(2, :), circleWorld(3, :), faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 1.1);
end

function drawCylinder(ax, q, scene)
R = rpy2rotmZYX(q(4:6));
center = q(1:3) + R * scene.objectCylinder.center_P;
axisVector = R * scene.objectCylinder.axis_P;
axisVector = axisVector / norm(axisVector);
[X, Y, Z] = cylinder(scene.objectCylinder.radius, 24);
Z = (Z - 0.5) * scene.objectCylinder.length;
pts = [Z(:)'; X(:)'; Y(:)'];
basis = makeBasis(axisVector);
world = center + basis * pts;
Xw = reshape(world(1, :), size(X));
Yw = reshape(world(2, :), size(Y));
Zw = reshape(world(3, :), size(Z));
surf(ax, Xw, Yw, Zw, 'FaceColor', [0.1 0.55 0.85], ...
    'FaceAlpha', 0.55, 'EdgeColor', 'none');
end

function basis = makeBasis(axisVector)
tmp = [0; 0; 1];
if abs(dot(tmp, axisVector)) > 0.9
    tmp = [0; 1; 0];
end
v2 = cross(tmp, axisVector);
v2 = v2 / norm(v2);
v3 = cross(axisVector, v2);
basis = [axisVector, v2, v3];
end

function drawBox(ax, center, R, halfSize, faceColor, faceAlpha, edgeColor)
cornersLocal = [ -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
                 -1 -1  1;  1 -1  1;  1  1  1; -1  1  1]' .* halfSize;
corners = center + R * cornersLocal;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', corners.', 'Faces', faces, 'FaceColor', faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 1.2);
end
