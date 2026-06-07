function animationFile = animateStewartTrajectory(traj, model, scene, resultDir, timestamp)
% animateStewartTrajectory - 绘制圆柱体-长方体两阶段轨迹动画
%
% 文件用途：
%   显示定平台、动平台、六条支链、运动圆柱体、固定长方体、途径点和目标点。
%
% 输入参数：
%   traj struct  - 优化后的节点轨迹。
%   model struct - Stewart 平台几何参数。
%   scene struct - 圆柱体-长方体两阶段场景。
%   resultDir/timestamp - 动画保存目录和标签。
%
% 输出参数：
%   animationFile - 保存的 MP4 文件路径；若视频写入失败则为空字符串。
%
% 核心公式：
%   圆柱中心 p_C=p+R*p_C^P，轴线 a_C=R*a_C^P；长方体由 box.center/R/halfSize 定义。
%
% 在优化链路中的作用：
%   run_01_ihsid_trajectory 求解后调用，用于离线视觉检查。

if nargin < 4 || isempty(resultDir)
    resultDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if nargin < 5 || isempty(timestamp)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

animationFile = fullfile(resultDir, ['cylinder_box_two_phase_', timestamp, '.mp4']);
fig = figure('Name', 'cylinder_box_two_phase_animation', 'Color', 'w');
try
    writer = VideoWriter(animationFile, 'MPEG-4');
    writer.FrameRate = 12;
    open(writer);
catch
    animationFile = '';
    writer = [];
end

for nodeIndex = 1:numel(traj.t)
    clf(fig);
    hold on; grid on; axis equal;
    q = traj.Q(:, nodeIndex);
    R = rpy2rotmZYX(q(4:6));
    pTop = q(1:3) + R * model.B(:, model.legMap);
    drawPlatformDisk([0; 0; 0], eye(3), model.rA, [0.62 0.69 0.76], 0.34, [0.35 0.41 0.48]);
    drawPlatformDisk(q(1:3), R, model.rB, [0.20 0.48 0.82], 0.42, [0.08 0.22 0.42]);
    plot3(model.A(1, :), model.A(2, :), model.A(3, :), 'ko', 'MarkerFaceColor', [0.2 0.2 0.2]);
    plot3(pTop(1, :), pTop(2, :), pTop(3, :), 'bo', 'MarkerFaceColor', 'b');
    for legIndex = 1:6
        plot3([model.A(1, legIndex), pTop(1, legIndex)], ...
              [model.A(2, legIndex), pTop(2, legIndex)], ...
              [model.A(3, legIndex), pTop(3, legIndex)], 'b-', 'LineWidth', 1.0);
    end
    drawBox(scene.box.center_S, scene.box.R_S, scene.box.halfSize, [0.85 0.72 0.28], 0.14, [0.45 0.35 0.1]);
    for obstacleIndex = 1:numel(scene.hood.obstacles)
        obstacle = scene.hood.obstacles(obstacleIndex);
        drawBox(obstacle.center_S, obstacle.R_S, obstacle.halfSize, [0.85 0.18 0.18], 0.25, [0.45 0.05 0.05]);
    end
    drawCylinder(q, scene);
    plot3(traj.Q(1, :), traj.Q(2, :), traj.Q(3, :), 'k-', 'LineWidth', 1.0);
    plot3(scene.qWaypoint(1), scene.qWaypoint(2), scene.qWaypoint(3), 'mp', 'MarkerFaceColor', 'm', 'MarkerSize', 10);
    plot3(scene.qGoal(1), scene.qGoal(2), scene.qGoal(3), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('t = %.2f s, clearance = %.4f m', traj.t(nodeIndex), traj.minClearance(nodeIndex)));
    view(45, 25);
    hoodCenters = reshape([scene.hood.obstacles.center_S], 3, []);
    allPoints = [model.A, pTop, traj.Q(1:3, :), scene.box.center_S, hoodCenters, scene.qWaypoint(1:3), scene.qGoal(1:3)];
    pad = 0.35;
    xlim([min(allPoints(1,:))-pad, max(allPoints(1,:))+pad]);
    ylim([min(allPoints(2,:))-pad, max(allPoints(2,:))+pad]);
    zlim([min(allPoints(3,:))-pad, max(allPoints(3,:))+pad]);
    drawnow;
    if ~isempty(writer)
        writeVideo(writer, getframe(fig));
    end
end

if ~isempty(writer)
    close(writer);
    fprintf('动画已保存：%s\n', animationFile);
end
close(fig);
end

function drawPlatformDisk(center, R, radius, faceColor, faceAlpha, edgeColor)
angles = linspace(0, 2*pi, 48);
circleLocal = radius * [cos(angles); sin(angles); zeros(1, numel(angles))];
circleWorld = center(:) + R * circleLocal;
patch(circleWorld(1, :), circleWorld(2, :), circleWorld(3, :), faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 1.1);
end

function drawCylinder(q, scene)
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
surf(Xw, Yw, Zw, 'FaceColor', [0.1 0.55 0.85], 'FaceAlpha', 0.55, 'EdgeColor', 'none');
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

function drawBox(center, R, halfSize, faceColor, faceAlpha, edgeColor)
cornersLocal = [ -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
                 -1 -1  1;  1 -1  1;  1  1  1; -1  1  1]' .* halfSize;
corners = center + R * cornersLocal;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', corners.', 'Faces', faces, 'FaceColor', faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 1.2);
end
