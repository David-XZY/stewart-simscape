function plotFiles = plotOptResult(traj, model, scene, result, resultDir, timestamp)
% plotOptResult - 绘制并保存圆柱体-长方体两阶段 HS 轨迹优化结果
%
% 文件用途：
%   绘制位姿、速度、腿长、腿速、腿加速度、驱动力、奇异性和碰撞最小间隙。
%
% 输入参数：
%   traj struct  - 主脚本由决策变量重建的节点/中点轨迹。
%   model struct - Stewart 几何、执行器、奇异性和目标函数参数。
%   scene struct - 圆柱体-长方体两阶段场景和碰撞安全距离。
%   result struct - run_01_ihsid_trajectory 重建并汇总的诊断结构。
%   resultDir/timestamp - 图片保存目录和时间戳。
%
% 输出参数：
%   plotFiles cell - 已保存的 PNG 文件路径列表。
%
% 核心公式：
%   所有边界来自 model/scene；中点以 x 标记，节点以 o 标记，便于检查 HS 网格。
%
% 在优化链路中的作用：
%   将求解结果转换为可视化诊断图，帮助判断失败时是哪类约束或后验诊断先触发。
if nargin < 5 || isempty(resultDir)
    resultDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if nargin < 6 || isempty(timestamp)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

plotFiles = {};
t = traj.t;
tc = traj.tc;

fig = figure('Name', 'cylinder_box_pose_velocity', 'Color', 'w');
tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
names = {'x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'};
Q = traj.Q;
V = traj.V;
Q(4:6, :) = rad2deg(Q(4:6, :));
V(4:6, :) = rad2deg(V(4:6, :));
for i = 1:6
    nexttile;
    plot(t, Q(i, :), 'o-', 'LineWidth', 1.1);
    xline(scene.phase.durationApproach, 'k--');
    grid on; xlabel('t [s]'); ylabel(names{i});
end
for i = 1:6
    nexttile;
    plot(t, V(i, :), 'o-', 'LineWidth', 1.1);
    xline(scene.phase.durationApproach, 'k--');
    grid on; xlabel('t [s]'); ylabel(['d ', names{i}]);
end
plotFiles{end+1} = saveFigure(fig, resultDir, ['pose_velocity_', timestamp]);

plotFiles{end+1} = plotLegSeries(t, tc, traj.L, traj.Lmid, model.lmin(1), model.lmax(1), ...
    'leg_length', 'leg length [m]', resultDir, timestamp);
plotFiles{end+1} = plotLegSeries(t, tc, traj.Ld, traj.LdMid, -model.actuator.ldotMax(1), model.actuator.ldotMax(1), ...
    'leg_speed', 'leg speed [m/s]', resultDir, timestamp);
plotFiles{end+1} = plotLegSeries(t, tc, traj.Ldd, traj.LddMid, -model.actuator.lddotMax(1), model.actuator.lddotMax(1), ...
    'leg_acceleration', 'leg acceleration [m/s^2]', resultDir, timestamp);
plotFiles{end+1} = plotLegSeries(t, tc, traj.Unode, traj.Umid, model.actuator.forceMin(1), model.actuator.forceMax(1), ...
    'actuator_force', 'force [N]', resultDir, timestamp);
plotFiles{end+1} = plotForceRateSeries(t, tc, traj.Unode, traj.Umid, model.objective.forceRateScale, ...
    resultDir, timestamp);

fig = figure('Name', 'singularity_collision', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(t, traj.sigmaMin, 'o-', tc, traj.sigmaMinMid, 'x', 'LineWidth', 1.1);
yline(model.singularity.sigmaMinSafe, 'r--');
xline(scene.phase.durationApproach, 'k--');
grid on; xlabel('t [s]'); ylabel('sigmaMin');
title('normalized spatial Jacobian minimum singular value (post-check threshold)');
nexttile;
plot(t, traj.condJ, 'o-', tc, traj.condJMid, 'x', 'LineWidth', 1.1);
yline(model.singularity.condWarning, 'k--');
xline(scene.phase.durationApproach, 'k--');
grid on; xlabel('t [s]'); ylabel('condJ');
title('normalized spatial Jacobian condition number (post-check warning)');
nexttile;
if isfield(traj, 'collisionDistances')
    plot(t, traj.collisionDistances.', 'o-', tc, traj.collisionDistancesMid.', 'x', 'LineWidth', 1.1);
    legend({scene.hood.obstacles.name}, 'Location', 'best');
else
    plot(t, traj.minClearance, 'o-', tc, traj.minClearanceMid, 'x', 'LineWidth', 1.1);
end
yline(scene.collision.safeDistance, 'r--');
yline(scene.collision.finalGap, 'g:');
xline(scene.phase.durationApproach, 'k--');
grid on; xlabel('t [s]'); ylabel('clearance [m]');
title(sprintf('cylinder-hood clearance, min %.6f m, final %.6f m', ...
    result.constraint.minClearance, result.constraint.finalGap));
plotFiles{end+1} = saveFigure(fig, resultDir, ['singularity_collision_', timestamp]);

fig = figure('Name', 'platform_path_3d', 'Color', 'w');
plot3(traj.Q(1, :), traj.Q(2, :), traj.Q(3, :), 'o-', 'LineWidth', 1.4);
hold on;
plot3(scene.q0(1), scene.q0(2), scene.q0(3), 'go', 'MarkerFaceColor', 'g');
plot3(scene.qWaypoint(1), scene.qWaypoint(2), scene.qWaypoint(3), 'mp', 'MarkerFaceColor', 'm', 'MarkerSize', 12);
plot3(scene.qGoal(1), scene.qGoal(2), scene.qGoal(3), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
drawPlatformDisk([0; 0; 0], eye(3), model.rA, [0.62 0.69 0.76], 0.24, [0.35 0.41 0.48]);
drawPlatformDisk(scene.q0(1:3), rpy2rotmZYX(scene.q0(4:6)), model.rB, [0.20 0.48 0.82], 0.16, 'none');
drawPlatformDisk(scene.qWaypoint(1:3), rpy2rotmZYX(scene.qWaypoint(4:6)), model.rB, [0.20 0.48 0.82], 0.22, 'none');
drawPlatformDisk(scene.qGoal(1:3), rpy2rotmZYX(scene.qGoal(4:6)), model.rB, [0.20 0.48 0.82], 0.28, [0.08 0.22 0.42]);
drawBox(scene.box.center_S, scene.box.R_S, scene.box.halfSize, [0.85 0.72 0.28], 0.12, [0.45 0.35 0.1]);
for obstacleIndex = 1:numel(scene.hood.obstacles)
    obstacle = scene.hood.obstacles(obstacleIndex);
    drawBox(obstacle.center_S, obstacle.R_S, obstacle.halfSize, [0.85 0.18 0.18], 0.22, [0.45 0.05 0.05]);
end
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('platform origin path');
legend({'trajectory','q0','qWaypoint','qGoal'}, 'Location', 'best');
plotFiles{end+1} = saveFigure(fig, resultDir, ['path_3d_', timestamp]);
end

function fileName = plotLegSeries(t, tc, nodeValues, midValues, lowerBound, upperBound, baseName, yLabelText, resultDir, timestamp)
fig = figure('Name', baseName, 'Color', 'w');
plot(t, nodeValues.', 'o-', 'LineWidth', 1.0);
hold on;
plot(tc, midValues.', 'x', 'LineWidth', 1.0);
yline(lowerBound, 'k--');
yline(upperBound, 'k--');
grid on;
xlabel('t [s]');
ylabel(yLabelText);
title(strrep(baseName, '_', ' '));
fileName = saveFigure(fig, resultDir, [baseName, '_', timestamp]);
end

function fileName = plotForceRateSeries(t, tc, forceNode, forceMid, forceRateScale, resultDir, timestamp)
% plotForceRateSeries - 绘制左右半区间驱动力变化率
numIntervals = numel(tc);
tRate = zeros(1, 2*numIntervals);
forceRate = zeros(size(forceNode, 1), 2*numIntervals);
for intervalIndex = 1:numIntervals
    h = t(intervalIndex + 1) - t(intervalIndex);
    leftIndex = 2*intervalIndex - 1;
    rightIndex = 2*intervalIndex;
    tRate(leftIndex) = 0.5 * (t(intervalIndex) + tc(intervalIndex));
    tRate(rightIndex) = 0.5 * (tc(intervalIndex) + t(intervalIndex + 1));
    forceRate(:, leftIndex) = (forceMid(:, intervalIndex) - forceNode(:, intervalIndex)) ./ (h/2);
    forceRate(:, rightIndex) = (forceNode(:, intervalIndex + 1) - forceMid(:, intervalIndex)) ./ (h/2);
end

fig = figure('Name', 'actuator_force_rate', 'Color', 'w');
plot(tRate, forceRate.', 'o-', 'LineWidth', 1.0);
hold on;
yline(forceRateScale, 'k--');
yline(-forceRateScale, 'k--');
grid on;
xlabel('t [s]');
ylabel('force rate [N/s]');
title('actuator force rate');
fileName = saveFigure(fig, resultDir, ['actuator_force_rate_', timestamp]);
end

function fileName = saveFigure(fig, resultDir, baseName)
fileName = fullfile(resultDir, [baseName, '.png']);
exportgraphics(fig, fileName, 'Resolution', 160);
end

function drawPlatformDisk(center, R, radius, faceColor, alphaValue, edgeColor)
angles = linspace(0, 2*pi, 48);
circleLocal = radius * [cos(angles); sin(angles); zeros(1, numel(angles))];
circleWorld = center(:) + R * circleLocal;
patch(circleWorld(1, :), circleWorld(2, :), circleWorld(3, :), faceColor, ...
    'FaceAlpha', alphaValue, 'EdgeColor', edgeColor, 'LineWidth', 1.0);
end

function drawBox(center, R, halfSize, faceColor, faceAlpha, edgeColor)
cornersLocal = [ -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
                 -1 -1  1;  1 -1  1;  1  1  1; -1  1  1]' .* halfSize;
corners = center + R * cornersLocal;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', corners.', 'Faces', faces, 'FaceColor', faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 1.0);
end
