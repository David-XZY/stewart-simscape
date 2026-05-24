function plotFiles = plotOptResult(traj, model, scene, result, resultDir, timestamp)
% plotOptResult - 绘制并保存预对准 HS 轨迹优化结果
%
% 文件用途：
%   绘制位姿、速度、腿长、腿速、腿加速度、驱动力、奇异性和碰撞最小间隙。
%
% 输入参数：
%   traj struct  - 主脚本由决策变量重建的节点/中点轨迹。
%   model struct - Stewart 几何、执行器、奇异性和目标函数参数。
%   scene struct - 预对准场景和碰撞安全距离。
%   result struct - analyzeHSResult 生成的诊断结构。
%   resultDir/timestamp - 图片保存目录和时间戳。
%
% 输出参数：
%   plotFiles cell - 已保存的 PNG 文件路径列表。
%
% 核心公式：
%   所有边界来自 model/scene；中点以 x 标记，节点以 o 标记，便于检查 HS 网格。
%
% 在优化链路中的作用：
%   将求解结果转换为可视化诊断图，帮助判断失败时是哪类硬约束先触发。
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

fig = figure('Name', 'prealign_pose_velocity', 'Color', 'w');
tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
names = {'x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'};
Q = traj.Q;
V = traj.V;
Q(4:6, :) = rad2deg(Q(4:6, :));
V(4:6, :) = rad2deg(V(4:6, :));
for i = 1:6
    nexttile;
    plot(t, Q(i, :), 'o-', 'LineWidth', 1.1);
    grid on; xlabel('t [s]'); ylabel(names{i});
end
for i = 1:6
    nexttile;
    plot(t, V(i, :), 'o-', 'LineWidth', 1.1);
    grid on; xlabel('t [s]'); ylabel(['d ', names{i}]);
end
plotFiles{end+1} = saveFigure(fig, resultDir, ['pose_velocity_', timestamp]); %#ok<AGROW>

plotFiles{end+1} = plotLegSeries(t, tc, traj.L, traj.Lmid, model.lmin(1), model.lmax(1), ...
    'leg_length', 'leg length [m]', resultDir, timestamp); %#ok<AGROW>
plotFiles{end+1} = plotLegSeries(t, tc, traj.Ld, traj.LdMid, -model.actuator.ldotMax(1), model.actuator.ldotMax(1), ...
    'leg_speed', 'leg speed [m/s]', resultDir, timestamp); %#ok<AGROW>
plotFiles{end+1} = plotLegSeries(t, tc, traj.Ldd, traj.LddMid, -model.actuator.lddotMax(1), model.actuator.lddotMax(1), ...
    'leg_acceleration', 'leg acceleration [m/s^2]', resultDir, timestamp); %#ok<AGROW>
plotFiles{end+1} = plotLegSeries(t, tc, traj.Unode, traj.Umid, model.actuator.forceMin(1), model.actuator.forceMax(1), ...
    'actuator_force', 'force [N]', resultDir, timestamp); %#ok<AGROW>

fig = figure('Name', 'singularity_collision', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(t, traj.sigmaMin, 'o-', tc, traj.sigmaMinMid, 'x', 'LineWidth', 1.1);
yline(model.singularity.sigmaMinSafe, 'r--');
grid on; xlabel('t [s]'); ylabel('sigmaMin');
title('normalized spatial Jacobian minimum singular value');
nexttile;
plot(t, traj.condJ, 'o-', tc, traj.condJMid, 'x', 'LineWidth', 1.1);
yline(model.singularity.condWarning, 'k--');
grid on; xlabel('t [s]'); ylabel('condJ');
title('normalized spatial Jacobian condition number');
nexttile;
plot(t, traj.minClearance, 'o-', tc, traj.minClearanceMid, 'x', 'LineWidth', 1.1);
yline(scene.collision.safeDistance, 'r--');
grid on; xlabel('t [s]'); ylabel('clearance [m]');
title(sprintf('minimum clearance, final report %.6f m', result.constraint.minClearance));
plotFiles{end+1} = saveFigure(fig, resultDir, ['singularity_collision_', timestamp]); %#ok<AGROW>

fig = figure('Name', 'platform_path_3d', 'Color', 'w');
plot3(traj.Q(1, :), traj.Q(2, :), traj.Q(3, :), 'o-', 'LineWidth', 1.4);
hold on;
plot3(scene.q0(1), scene.q0(2), scene.q0(3), 'go', 'MarkerFaceColor', 'g');
plot3(scene.qPre(1), scene.qPre(2), scene.qPre(3), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('platform origin path');
legend({'trajectory','q0','qPre'}, 'Location', 'best');
plotFiles{end+1} = saveFigure(fig, resultDir, ['path_3d_', timestamp]); %#ok<AGROW>
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

function fileName = saveFigure(fig, resultDir, baseName)
fileName = fullfile(resultDir, [baseName, '.png']);
exportgraphics(fig, fileName, 'Resolution', 160);
end
