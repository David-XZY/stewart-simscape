function animationFile = animateStewartTrajectory(traj, model, scene, resultDir, timestamp)
% animateStewartTrajectory - 生成 Stewart 预对准轨迹三维动画
%
% 文件用途：
%   显示定平台、动平台、六条支链、弹体球链、四个挂耳包络球、挂架 OBB、平台轨迹
%   以及当前最小碰撞间隙，并固定保存为 5 秒 MP4；若 MP4 不可用则退化为 5 秒 GIF。
%
% 输入参数：
%   traj struct  - 节点轨迹和诊断量。
%   model struct - Stewart 几何参数。
%   scene struct - 弹体、挂耳、挂架和碰撞参数。
%   resultDir/timestamp - 动画保存目录和时间戳。
%
% 输出参数：
%   animationFile char - 动画文件完整路径；默认播放时长为 5 秒。
%
% 核心公式：
%   动平台铰点 P_i=p+R*B_i；弹体/挂耳球心 c_W=p+R*c_P；挂架 OBB 由
%   scene.collisionGeometry.rack.center/R/halfSize 定义。
%
% 在优化链路中的作用：
%   提供不依赖 Simscape 的离线视觉检查，确认轨迹、弹体、挂耳和挂架相对位置合理。
if nargin < 4 || isempty(resultDir)
    resultDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if nargin < 5 || isempty(timestamp)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

animationDuration = 5.0;
frameRate = 15;
frameCount = max(2, round(animationDuration * frameRate));
sampleTimes = linspace(traj.t(1), traj.t(end), frameCount);
sampleQ = interp1(traj.t(:), traj.Q.', sampleTimes(:), 'pchip').';
sampleClearance = interp1(traj.t(:), traj.minClearance(:), sampleTimes(:), 'pchip').';
sampleSigma = interp1(traj.t(:), traj.sigmaMin(:), sampleTimes(:), 'pchip').';
pathQ = sampleQ;

mp4File = fullfile(resultDir, ['stewart_prealign_5s_', timestamp, '.mp4']);
gifFile = fullfile(resultDir, ['stewart_prealign_5s_', timestamp, '.gif']);
useMp4 = true;
try
    writer = VideoWriter(mp4File, 'MPEG-4');
    writer.FrameRate = frameRate;
    open(writer);
catch
    useMp4 = false;
end

fig = figure('Name', 'stewart_prealign_animation', 'Color', 'w');
for frameIndex = 1:frameCount
    clf(fig);
    hold on; grid on; axis equal;
    view(42, 24);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    q = sampleQ(:, frameIndex);
    kin = sgpIK(q, model);
    pTopLeg = q(1:3) + kin.rB;
    pTopPlatform = q(1:3) + kin.R * model.B;

    drawOBB(scene.collisionGeometry.rack.center, scene.collisionGeometry.rack.R, scene.collisionGeometry.rack.halfSize);
    plotClosedPolygon(model.A, 'k-', 1.5);
    plotClosedPolygon(pTopPlatform, 'b-', 1.5);
    scatter3(model.A(1, :), model.A(2, :), model.A(3, :), 28, 'k', 'filled');
    scatter3(pTopPlatform(1, :), pTopPlatform(2, :), pTopPlatform(3, :), 28, 'b', 'filled');

    for legIndex = 1:6
        plot3([model.A(1, legIndex), pTopLeg(1, legIndex)], ...
              [model.A(2, legIndex), pTopLeg(2, legIndex)], ...
              [model.A(3, legIndex), pTopLeg(3, legIndex)], ...
              'Color', [0.1, 0.35, 0.72], 'LineWidth', 1.8);
    end

    clearance = evaluateCollisionClearance(q, scene);
    bodyCount = scene.collision.bodySphereCount;
    scatter3(clearance.centersWorld(1, 1:bodyCount), clearance.centersWorld(2, 1:bodyCount), ...
        clearance.centersWorld(3, 1:bodyCount), 16, [0.85, 0.25, 0.15], 'filled');
    scatter3(clearance.centersWorld(1, bodyCount+1:end), clearance.centersWorld(2, bodyCount+1:end), ...
        clearance.centersWorld(3, bodyCount+1:end), 42, [0.1, 0.6, 0.25], 'filled');
    plot3(pathQ(1, 1:frameIndex), pathQ(2, 1:frameIndex), pathQ(3, 1:frameIndex), ...
        'm-', 'LineWidth', 1.3);
    drawPlatformAxes(q(1:3), kin.R, 0.12);

    allPoints = [model.A, pTopPlatform, clearance.centersWorld, scene.collisionGeometry.rack.center];
    margin = 0.25;
    xlim([min(allPoints(1, :))-margin, max(allPoints(1, :))+margin]);
    ylim([min(allPoints(2, :))-margin, max(allPoints(2, :))+margin]);
    zlim([min(allPoints(3, :))-margin, max(allPoints(3, :))+margin]);
    title(sprintf('t=%.2f s, min clearance=%.4f m, sigmaMin=%.3f, video=5.0 s', ...
        sampleTimes(frameIndex), sampleClearance(frameIndex), sampleSigma(frameIndex)));

    drawnow;
    frame = getframe(fig);
    if useMp4
        writeVideo(writer, frame);
    else
        [indexedImage, colorMap] = rgb2ind(frame2im(frame), 256);
        if frameIndex == 1
            imwrite(indexedImage, colorMap, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 1/frameRate);
        else
            imwrite(indexedImage, colorMap, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1/frameRate);
        end
    end
end

if useMp4
    close(writer);
    animationFile = mp4File;
else
    animationFile = gifFile;
end
fprintf('动画已保存：%s\n', animationFile);
end

function plotClosedPolygon(P, lineSpec, lineWidth)
order = [1:size(P, 2), 1];
plot3(P(1, order), P(2, order), P(3, order), lineSpec, 'LineWidth', lineWidth);
end

function drawPlatformAxes(origin, R, axisLength)
colors = eye(3);
for axisIndex = 1:3
    endpoint = origin + axisLength * R(:, axisIndex);
    plot3([origin(1), endpoint(1)], [origin(2), endpoint(2)], [origin(3), endpoint(3)], ...
        'Color', colors(axisIndex, :), 'LineWidth', 1.6);
end
end

function drawOBB(center, R, halfSize)
cornerLocal = [ ...
    -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
    -1 -1  1;  1 -1  1;  1  1  1; -1  1  1].' .* halfSize(:);
cornerWorld = center(:) + R * cornerLocal;
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', cornerWorld.', 'Faces', faces, ...
    'FaceColor', [0.75, 0.75, 0.78], 'FaceAlpha', 0.18, ...
    'EdgeColor', [0.35, 0.35, 0.35], 'LineWidth', 1.0);
end
