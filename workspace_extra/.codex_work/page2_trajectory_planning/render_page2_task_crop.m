function render_page2_task_crop()
% Render a compact physical task cue using the real Stewart geometry:
% a small start pose, a complete constrained target pose, and only upper-leg
% portions (the base and lower limbs stay outside the crop).

set(groot, 'defaultFigureVisible', 'off');
rootDir = 'D:/NPU/Matlab_code/stewart-simscape';
outDir = 'D:/NPU/Matlab_code/output/page2_trajectory_planning';
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'opt_minimal', 'core'));
if ~exist(outDir, 'dir'), mkdir(outDir); end

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
qStart = scene.q0;
qGoal = scene.qGoal;

fig = figure('Color', 'w', 'Units', 'pixels', 'Position', [100, 100, 760, 600]);
ax = axes('Parent', fig);
hold(ax, 'on'); axis(ax, 'equal'); axis(ax, 'off');
set(ax, 'Visible', 'off');

% The lighter starting platform is deliberately small; it only establishes
% the motion context.  The full dark-blue/orange target is retained.
[startPoints, startCuts] = drawUpperRobot(ax, qStart, model, [0.72 0.76 0.80], ...
    [0.59 0.66 0.72], 0.38, scene, [0.74 0.77 0.80], 0.42);
drawHood(ax, scene);                   % Actual roof and side skirts at the target.
[goalPoints, goalCuts] = drawUpperRobot(ax, qGoal, model, [0.06 0.24 0.43], ...
    [0.26 0.40 0.54], 0.92, scene, [0.93 0.55 0.23], 0.96);

% Wider geometric crop makes both the moving platform and the payload much
% smaller than the previous single-pose rendering while keeping the target
% and its constrained hood completely visible.
hoodPoints = [scene.hood.roof.center_S, scene.hood.leftSkirt.center_S, scene.hood.rightSkirt.center_S];
allPoints = [startPoints, startCuts, goalPoints, goalCuts, hoodPoints];
pad = [1.45; 1.35; 0.90];
xlim(ax, [min(allPoints(1,:))-pad(1), max(allPoints(1,:))+pad(1)]);
ylim(ax, [min(allPoints(2,:))-pad(2), max(allPoints(2,:))+pad(2)]);
zlim(ax, [min(allPoints(3,:))-pad(3), max(allPoints(3,:))+pad(3)]);
view(ax, 40, 19);
camlight(ax, 'headlight'); lighting(ax, 'gouraud');
drawnow;

exportgraphics(fig, fullfile(outDir, 'fig_task_matlab_crop.png'), 'Resolution', 260);
exportgraphics(fig, fullfile(outDir, 'fig_task_matlab_crop.pdf'), 'ContentType', 'vector');
exportgraphics(fig, fullfile(outDir, 'fig_task_matlab_crop.svg'), 'ContentType', 'vector');
savefig(fig, fullfile(outDir, 'fig_task_matlab_crop.fig'));
close(fig);
fprintf('TASK_CROP_OK\n');
end

function [upperPoints, legCuts] = drawUpperRobot(ax, q, model, platformColor, legColor, alpha, scene, cylinderColor, cylinderAlpha)
R = rpy2rotmZYX(q(4:6));
pTop = q(1:3) + R * model.B(:, model.legMap);
legCuts = zeros(3, 6);
for legIndex = 1:6
    pA = model.A(:, legIndex); pB = pTop(:, legIndex);
    pCut = 0.18 * pA + 0.82 * pB;     % retain only short top segments
    legCuts(:,legIndex) = pCut;
    plot3(ax, [pCut(1), pB(1)], [pCut(2), pB(2)], [pCut(3), pB(3)], '-', ...
        'Color', legColor, 'LineWidth', 1.45);
    plot3(ax, pB(1), pB(2), pB(3), 'o', 'MarkerSize', 3.0, ...
        'MarkerFaceColor', platformColor, 'MarkerEdgeColor', platformColor);
end
drawPlatformDisk(ax, q(1:3), R, model.rB, platformColor, alpha, platformColor);
drawCylinder(ax, q, scene, cylinderColor, cylinderAlpha);
[cylinderCenter, cylinderAxis] = cylinderPose(q, scene);
upperPoints = [pTop, cylinderCenter - 0.5*scene.objectCylinder.length*cylinderAxis, ...
    cylinderCenter + 0.5*scene.objectCylinder.length*cylinderAxis];
end

function drawPlatformDisk(ax, center, R, radius, faceColor, faceAlpha, edgeColor)
angles = linspace(0, 2*pi, 64);
circleLocal = radius * [cos(angles); sin(angles); zeros(1, numel(angles))];
circleWorld = center(:) + R * circleLocal;
patch(ax, circleWorld(1,:), circleWorld(2,:), circleWorld(3,:), faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 1.2);
end

function drawCylinder(ax, q, scene, faceColor, faceAlpha)
[center, axisVector] = cylinderPose(q, scene);
[X, Y, Z] = cylinder(scene.objectCylinder.radius, 32);
Z = (Z - 0.5) * scene.objectCylinder.length;
pts = [Z(:)'; X(:)'; Y(:)'];
basis = makeBasis(axisVector);
world = center + basis * pts;
Xw = reshape(world(1,:), size(X));
Yw = reshape(world(2,:), size(Y));
Zw = reshape(world(3,:), size(Z));
surf(ax, Xw, Yw, Zw, 'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
    'EdgeColor', faceColor * 0.78, 'LineWidth', 0.18);
end

function drawHood(ax, scene)
for index = 1:numel(scene.hood.obstacles)
    obstacle = scene.hood.obstacles(index);
    drawOrientedBox(ax, obstacle.center_S, obstacle.R_S, obstacle.size, ...
        [0.76 0.80 0.83], 0.14, [0.56 0.63 0.69]);
end
end

function drawOrientedBox(ax, center, R, sizeValue, faceColor, faceAlpha, edgeColor)
half = sizeValue(:) / 2;
local = half .* [-1 -1 -1 -1 1 1 1 1; -1 -1 1 1 -1 -1 1 1; -1 1 -1 1 -1 1 -1 1];
p = center(:) + R * local;
faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
patch(ax, 'Vertices', p.', 'Faces', faces, 'FaceColor', faceColor, ...
    'FaceAlpha', faceAlpha, 'EdgeColor', edgeColor, 'LineWidth', 0.8);
end

function [center, axisVector] = cylinderPose(q, scene)
R = rpy2rotmZYX(q(4:6));
center = q(1:3) + R * scene.objectCylinder.center_P;
axisVector = R * scene.objectCylinder.axis_P;
axisVector = axisVector / norm(axisVector);
end

function basis = makeBasis(axisVector)
tmp = [0; 0; 1];
if abs(dot(tmp, axisVector)) > 0.9, tmp = [0; 1; 0]; end
v2 = cross(tmp, axisVector); v2 = v2 / norm(v2);
v3 = cross(axisVector, v2);
basis = [axisVector, v2, v3];
end
