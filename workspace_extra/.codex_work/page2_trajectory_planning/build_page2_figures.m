function build_page2_figures()
% Reconstruct and replot Page 2 evidence from the accepted standard IHSID sample.
% The optimized node references are loaded from the archived accepted sample;
% the baseline is the deterministic two-phase quintic initial trajectory used
% by the same IHSID entry point, with identical scene, horizon, and grid.

set(groot, 'defaultFigureVisible', 'off');
rootDir = 'D:/NPU/Matlab_code/stewart-simscape';
outDir = 'D:/NPU/Matlab_code/output/page2_trajectory_planning';
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'opt_minimal', 'core'));
addpath(fullfile(rootDir, 'opt_minimal', 'ihsid'));
addpath(fullfile(rootDir, 'opt_minimal', 'validation'));
if ~exist(outDir, 'dir'), mkdir(outDir); end

sampleFile = fullfile(rootDir, 'opt_minimal', 'examples', ...
    'ihsid_40x20_limited_memory', 'simscape_references.mat');
S = load(sampleFile, 'refs', 'denseReport', 'solverResult', 'result');

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildTwoPhaseIHSDiscretization(scene, 40, 20);

% Baseline: exact deterministic IHSID initial guess used before IPOPT.
[z0, ~] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
initialData = evaluateImplicitIHSTrajectoryNumeric(z0, model, scene, disc);
initialTraj = rebuildComparisonTrajectory(initialData, scene, disc);

% Accepted optimized trajectory: original optimizer references at the 61 nodes.
opt = reconstructAcceptedNodeTrajectory(S.refs, model, scene, disc);

% Phase-aware normalized margins. 0 denotes an active boundary, positive safe.
[margins, risk, forceRate] = computeMargins(opt, model, scene, disc);
[~, ~, initialForceRate] = computeMarginsFromTrajectory(initialTraj, model, scene, disc);

% Smoothness uses all 6 leg samples at the common 61-node reference grid.
baseAccel = abs(initialTraj.Ldd(:));
optAccel = abs(opt.Ldd(:));
smooth = makeSmoothStats(baseAccel, optAccel);

% Standard accepted dense metrics are authoritative for cards; re-evaluation
% at nodes supports the heatmap and risk-colored 3D visual.
metrics = struct();
metrics.minClearance = S.denseReport.minClearance;
metrics.minSigma = S.denseReport.minSigmaMin;
metrics.maxCond = S.denseReport.maxCondJ;
metrics.maxLegSpeed = S.denseReport.maxAbsLd;
metrics.maxLegAccel = S.denseReport.maxAbsLdd;
metrics.maxForce = S.denseReport.maxAbsForce;
metrics.maxForceRateNode = max(forceRate(:));
metrics.maxForceRateInitialNode = max(initialForceRate(:));
metrics.minRiskMargin = min(risk);
metrics.smooth = smooth;
metrics.duration = disc.duration;
metrics.samples = numel(opt.t);
metrics.accepted = S.result.engineeringTrajectoryPassed && S.solverResult.success;

% Four requested independent panels.
drawMainTrajectory(opt, risk, model, scene, disc, metrics, outDir);
drawPipeline(outDir);
drawConstraintHeatmap(margins, opt.t, disc, metrics, outDir);
drawSmoothnessEcdf(baseAccel, optAccel, smooth, outDir);
drawSmoothnessCompact(baseAccel, optAccel, smooth, outDir);

save(fullfile(outDir, 'page2_replot_data.mat'), 'metrics', 'margins', 'risk', ...
    'forceRate', 'initialForceRate', 'opt', 'initialTraj', 'model', 'scene', 'disc', '-v7');
writeMetricsCsv(metrics, model, scene, disc, outDir);
writeNotes(metrics, model, scene, disc, sampleFile, outDir);
fprintf('PAGE2_FIGURES_OK\n');
fprintf('smooth mean/rms/p95/peak improvements = %.3f / %.3f / %.3f / %.3f %%\n', ...
    smooth.improvement.mean, smooth.improvement.rms, smooth.improvement.p95, smooth.improvement.peak);
end

function opt = reconstructAcceptedNodeTrajectory(refs, model, scene, disc)
opt = struct();
opt.t = refs.t(:).';
opt.Q = refs.q;
opt.V = refs.qd;
opt.F = refs.Fleg;
n = numel(opt.t);
opt.L = zeros(6, n); opt.Ld = zeros(6, n); opt.Ldd = zeros(6, n);
opt.sigma = zeros(1, n); opt.cond = zeros(1, n);
opt.clearance = zeros(3, n); opt.qdd = zeros(6, n);
for k = 1:n
    point = evaluatePathConstraintsAtPoint([opt.Q(:, k); opt.V(:, k)], opt.F(:, k), model, scene);
    opt.L(:, k) = point.L;
    opt.Ld(:, k) = point.Ld;
    opt.Ldd(:, k) = point.Ldd;
    opt.sigma(k) = point.sigmaMin;
    opt.cond(k) = point.condJ;
    opt.clearance(:, k) = point.collisionDistances;
    opt.qdd(:, k) = point.qdd;
end
opt.refsDescription = refs.description;
opt.stageBoundaryNode = refs.stageBoundaryNode;
end

function [M, risk, rateAtNodes] = computeMargins(opt, model, scene, disc)
n = numel(opt.t);
rate = abs(diff(opt.F, 1, 2)) ./ disc.h;
rateAtNodes = zeros(6, n);
rateAtNodes(:, 1) = rate(:, 1);
rateAtNodes(:, end) = rate(:, end);
rateAtNodes(:, 2:end-1) = 0.5 * (rate(:, 1:end-1) + rate(:, 2:end));

collision = zeros(1, n);
for k = 1:n
    if opt.t(k) <= disc.durationApproach + 1e-10
        roofRequired = scene.collision.stage1ConstraintDistance;
    else
        roofRequired = scene.collision.finalGap;
    end
    roofMargin = (opt.clearance(1, k) - roofRequired) / max(roofRequired, 1e-6);
    sideMargin = min((opt.clearance(2:3, k) - scene.collision.safeDistance) / scene.collision.safeDistance);
    collision(k) = min(roofMargin, sideMargin);
end
sigmaMargin = (opt.sigma - model.singularity.sigmaMinSafe) / model.singularity.sigmaMinSafe;
condMargin = (model.singularity.condWarning - opt.cond) / model.singularity.condWarning;
lenMargin = min([(opt.L - model.lmin) ./ (model.lmax - model.lmin); ...
                 (model.lmax - opt.L) ./ (model.lmax - model.lmin)], [], 1);
speedMargin = min((model.actuator.ldotMax - abs(opt.Ld)) ./ model.actuator.ldotMax, [], 1);
accMargin = min((model.actuator.lddotMax - abs(opt.Ldd)) ./ model.actuator.lddotMax, [], 1);
forceMargin = min([(opt.F - model.actuator.forceMin) ./ (model.actuator.forceMax - model.actuator.forceMin); ...
                   (model.actuator.forceMax - opt.F) ./ (model.actuator.forceMax - model.actuator.forceMin)], [], 1);
forceRateMargin = 1 - max(rateAtNodes, [], 1) / model.objective.forceRateScale;
M = [collision; sigmaMargin; condMargin; lenMargin; speedMargin; accMargin; forceMargin; forceRateMargin];
% The risk color encodes closest hard bound. Force rate is deliberately excluded:
% it is an IHSID objective scale, not an inequality constraint in this run.
risk = min(M(1:7, :), [], 1);
end

function [M, risk, rateAtNodes] = computeMarginsFromTrajectory(traj, model, scene, disc)
tmp = struct('t', traj.t, 'Q', traj.Q, 'V', traj.V, 'F', traj.Unode, ...
    'L', traj.L, 'Ld', traj.Ld, 'Ldd', traj.Ldd, 'sigma', traj.sigmaMin, ...
    'cond', traj.condJ, 'clearance', traj.collisionDistances);
[M, risk, rateAtNodes] = computeMargins(tmp, model, scene, disc);
end

function smooth = makeSmoothStats(base, opt)
smooth = struct();
smooth.base.mean = mean(base); smooth.opt.mean = mean(opt);
smooth.base.rms = sqrt(mean(base.^2)); smooth.opt.rms = sqrt(mean(opt.^2));
smooth.base.p95 = prctile(base, 95); smooth.opt.p95 = prctile(opt, 95);
smooth.base.peak = max(base); smooth.opt.peak = max(opt);
names = {'mean','rms','p95','peak'};
for i = 1:numel(names)
    name = names{i};
    smooth.improvement.(name) = 100 * (1 - smooth.opt.(name) / smooth.base.(name));
end
end

function drawMainTrajectory(opt, risk, model, scene, disc, metrics, outDir)
fig = figure('Color', 'w', 'Position', [80 80 2400 1760], 'Renderer', 'opengl');
ax = axes(fig, 'Position', [0.06 0.08 0.86 0.86]); hold(ax, 'on');
drawScene(ax, model, scene);
drawRiskTrajectory(ax, opt.Q(1:3, :), risk);

% Key task states and risk locations.
plot3(ax, opt.Q(1,1), opt.Q(2,1), opt.Q(3,1), 'o', 'MarkerSize', 11, ...
    'MarkerFaceColor', [0.10 0.42 0.70], 'MarkerEdgeColor', 'w', 'LineWidth', 1.4);
waypoint = disc.waypointNodeIndex;
plot3(ax, opt.Q(1,waypoint), opt.Q(2,waypoint), opt.Q(3,waypoint), '^', 'MarkerSize', 11, ...
    'MarkerFaceColor', [0.20 0.25 0.31], 'MarkerEdgeColor', 'w', 'LineWidth', 1.2);
plot3(ax, opt.Q(1,end), opt.Q(2,end), opt.Q(3,end), 'p', 'MarkerSize', 14, ...
    'MarkerFaceColor', [0.88 0.32 0.14], 'MarkerEdgeColor', 'w', 'LineWidth', 1.4);
[~, iRisk] = min(risk);
plot3(ax, opt.Q(1,iRisk), opt.Q(2,iRisk), opt.Q(3,iRisk), 'o', 'MarkerSize', 20, ...
    'MarkerFaceColor', 'none', 'MarkerEdgeColor', [0.88 0.32 0.14], 'LineWidth', 2.2);

addText3(ax, opt.Q(1,1), opt.Q(2,1), opt.Q(3,1), '  起点');
addText3(ax, opt.Q(1,waypoint), opt.Q(2,waypoint), opt.Q(3,waypoint), '  接近/对准');
addText3(ax, opt.Q(1,end), opt.Q(2,end), opt.Q(3,end), '  挂装终点');

drawPlatformDisk(ax, opt.Q(1:3,1), rpy2rotmZYX(opt.Q(4:6,1)), model.rB, [0.12 0.41 0.68], 0.14);
drawPlatformDisk(ax, opt.Q(1:3,waypoint), rpy2rotmZYX(opt.Q(4:6,waypoint)), model.rB, [0.24 0.33 0.42], 0.18);
drawPlatformDisk(ax, opt.Q(1:3,end), rpy2rotmZYX(opt.Q(4:6,end)), model.rB, [0.88 0.32 0.14], 0.22);
drawCylinder(ax, opt.Q(:,end), scene, [0.23 0.53 0.72], 0.35);

axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'off');
xlabel(ax, 'X / m', 'FontName', 'Microsoft YaHei', 'FontSize', 15);
ylabel(ax, 'Y / m', 'FontName', 'Microsoft YaHei', 'FontSize', 15);
zlabel(ax, 'Z / m', 'FontName', 'Microsoft YaHei', 'FontSize', 15);
set(ax, 'FontName', 'Aptos', 'FontSize', 13, 'GridAlpha', 0.12, 'LineWidth', 1);
view(ax, 46, 24); camproj(ax, 'perspective');
title(ax, '挂装场景中的优化轨迹与关键风险位置', 'FontName', 'Microsoft YaHei', ...
    'FontSize', 24, 'FontWeight', 'bold', 'Color', [0.08 0.13 0.20]);
subtitle(ax, '轨迹颜色 = 最近硬约束的归一化安全裕度（蓝：裕度充足；橙：接近边界）', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 13, 'Color', [0.32 0.38 0.45]);

% Local cross-section inset: roof-bottom to cylinder-top gap at terminal.
inset = axes(fig, 'Position', [0.64 0.13 0.28 0.24], 'Color', [1 1 1]); hold(inset, 'on');
drawClearanceInset(inset, opt.Q(:,end), scene, metrics.minClearance);
annotation(fig, 'textbox', [0.64 0.37 0.27 0.035], 'String', '局部：终端设计间隙', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.12 0.18 0.24], ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left');
exportPanel(fig, outDir, 'fig_main_3d_trajectory');
close(fig);
end

function drawScene(ax, model, scene)
drawPlatformDisk(ax, [0;0;0], eye(3), model.rA, [0.27 0.32 0.38], 0.14);
drawBox(ax, scene.box.center_S, scene.box.R_S, scene.box.halfSize, [0.70 0.74 0.78], 0.13, [0.42 0.46 0.51]);
for i = 1:numel(scene.hood.obstacles)
    drawBox(ax, scene.hood.obstacles(i).center_S, scene.hood.obstacles(i).R_S, ...
        scene.hood.obstacles(i).halfSize, [0.50 0.54 0.58], 0.22, [0.30 0.33 0.37]);
end
% Light guide plane makes the constrained entry direction legible.
R = scene.box.R_S;
cornersB = [-0.56 -0.05 -0.24 -0.56; -0.24 -0.24 -0.24 -0.24; -0.27 -0.27 -0.27 -0.27];
corners = scene.box.center_S + R * cornersB;
patch(ax, corners(1,:), corners(2,:), corners(3,:), [0.72 0.80 0.88], ...
    'FaceAlpha', 0.08, 'EdgeColor', 'none');
end

function drawRiskTrajectory(ax, Q, risk)
low = [0.10 0.40 0.67]; high = [0.92 0.31 0.12];
r = max(0, min(1, 1-risk));
for i = 1:size(Q,2)-1
    c = low*(1-r(i)) + high*r(i);
    plot3(ax, Q(1,i:i+1), Q(2,i:i+1), Q(3,i:i+1), '-', 'Color', c, 'LineWidth', 5.5);
end
scatter3(ax, Q(1,:), Q(2,:), Q(3,:), 20, r, 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 0.3);
end

function drawClearanceInset(ax, q, scene, minGap)
R = rpy2rotmZYX(q(4:6));
cS = q(1:3) + R * scene.objectCylinder.center_P;
cB = scene.box.R_S.' * (cS - scene.box.center_S);
roofBottom = scene.hood.roof.center_B(3) - scene.hood.roof.halfSize(3);
rectangle(ax, 'Position', [-0.52 roofBottom 1.04 0.06], 'FaceColor', [0.42 0.46 0.51], 'EdgeColor', 'none');
rectangle(ax, 'Position', [cB(1)-scene.objectCylinder.length/2 cB(3)-scene.objectCylinder.radius, ...
    scene.objectCylinder.length 2*scene.objectCylinder.radius], 'Curvature', [0.15 0.15], ...
    'FaceColor', [0.23 0.53 0.72], 'FaceAlpha', 0.48, 'EdgeColor', [0.10 0.40 0.62], 'LineWidth', 1.1);
xGap = 0.42;
plot(ax, [xGap xGap], [cB(3)+scene.objectCylinder.radius roofBottom], '-', 'Color', [0.88 0.32 0.14], 'LineWidth', 2);
plot(ax, [xGap-0.035 xGap+0.035], [cB(3)+scene.objectCylinder.radius cB(3)+scene.objectCylinder.radius], '-', 'Color', [0.88 0.32 0.14], 'LineWidth', 2);
plot(ax, [xGap-0.035 xGap+0.035], [roofBottom roofBottom], '-', 'Color', [0.88 0.32 0.14], 'LineWidth', 2);
text(ax, -0.50, roofBottom+0.075, sprintf('最小间隙 %.1f mm', minGap*1e3), 'FontName', 'Microsoft YaHei', ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.75 0.23 0.10]);
text(ax, -0.50, roofBottom+0.025, '罩体下缘', 'FontName', 'Microsoft YaHei', 'FontSize', 10, 'Color', [0.25 0.30 0.35]);
axis(ax, 'equal'); xlim(ax, [-0.57 0.57]); ylim(ax, [cB(3)-0.20 roofBottom+0.15]); axis(ax, 'off');
end

function drawPipeline(outDir)
fig = figure('Color', 'w', 'Position', [100 100 2200 560]);
ax = axes(fig, 'Position', [0 0 1 1]); axis(ax, [0 1 0 1]); axis(ax, 'off'); hold(ax, 'on');
blue = [0.12 0.39 0.65]; ink = [0.12 0.17 0.23]; soft = [0.94 0.96 0.98]; orange = [0.88 0.33 0.14];
xs = [0.02 0.275 0.53 0.775]; w = 0.20; y = 0.17; h = 0.64;
for i = 1:4
    if i == 3
        fc = [0.92 0.95 0.98]; ec = blue;
    else
        fc = soft; ec = [0.72 0.77 0.82];
    end
    rectangle(ax, 'Position', [xs(i) y w h], 'Curvature', [0.08 0.08], ...
        'FaceColor', fc, 'EdgeColor', ec, 'LineWidth', 2);
    if i < 4
        annotation(fig, 'arrow', [xs(i)+w+0.008 xs(i+1)-0.012], [0.49 0.49], ...
            'Color', [0.50 0.56 0.62], 'LineWidth', 1.7, 'HeadLength', 7, 'HeadWidth', 7);
    end
end
% Small schematic icons.
drawTaskIcon(ax, xs(1)+0.042, 0.61, blue); drawConstraintIcon(ax, xs(2)+0.052, 0.61, orange);
drawOptimizerIcon(ax, xs(3)+0.065, 0.61, blue); drawOutputIcon(ax, xs(4)+0.060, 0.61, blue);
labels = {'任务输入','工程约束','IHSID 优化','参考输出'};
lines = {{'起点 / 目标','环境 / 结构'}, {'碰撞 · 奇异','行程 · 速度 · 力'}, {'连续轨迹','多约束协调'}, {'位姿 / 速度','支链 / 前馈力'}};
for i = 1:4
    text(ax, xs(i)+w/2, 0.47, labels{i}, 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', ...
        'FontSize', 18, 'FontWeight', 'bold', 'Color', ink);
    text(ax, xs(i)+w/2, 0.34, lines{i}{1}, 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', ...
        'FontSize', 12, 'Color', [0.32 0.38 0.44]);
    text(ax, xs(i)+w/2, 0.27, lines{i}{2}, 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', ...
        'FontSize', 12, 'Color', [0.32 0.38 0.44]);
end
text(ax, 0.02, 0.94, '轨迹规划层的信息流', 'FontName', 'Microsoft YaHei', 'FontSize', 22, ...
    'FontWeight', 'bold', 'Color', ink, 'VerticalAlignment', 'top');
exportPanel(fig, outDir, 'fig_pipeline_icons');
close(fig);
end

function drawTaskIcon(ax, x, y, color)
plot(ax, [x x+0.10], [y y+0.04], '-', 'Color', color, 'LineWidth', 2.5);
plot(ax, x, y, 'o', 'MarkerSize', 9, 'MarkerFaceColor', color, 'MarkerEdgeColor', 'w');
plot(ax, x+0.10, y+0.04, 'p', 'MarkerSize', 12, 'MarkerFaceColor', color, 'MarkerEdgeColor', 'w');
end
function drawConstraintIcon(ax, x, y, color)
rectangle(ax, 'Position', [x y 0.09 0.075], 'Curvature', [0.25 0.25], 'FaceColor', 'w', 'EdgeColor', color, 'LineWidth', 2);
plot(ax, [x+0.018 x+0.072], [y+0.037 y+0.037], '-', 'Color', color, 'LineWidth', 2);
end
function drawOptimizerIcon(ax, x, y, color)
theta = linspace(0, 2*pi, 30); plot(ax, x+0.038+0.036*cos(theta), y+0.036+0.036*sin(theta), '-', 'Color', color, 'LineWidth', 2);
plot(ax, [x+0.038 x+0.068], [y+0.036 y+0.060], '-', 'Color', color, 'LineWidth', 2);
plot(ax, [x+0.038 x+0.015], [y+0.036 y+0.012], '-', 'Color', color, 'LineWidth', 2);
end
function drawOutputIcon(ax, x, y, color)
for i = 0:2
    plot(ax, [x x+0.09], [y+0.01+0.03*i y+0.01+0.03*i], '-', 'Color', color, 'LineWidth', 2.2);
end
end

function drawConstraintHeatmap(M, t, disc, metrics, outDir)
fig = figure('Color', 'w', 'Position', [100 100 2200 1020]);
ax = axes(fig, 'Position', [0.11 0.18 0.74 0.66]);
imagesc(ax, t/disc.duration*100, 1:8, M); set(ax, 'YDir', 'reverse');
colormap(ax, marginMap()); caxis(ax, [-0.25 1.00]);
labels = {'碰撞间隙','σ_{min}(J)','κ(J)','支链长度','支链速度','支链加速度','驱动力','力变化率*'};
set(ax, 'YTick', 1:8, 'YTickLabel', labels, 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'TickLength', [0 0]);
xlabel(ax, '任务进度 / %', 'FontName', 'Microsoft YaHei', 'FontSize', 16);
xline(ax, disc.durationApproach/disc.duration*100, '-', 'Color', [0.20 0.25 0.30], 'LineWidth', 1.4);
text(ax, 32, 0.35, '接近阶段', 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', 'FontSize', 14, 'Color', [0.28 0.35 0.42]);
text(ax, 83, 0.35, '对准 / 挂装', 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', 'FontSize', 14, 'Color', [0.28 0.35 0.42]);
title(ax, '轨迹全过程的约束安全裕度分布', 'FontName', 'Microsoft YaHei', 'FontSize', 24, 'FontWeight', 'bold', 'Color', [0.08 0.13 0.20]);
cb = colorbar(ax, 'Position', [0.89 0.18 0.025 0.66]);
cb.Label.String = '归一化安全裕度'; cb.Label.FontName = 'Microsoft YaHei'; cb.Label.FontSize = 14;
cb.Ticks = [-0.25 0 0.5 1]; cb.TickLabels = {'<0 违规','0 边界','0.5','≥1 安全'}; cb.FontSize = 12;
annotation(fig, 'textbox', [0.11 0.06 0.76 0.055], 'String', ...
    sprintf('硬约束均满足；终端按设计贴近到 %.1f mm。* 力变化率按 IHSID 代价归一化尺度 1000 N/s 展示。', metrics.minClearance*1e3), ...
    'EdgeColor', 'none', 'FontName', 'Microsoft YaHei', 'FontSize', 14, 'Color', [0.28 0.34 0.41]);
exportPanel(fig, outDir, 'fig_constraint_heatmap');
close(fig);
end

function cmap = marginMap()
cmap = [linspace(0.82,0.95,32)', linspace(0.20,0.72,32)', linspace(0.09,0.42,32)'; ...
        linspace(0.96,0.93,32)', linspace(0.78,0.96,32)', linspace(0.66,0.98,32)'; ...
        linspace(0.87,0.12,64)', linspace(0.94,0.39,64)', linspace(0.98,0.66,64)'];
end

function drawSmoothnessEcdf(base, opt, smooth, outDir)
fig = figure('Color', 'w', 'Position', [100 100 1900 980]);
ax = axes(fig, 'Position', [0.12 0.18 0.78 0.67]); hold(ax, 'on');
[xb, yb] = simpleEcdf(base); [xo, yo] = simpleEcdf(opt);
plot(ax, xb, yb, '-', 'Color', [0.90 0.34 0.15], 'LineWidth', 3.2);
plot(ax, xo, yo, '-', 'Color', [0.10 0.40 0.67], 'LineWidth', 3.4);
grid(ax, 'on'); box(ax, 'off'); set(ax, 'FontName', 'Aptos', 'FontSize', 15, 'GridAlpha', 0.13);
xlabel(ax, '所有支链采样点的 |加速度| / m·s^{-2}', 'FontName', 'Microsoft YaHei', 'FontSize', 16);
ylabel(ax, '累计样本比例', 'FontName', 'Microsoft YaHei', 'FontSize', 16);
title(ax, '优化前后轨迹平滑性对比', 'FontName', 'Microsoft YaHei', 'FontSize', 24, 'FontWeight', 'bold', 'Color', [0.08 0.13 0.20]);
legend(ax, {'原始五次时间律','IHSID 优化'}, 'Location', 'southeast', 'FontName', 'Microsoft YaHei', 'FontSize', 14, 'Box', 'off');
text(ax, 0.02, 0.98, sprintf('平均值  %.1f%%\nRMS    %.1f%%\nP95    %.1f%%\n峰值    %.1f%%', ...
    smooth.improvement.mean, smooth.improvement.rms, smooth.improvement.p95, smooth.improvement.peak), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'FontName', 'Microsoft YaHei', 'FontSize', 15, ...
    'FontWeight', 'bold', 'Color', [0.08 0.24 0.40], 'BackgroundColor', [0.94 0.97 0.99], 'Margin', 8);
annotation(fig, 'textbox', [0.12 0.07 0.76 0.045], 'String', ...
    '口径：同一 7.5 s、61 节点网格内 6 条支链的绝对加速度（n = 366）。曲线整体左移代表执行器加速度负担下降。', ...
    'EdgeColor', 'none', 'FontName', 'Microsoft YaHei', 'FontSize', 13, 'Color', [0.30 0.36 0.43]);
exportPanel(fig, outDir, 'fig_smoothness_ecdf');
close(fig);
end

function drawSmoothnessCompact(base, opt, smooth, outDir)
% Wide, crop-safe companion for the right-bottom panel of the final slide.
fig = figure('Color', 'w', 'Position', [100 100 2800 660]);
ax = axes(fig, 'Position', [0.06 0.23 0.64 0.66]); hold(ax, 'on');
[xb, yb] = simpleEcdf(base); [xo, yo] = simpleEcdf(opt);
plot(ax, xb, yb, '-', 'Color', [0.90 0.34 0.15], 'LineWidth', 3.0);
plot(ax, xo, yo, '-', 'Color', [0.10 0.40 0.67], 'LineWidth', 3.0);
grid(ax, 'on'); box(ax, 'off'); set(ax, 'FontName', 'Aptos', 'FontSize', 14, 'GridAlpha', 0.13);
xlabel(ax, '|支链加速度| / m·s^{-2}', 'FontName', 'Microsoft YaHei', 'FontSize', 14);
ylabel(ax, '累计比例', 'FontName', 'Microsoft YaHei', 'FontSize', 14);
legend(ax, {'原始','IHSID'}, 'Location', 'southeast', 'FontName', 'Microsoft YaHei', 'FontSize', 12, 'Box', 'off');
annotation(fig, 'textbox', [0.74 0.53 0.22 0.28], 'String', sprintf('分布整体左移\nMean  ↓ %.1f%%\nRMS   ↓ %.1f%%\nP95    ↓ %.1f%%', ...
    smooth.improvement.mean, smooth.improvement.rms, smooth.improvement.p95), 'EdgeColor', 'none', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 17, 'FontWeight', 'bold', 'Color', [0.08 0.25 0.42], 'VerticalAlignment', 'middle');
annotation(fig, 'textbox', [0.74 0.20 0.22 0.13], 'String', '同一 61 节点 × 6 支链\n绝对加速度 ECDF', ...
    'EdgeColor', 'none', 'FontName', 'Microsoft YaHei', 'FontSize', 12, 'Color', [0.34 0.41 0.49]);
exportgraphics(fig, fullfile(outDir, 'fig_smoothness_ecdf_compact.png'), 'Resolution', 260);
close(fig);
end

function [x, y] = simpleEcdf(values)
x = sort(values(:)); y = (1:numel(x)).' / numel(x);
end

function exportPanel(fig, outDir, baseName)
pngFile = fullfile(outDir, [baseName '.png']);
pdfFile = fullfile(outDir, [baseName '.pdf']);
svgFile = fullfile(outDir, [baseName '.svg']);
exportgraphics(fig, pngFile, 'Resolution', 260);
exportgraphics(fig, pdfFile, 'ContentType', 'vector');
exportgraphics(fig, svgFile, 'ContentType', 'vector');
end

function writeMetricsCsv(metrics, model, scene, disc, outDir)
metric = {'min_collision_clearance';'min_jacobian_sigma';'max_jacobian_condition';'max_leg_speed';'max_leg_acceleration';'max_actuator_force';'max_node_force_rate';'baseline_abs_leg_accel_mean';'optimized_abs_leg_accel_mean';'baseline_abs_leg_accel_rms';'optimized_abs_leg_accel_rms';'baseline_abs_leg_accel_p95';'optimized_abs_leg_accel_p95';'baseline_abs_leg_accel_peak';'optimized_abs_leg_accel_peak';'accel_mean_improvement';'accel_rms_improvement';'accel_p95_improvement';'accel_peak_improvement';'solver_and_engineering_acceptance'};
value = [metrics.minClearance;metrics.minSigma;metrics.maxCond;metrics.maxLegSpeed;metrics.maxLegAccel;metrics.maxForce;metrics.maxForceRateNode;metrics.smooth.base.mean;metrics.smooth.opt.mean;metrics.smooth.base.rms;metrics.smooth.opt.rms;metrics.smooth.base.p95;metrics.smooth.opt.p95;metrics.smooth.base.peak;metrics.smooth.opt.peak;metrics.smooth.improvement.mean;metrics.smooth.improvement.rms;metrics.smooth.improvement.p95;metrics.smooth.improvement.peak;double(metrics.accepted)];
unit = {'m';'dimensionless';'dimensionless';'m/s';'m/s^2';'N';'N/s';'m/s^2';'m/s^2';'m/s^2';'m/s^2';'m/s^2';'m/s^2';'m/s^2';'m/s^2';'%';'%';'%';'%';'boolean'};
source = {'accepted denseReport';'accepted denseReport';'accepted denseReport';'accepted denseReport';'accepted denseReport';'accepted denseReport';'accepted refs node reconstruction';'reconstructed baseline node grid';'accepted refs node reconstruction';'reconstructed baseline node grid';'accepted refs node reconstruction';'reconstructed baseline node grid';'accepted refs node reconstruction';'reconstructed baseline node grid';'accepted refs node reconstruction';'derived from node-grid ECDF sample';'derived from node-grid ECDF sample';'derived from node-grid ECDF sample';'derived from node-grid ECDF sample';'solverResult and result'};
definition = {'minimum cylinder-to-hood clearance, dense post-check';'minimum normalized spatial Jacobian singular value, dense post-check';'maximum normalized spatial Jacobian condition number, dense post-check';'maximum abs leg speed, dense post-check';'maximum abs leg acceleration, dense post-check';'maximum abs leg force, dense post-check';'node finite-difference abs force-rate maximum; objective scale is 1000 N/s';'mean of abs Ldd across 6 x 61 baseline node samples';'mean of abs Ldd across 6 x 61 optimized node samples reconstructed from accepted refs';'RMS of abs Ldd across 6 x 61 baseline node samples';'RMS of abs Ldd across 6 x 61 optimized node samples reconstructed from accepted refs';'95th percentile of abs Ldd across 6 x 61 baseline node samples';'95th percentile of abs Ldd across 6 x 61 optimized node samples reconstructed from accepted refs';'maximum abs Ldd across 6 x 61 baseline node samples';'maximum abs Ldd across 6 x 61 optimized node samples reconstructed from accepted refs';'100*(1-optimized/baseline)';'100*(1-optimized/baseline)';'100*(1-optimized/baseline)';'100*(1-optimized/baseline)';'both values true in accepted sample'};
T = table(metric, value, unit, source, definition);
writetable(T, fullfile(outDir, 'page2_metrics_summary.csv'), 'Encoding', 'UTF-8');
end

function writeNotes(metrics, model, scene, disc, sampleFile, outDir)
file = fullfile(outDir, 'page2_notes.md');
fid = fopen(file, 'w'); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# 第 2 页：轨迹规划层 — 数据与口径说明\n\n');
fprintf(fid, '## 数据来源\n\n');
fprintf(fid, '- 优化轨迹：`%s` 中已通过求解器与工程后验双重验收的 `refs`、`denseReport`、`solverResult` 与 `result`。\n', strrep(sampleFile, '\\', '/'));
fprintf(fid, '- 优化器设置：standard IHSID，40+20 区间，7.5 s，总节点数 61；场景与约束由 `buildOptModelCustom.m` 与 `buildCylinderBoxTransferScene.m` 重建。\n');
fprintf(fid, '- 原始（未优化）轨迹：同一 IHSID 主入口在 IPOPT 求解前调用 `buildInitialGuessTwoPhaseIHSImplicit.m` 生成的两阶段五次时间律初值；起终点、环境、网格与时域均相同。\n\n');
fprintf(fid, '## 子图口径\n\n');
fprintf(fid, '- `fig_main_3d_trajectory`：使用已验收 `refs.q` 的 61 节点位姿重绘；轨迹颜色为 7 项硬约束中最小归一化安全裕度。局部窗显示终端设计间隙 %.1f mm。\n', metrics.minClearance*1e3);
fprintf(fid, '- `fig_constraint_heatmap`：在接受轨迹的 61 节点重算各物理量。碰撞采用阶段化边界：接近阶段屋顶 %.1f mm、送入阶段终端 %.1f mm、侧裙 %.1f mm；其余硬约束取模型阈值。力变化率采用 %.0f N/s 的 IHSID 代价归一化尺度，仅作执行负担指示，并非本次 IHSID 的硬不等式。\n', scene.collision.stage1ConstraintDistance*1e3, scene.collision.finalGap*1e3, scene.collision.safeDistance*1e3, model.objective.forceRateScale);
fprintf(fid, '- `fig_smoothness_ecdf`：比较同一 61 节点网格内全部 6 条支链的绝对加速度，共 366 个样本/轨迹。优化轨迹的节点加速度由已保存 `refs.q`、`refs.qd`、`refs.Fleg` 在同一动力学模型中重算；基线使用原始初值中的解析节点加速度。\n');
fprintf(fid, '- `fig_pipeline_icons`：依据项目实际接口整理：任务与环境输入，经碰撞/奇异/执行器约束进入 IHSID，输出位姿、速度、支链参考与力前馈。\n\n');
fprintf(fid, '## 卡片数值（接受轨迹的密集后验）\n\n');
fprintf(fid, '- 最小碰撞间隙：%.3f m；最小 σ_min：%.6f；最大 κ(J)：%.6f。\n', metrics.minClearance, metrics.minSigma, metrics.maxCond);
fprintf(fid, '- 最大支链速度：%.6f m/s；最大支链加速度：%.6f m/s²；最大驱动力：%.3f N。\n', metrics.maxLegSpeed, metrics.maxLegAccel, metrics.maxForce);
fprintf(fid, '- 平滑性改善（节点口径）：Mean %.2f%%、RMS %.2f%%、P95 %.2f%%、Peak %.2f%%。\n', metrics.smooth.improvement.mean, metrics.smooth.improvement.rms, metrics.smooth.improvement.p95, metrics.smooth.improvement.peak);
fprintf(fid, '\n## 重要边界\n\n');
fprintf(fid, '- 终端 %.1f mm 间隙是本任务“贴近挂装”的设计目标，而非将全程统一维持 %.1f mm 的安全距离。接受轨迹在送入阶段同时保留侧向安全裕度。\n', scene.collision.finalGap*1e3, scene.collision.safeDistance*1e3);
fprintf(fid, '- 页面卡片中的碰撞、奇异、速度与加速度数值采用保存的密集后验，因此与节点热图的局部读数可能略有差异；卡片数值优先。\n');
fprintf(fid, '- 该页不将 IHSID 的力变化率目标项描述为硬约束；相关行仅用于解释执行器负担。\n');
end

function drawPlatformDisk(ax, center, R, radius, faceColor, alpha)
theta = linspace(0,2*pi,56); pts = center(:) + R * (radius*[cos(theta); sin(theta); zeros(size(theta))]);
patch(ax, pts(1,:), pts(2,:), pts(3,:), faceColor, 'FaceAlpha', alpha, 'EdgeColor', faceColor*0.7, 'LineWidth', 1.0);
end

function drawBox(ax, center, R, halfSize, faceColor, alpha, edgeColor)
local = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1]' .* halfSize;
corners = center(:) + R * local; faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
patch(ax, 'Vertices', corners.', 'Faces', faces, 'FaceColor', faceColor, 'FaceAlpha', alpha, 'EdgeColor', edgeColor, 'LineWidth', 0.8);
end

function drawCylinder(ax, q, scene, faceColor, alpha)
R = rpy2rotmZYX(q(4:6)); c = q(1:3) + R*scene.objectCylinder.center_P; a = R*scene.objectCylinder.axis_P; a = a/norm(a);
[X,Y,Z] = cylinder(scene.objectCylinder.radius, 28); Z = (Z-0.5)*scene.objectCylinder.length;
basis = makeBasis(a); world = c + basis*[Z(:)';X(:)';Y(:)'];
surf(ax, reshape(world(1,:),size(X)), reshape(world(2,:),size(Y)), reshape(world(3,:),size(Z)), ...
    'FaceColor', faceColor, 'FaceAlpha', alpha, 'EdgeColor', 'none');
end

function basis = makeBasis(a)
tmp = [0;0;1]; if abs(dot(tmp,a))>0.9, tmp=[0;1;0]; end
b = cross(tmp,a); b = b/norm(b); c = cross(a,b); basis = [a b c];
end

function addText3(ax,x,y,z,txt)
text(ax,x,y,z,txt,'FontName','Microsoft YaHei','FontSize',13,'FontWeight','bold','Color',[0.10 0.15 0.20], 'BackgroundColor','w','Margin',2);
end
