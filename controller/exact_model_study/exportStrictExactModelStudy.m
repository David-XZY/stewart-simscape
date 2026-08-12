function paths = exportStrictExactModelStudy(runs, config, outputDir, resolution)
% exportStrictExactModelStudy - Export thesis-ready exact-model figures.
arguments
    runs struct
    config struct
    outputDir {mustBeTextScalar}
    resolution (1,1) double {mustBePositive, mustBeFinite} = 220
end
outputDir = char(outputDir);
colors = lines(6);
paths = strings(0, 1);

paths(end+1) = exportTrackingComparison(runs, 1:3, colors, ...
    fullfile(outputDir, 'exact_model_ablation_tracking.png'), ...
    'Exact nonlinear model: controller ablation', resolution);
paths(end+1) = exportSafetyComparison(runs, 1:3, colors, config, ...
    fullfile(outputDir, 'exact_model_ablation_safety.png'), ...
    'Exact-model safety ablation', resolution);
paths(end+1) = exportTrackingComparison(runs, 3:6, colors, ...
    fullfile(outputDir, 'exact_model_robustness_tracking.png'), ...
    'Complete strict QP: robustness tracking', resolution);
paths(end+1) = exportSafetyComparison(runs, 3:6, colors, config, ...
    fullfile(outputDir, 'exact_model_robustness_safety.png'), ...
    'Complete strict QP: robustness safety metrics', resolution);
paths(end+1) = exportQpDiagnostics(runs, colors, ...
    fullfile(outputDir, 'exact_model_qp_diagnostics.png'), resolution);
paths(end+1) = exportNominalBarrierDetail(runs(3), config, ...
    fullfile(outputDir, 'exact_model_nominal_strict_barriers.png'), resolution);
end

function path = exportTrackingComparison(runs, indices, colors, fileName, ...
        figureTitle, resolution)
figureHandle = publicationFigure();
layout = tiledlayout(2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, figureTitle, 'FontWeight', 'normal');
nexttile;
hold on;
translationPeaks = zeros(numel(indices), 1);
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    errorNorm = vecnorm(run.control.poseError(1:3, :), 2, 1)*1e3;
    plot(run.control.time, errorNorm, 'LineWidth', 1.25, ...
        'Color', colors(plotIndex, :));
    translationPeaks(plotIndex) = max(errorNorm);
end
grid on;
ylabel('Translation error (mm)');
useLogScaleIfWide(gca, translationPeaks);
legend(arrayfun(@(index) runs(index).scenario.displayName, indices), ...
    'Location', 'best', 'Interpreter', 'none');
formatAxes(gca);

nexttile;
hold on;
rotationPeaks = zeros(numel(indices), 1);
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    errorNorm = rad2deg(vecnorm(run.control.poseError(4:6, :), 2, 1));
    plot(run.control.time, errorNorm, 'LineWidth', 1.25, ...
        'Color', colors(plotIndex, :));
    rotationPeaks(plotIndex) = max(errorNorm);
end
grid on;
xlabel('Time (s)');
ylabel('Rotation error (deg)');
useLogScaleIfWide(gca, rotationPeaks);
formatAxes(gca);
exportgraphics(figureHandle, fileName, 'Resolution', resolution);
close(figureHandle);
path = string(fileName);
end

function path = exportSafetyComparison(runs, indices, colors, config, ...
        fileName, figureTitle, resolution)
figureHandle = publicationFigure();
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, figureTitle, 'FontWeight', 'normal');

nexttile;
hold on;
allCollisionMargin = zeros(0, 1);
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    value = min(run.dense.collisionMargin, [], 1)*1e3;
    plot(run.dense.time, value, 'LineWidth', 1.2, ...
        'Color', colors(plotIndex, :));
    allCollisionMargin = [allCollisionMargin; value(:)]; %#ok<AGROW>
end
positiveCollision = allCollisionMargin( ...
    isfinite(allCollisionMargin) & allCollisionMargin > 0);
useCollisionLog = numel(positiveCollision) == numel(allCollisionMargin) && ...
    max(positiveCollision) > 100*min(positiveCollision);
if useCollisionLog
    set(gca, 'YScale', 'log');
else
    yline(0, 'k--', 'LineWidth', 1);
end
grid on;
ylabel('Min. clearance margin (mm)');
legend(arrayfun(@(index) runs(index).scenario.displayName, indices), ...
    'Location', 'best', 'Interpreter', 'none');
formatAxes(gca);

nexttile;
hold on;
allAcceleration = zeros(0, 1);
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    value = max(abs(run.dense.legAcceleration), [], 1);
    allAcceleration = [allAcceleration; value(:)]; %#ok<AGROW>
    plot(run.dense.time, value, 'LineWidth', 1.2, ...
        'Color', colors(plotIndex, :));
end
yline(max(config.legAccelerationLimit), 'k--', 'LineWidth', 1);
grid on;
ylabel('Max. leg acceleration (m/s^2)');
useLogScaleIfWide(gca, allAcceleration);
formatAxes(gca);

nexttile;
hold on;
allSigma = zeros(0, 1);
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    plot(run.dense.time, run.dense.sigmaMin, 'LineWidth', 1.2, ...
        'Color', colors(plotIndex, :));
    allSigma = [allSigma; run.dense.sigmaMin(:)]; %#ok<AGROW>
end
yline(config.sigmaSafe, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Min. singular value');
useLogScaleIfWide(gca, allSigma);
formatAxes(gca);

nexttile;
hold on;
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    value = max(abs(run.control.appliedForce), [], 1);
    plot(run.control.time, value, 'LineWidth', 1.2, ...
        'Color', colors(plotIndex, :));
end
yline(max(config.forceMax), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Max. applied force (N)');
formatAxes(gca);
exportgraphics(figureHandle, fileName, 'Resolution', resolution);
close(figureHandle);
path = string(fileName);
end

function path = exportQpDiagnostics(runs, colors, fileName, resolution)
indices = 2:6;
figureHandle = publicationFigure();
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, 'Exact nonlinear model: QP diagnostics', ...
    'FontWeight', 'normal');

nexttile;
hold on;
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    plot(run.control.time, 1e3*run.diagnostics.qpSolveTime, ...
        'LineWidth', 1.05, 'Color', colors(plotIndex, :));
end
yline(10, 'k--', 'LineWidth', 1);
grid on;
ylabel('QP solution time (ms)');
legend(arrayfun(@(index) runs(index).scenario.displayName, indices), ...
    'Location', 'best', 'Interpreter', 'none');
formatAxes(gca);

nexttile;
hold on;
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    value = max(run.diagnostics.clfSlack, 1e-12);
    semilogy(run.control.time, value, 'LineWidth', 1.05, ...
        'Color', colors(plotIndex, :));
end
grid on;
ylabel('CLF relaxation');
formatAxes(gca);

nexttile;
hold on;
for plotIndex = 2:numel(indices)
    run = runs(indices(plotIndex));
    plot(run.control.time, ...
        run.diagnostics.minimumCommandCbfResidual, ...
        'LineWidth', 1.05, 'Color', colors(plotIndex, :));
end
yline(0, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Minimum command-side CBF residual');
formatAxes(gca);

nexttile;
hold on;
for plotIndex = 1:numel(indices)
    run = runs(indices(plotIndex));
    stairs(run.control.time, ...
        run.diagnostics.activeConstraintCount, ...
        'LineWidth', 1.05, 'Color', colors(plotIndex, :));
end
grid on;
xlabel('Time (s)');
ylabel('Active constraints');
formatAxes(gca);
exportgraphics(figureHandle, fileName, 'Resolution', resolution);
close(figureHandle);
path = string(fileName);
end

function path = exportNominalBarrierDetail(run, config, fileName, resolution)
figureHandle = publicationFigure();
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plot(run.dense.time, run.dense.collisionMargin.'*1e3, 'LineWidth', 1.15);
hold on;
yline(0, 'k--', 'LineWidth', 1);
grid on;
ylabel('Geometric margin (mm)');
legend({'roof', 'left skirt', 'right skirt'}, ...
    'Location', 'best');
formatAxes(gca);

nexttile;
plot(run.control.time, ...
    run.diagnostics.collisionCertificateMargin.'*1e3, 'LineWidth', 1.15);
hold on;
yline(0, 'k--', 'LineWidth', 1);
grid on;
ylabel('Certificate margin (mm)');
legend({'roof', 'left skirt', 'right skirt'}, ...
    'Location', 'best');
formatAxes(gca);

nexttile;
plot(run.dense.time, run.dense.sigmaMin, ...
    'LineWidth', 1.15, 'DisplayName', 'actual sigma min');
hold on;
plot(run.control.time, sqrt(max(0, ...
    run.diagnostics.sigmaLowerSquared)), '--', 'LineWidth', 1.15, ...
    'DisplayName', 'strict lower bound');
yline(config.sigmaSafe, 'k:', 'LineWidth', 1, ...
    'DisplayName', 'safety threshold');
grid on;
xlabel('Time (s)');
ylabel('Singularity measure');
legend('Location', 'best');
formatAxes(gca);

nexttile;
yyaxis left;
plot(run.control.time, run.diagnostics.minimumCommandCbfResidual, ...
    'LineWidth', 1.1);
hold on;
yline(0, 'k--', 'LineWidth', 1);
ylabel('Min. CBF residual');
yyaxis right;
plot(run.control.time, run.diagnostics.clfSlack, ...
    'LineWidth', 1.1);
ylabel('CLF slack');
grid on;
xlabel('Time (s)');
formatAxes(gca);
exportgraphics(figureHandle, fileName, 'Resolution', resolution);
close(figureHandle);
path = string(fileName);
end

function figureHandle = publicationFigure()
figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Units', 'inches', 'Position', [1, 1, 6.8, 4.6]);
end

function formatAxes(axisHandle)
set(axisHandle, 'FontName', 'Times New Roman', 'FontSize', 10.5, ...
    'LineWidth', 0.8, 'Box', 'on');
end

function useLogScaleIfWide(axisHandle, values)
values = values(isfinite(values) & values > 0);
if ~isempty(values) && max(values) > 100*min(values)
    set(axisHandle, 'YScale', 'log');
end
end
