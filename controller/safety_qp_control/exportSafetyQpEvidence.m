function exported = exportSafetyQpEvidence(comparison, outputDir, config)
% exportSafetyQpEvidence - 导出 SC-QP 控制方法的表格、数据和图像证据
arguments
    comparison struct
    outputDir {mustBeTextScalar}
    config struct
end

outputDir = char(outputDir);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

controlTable = comparison.summaryTable;
ablationTable = comparison.ablationTable;
robustnessTable = comparison.robustnessTable;
writetable(controlTable, fullfile(outputDir, 'table_control_comparison.csv'));
writetable(ablationTable, fullfile(outputDir, 'table_clf_cbf_ablation.csv'));
writetable(robustnessTable, fullfile(outputDir, 'table_robustness_safety_qp.csv'));
save(fullfile(outputDir, 'safety_qp_comparison_data.mat'), ...
    'comparison', 'controlTable', 'ablationTable', 'robustnessTable');

figureFiles = strings(0, 1);
figureFiles(end + 1) = saveControlStructureFigure(outputDir);
figureFiles(end + 1) = saveSingleStepQpFigure(outputDir);
figureFiles(end + 1) = savePoseErrorFigure(comparison, outputDir);
figureFiles(end + 1) = saveAttitudeErrorFigure(comparison, outputDir);
figureFiles(end + 1) = saveForceFigure(comparison, outputDir);
figureFiles(end + 1) = saveForceRateFigure(comparison, outputDir);
figureFiles(end + 1) = saveLegConstraintFigure(comparison, outputDir, config);
figureFiles(end + 1) = saveLyapunovFigure(comparison, outputDir);
figureFiles(end + 1) = saveCbfMarginFigure(comparison, outputDir);
figureFiles(end + 1) = saveActiveConstraintFigure(comparison, outputDir);
figureFiles(end + 1) = saveAblationFigure(comparison, outputDir);
figureFiles(end + 1) = saveRobustnessFigure(comparison, outputDir);
figureFiles(end + 1) = saveSingularityFigure(comparison, outputDir);
figureFiles(end + 1) = saveCollisionFigure(comparison, outputDir);
figureFiles(end + 1) = saveQpTimeFigure(comparison, outputDir);

exported = struct();
exported.outputDir = outputDir;
exported.figureFiles = figureFiles;
exported.controlTableFile = fullfile(outputDir, 'table_control_comparison.csv');
exported.ablationTableFile = fullfile(outputDir, 'table_clf_cbf_ablation.csv');
exported.robustnessTableFile = fullfile(outputDir, 'table_robustness_safety_qp.csv');
exported.dataFile = fullfile(outputDir, 'safety_qp_comparison_data.mat');
end

function baseName = saveControlStructureFigure(outputDir)
fig = newFigure();
axis off;
labels = ["Nominal controller", "CLF constraint", "CBF supervisor", ...
    "QP solver", "Stewart plant", "Diagnostics"];
positions = [0.08 0.55; 0.36 0.75; 0.36 0.35; 0.62 0.55; 0.84 0.55; 0.62 0.18];
for index = 1:numel(labels)
    annotation('textbox', [positions(index, 1), positions(index, 2), 0.16, 0.12], ...
        'String', labels(index), 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'LineWidth', 1.2, 'BackgroundColor', [0.96 0.98 1]);
end
title('SC-QP control structure');
baseName = saveFigure(fig, outputDir, '01_sc_qp_control_structure');
end

function baseName = saveSingleStepQpFigure(outputDir)
fig = newFigure();
x = linspace(-2, 2, 160);
y = linspace(-2, 2, 160);
[X, Y] = meshgrid(x, y);
Z = (X - 0.45).^2 + 0.7 * (Y + 0.25).^2;
contourf(X, Y, Z, 18, 'LineColor', 'none');
hold on;
plot([-1.8, 1.8], [0.8, -0.4], 'w-', 'LineWidth', 2);
plot([-1.3, 1.1], [-1.3, 1.5], 'k-', 'LineWidth', 2);
plot(0.45, -0.25, 'ro', 'MarkerFaceColor', 'r');
plot(0.05, 0.20, 'go', 'MarkerFaceColor', 'g');
grid on;
xlabel('F_1 direction');
ylabel('F_2 direction');
title('Single-step CLF-CBF-QP geometry');
legend({'objective', 'CLF boundary', 'CBF boundary', 'nominal force', 'QP command'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
baseName = saveFigure(fig, outputDir, '02_single_step_qp_geometry');
end

function baseName = savePoseErrorFigure(comparison, outputDir)
fig = newFigure();
hold on;
for index = selectedRunIndices(comparison)
    run = comparison.runs(index);
    plot(run.time, vecnorm(run.poseError(1:3, :), 2, 1), 'LineWidth', 1.4);
end
grid on;
xlabel('Time (s)');
ylabel('Position error (m)');
title('Position tracking error comparison');
legend(selectedRunNames(comparison), 'Interpreter', 'none');
baseName = saveFigure(fig, outputDir, '03_position_tracking_error');
end

function baseName = saveAttitudeErrorFigure(comparison, outputDir)
fig = newFigure();
hold on;
for index = selectedRunIndices(comparison)
    run = comparison.runs(index);
    plot(run.time, vecnorm(run.poseError(4:6, :), 2, 1), 'LineWidth', 1.4);
end
grid on;
xlabel('Time (s)');
ylabel('Attitude error (rad)');
title('Attitude tracking error comparison');
legend(selectedRunNames(comparison), 'Interpreter', 'none');
baseName = saveFigure(fig, outputDir, '04_attitude_tracking_error');
end

function baseName = saveForceFigure(comparison, outputDir)
fig = newFigure();
run = comparison.runs(end);
plot(run.time, run.force.', 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Six-leg SC-QP commanded force');
baseName = saveFigure(fig, outputDir, '05_six_leg_force');
end

function baseName = saveForceRateFigure(comparison, outputDir)
fig = newFigure();
run = comparison.runs(end);
forceRate = [zeros(6, 1), diff(run.force, 1, 2) / median(diff(run.time))];
plot(run.time, forceRate.', 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Force rate (N/s)');
title('Six-leg force-rate comparison');
baseName = saveFigure(fig, outputDir, '06_force_rate');
end

function baseName = saveLegConstraintFigure(comparison, outputDir, config)
fig = newFigure();
run = comparison.runs(end);
tiledlayout(3, 1);
nexttile; plot(run.time, run.legLength.', 'LineWidth', 1.0); hold on;
yline(min(config.lengthMin + config.lengthMargin), 'r--');
yline(max(config.lengthMax - config.lengthMargin), 'r--');
ylabel('Length (m)'); grid on;
nexttile; plot(run.time, run.legSpeed.', 'LineWidth', 1.0); hold on;
yline(max(config.legSpeedLimit), 'r--'); yline(-max(config.legSpeedLimit), 'r--');
ylabel('Speed (m/s)'); grid on;
nexttile; plot(run.time, run.legAcceleration.', 'LineWidth', 1.0); hold on;
yline(max(config.legAccelerationLimit), 'r--'); yline(-max(config.legAccelerationLimit), 'r--');
ylabel('Accel. (m/s^2)'); xlabel('Time (s)'); grid on;
title('Leg length, speed and acceleration constraints');
baseName = saveFigure(fig, outputDir, '07_leg_constraints');
end

function baseName = saveLyapunovFigure(comparison, outputDir)
fig = newFigure();
run = comparison.runs(end);
plot(run.time, run.V, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('V');
title('CLF Lyapunov function');
baseName = saveFigure(fig, outputDir, '08_lyapunov_function');
end

function baseName = saveCbfMarginFigure(comparison, outputDir)
fig = newFigure();
run = comparison.runs(end);
plot(run.time, run.minCbfMarginSeries, 'LineWidth', 1.5);
hold on;
plot(run.time, run.sigmaMin, 'LineWidth', 1.2);
plot(run.time, run.collisionDistance, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Margin');
title('CBF safety margins');
legend({'minimum CBF margin', 'sigma min', 'collision distance'});
baseName = saveFigure(fig, outputDir, '09_cbf_safety_margin');
end

function baseName = saveActiveConstraintFigure(comparison, outputDir)
fig = newFigure();
run = comparison.runs(end);
imagesc(run.time, 1:7, run.activeCounts.');
colormap(parula);
colorbar;
yticks(1:7);
yticklabels({'force', 'rate', 'length', 'speed', 'accel', 'singularity', 'collision'});
xlabel('Time (s)');
title('QP active constraints');
baseName = saveFigure(fig, outputDir, '10_active_constraints');
end

function baseName = saveAblationFigure(comparison, outputDir)
fig = newFigure();
bar(comparison.ablationTable.tracking_error);
grid on;
xticks(1:height(comparison.ablationTable));
xticklabels(comparison.ablationTable.method);
xtickangle(25);
ylabel('Tracking error');
title('CLF-CBF ablation');
baseName = saveFigure(fig, outputDir, '11_clf_cbf_ablation');
end

function baseName = saveRobustnessFigure(comparison, outputDir)
fig = newFigure();
z = reshapeComparisonForHeatmap(comparison.robustnessTable.position_rmse);
imagesc(z);
colorbar;
xlabel('Payload disturbance');
ylabel('Actuator gain error');
title('Robustness heatmap');
baseName = saveFigure(fig, outputDir, '12_robustness_heatmap');
end

function baseName = saveSingularityFigure(comparison, outputDir)
fig = newFigure();
hold on;
for index = selectedRunIndices(comparison)
    run = comparison.runs(index);
    plot(run.time, run.sigmaMin, 'LineWidth', 1.3);
end
grid on;
xlabel('Time (s)');
ylabel('\sigma_{min}');
title('Minimum singular value comparison');
legend(selectedRunNames(comparison), 'Interpreter', 'none');
baseName = saveFigure(fig, outputDir, '13_minimum_singular_value');
end

function baseName = saveCollisionFigure(comparison, outputDir)
fig = newFigure();
hold on;
for index = selectedRunIndices(comparison)
    run = comparison.runs(index);
    plot(run.time, run.collisionDistance, 'LineWidth', 1.3);
end
grid on;
xlabel('Time (s)');
ylabel('Distance (m)');
title('Minimum collision distance comparison');
legend(selectedRunNames(comparison), 'Interpreter', 'none');
baseName = saveFigure(fig, outputDir, '14_minimum_collision_distance');
end

function baseName = saveQpTimeFigure(comparison, outputDir)
fig = newFigure();
qpTimes = arrayfun(@(run) run.metrics.mean_qp_time, comparison.runs).';
bar(qpTimes);
grid on;
xticks(1:numel(comparison.runs));
xticklabels(comparison.methodNames);
xtickangle(35);
ylabel('Mean solve time (s)');
title('QP solve-time statistics');
baseName = saveFigure(fig, outputDir, '15_qp_solve_time');
end

function indices = selectedRunIndices(comparison)
indices = [3, 4, numel(comparison.runs)];
indices = indices(indices <= numel(comparison.runs));
end

function names = selectedRunNames(comparison)
indices = selectedRunIndices(comparison);
names = comparison.methodNames(indices);
end

function z = reshapeComparisonForHeatmap(values)
values = values(:);
gridSize = ceil(sqrt(numel(values)));
z = nan(gridSize);
z(1:numel(values)) = values;
end

function fig = newFigure()
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1280, 720]);
end

function baseName = saveFigure(fig, outputDir, stem)
baseName = fullfile(outputDir, stem);
set(fig, 'PaperPositionMode', 'auto');
savefig(fig, [baseName, '.fig']);
print(fig, [baseName, '.png'], '-dpng', '-r200');
close(fig);
end
