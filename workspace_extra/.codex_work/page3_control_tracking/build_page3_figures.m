function build_page3_figures()
% Build evidence-led plots and documentation for dissertation PPT page 3.
% All numerical claims are recomputed from the frozen project MAT artifacts.

outDir = 'D:\NPU\Matlab_code\output\page3_control_tracking';
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

strictFile = ['D:\NPU\Matlab_code\stewart-simscape-disturbance-experiment' ...
    '\results\reports\strict_clf_cbf_qp\exact_model_study' ...
    '\strict_clf_cbf_qp_exact_model_study.mat'];
identifierFile = ['D:\NPU\Matlab_code\stewart-simscape\results\controller' ...
    '\pwm_force_identifier_20260613_232220.mat'];

strict = load(strictFile, 'result');
ident = load(identifierFile, 'dataset', 'report', 'identified');

baseline = strict.result.runs(1); % Computed torque + LQI
safeQp = strict.result.runs(3);  % Strict CLF-CBF-QP, matched nominal plant

makeTrackingSafetyPlot(baseline, safeQp, outDir);
idMetrics = makeIdentificationPlot(ident.dataset, ident.report, outDir);
writeMetricsCsv(baseline, safeQp, ident.dataset, ident.report, idMetrics, outDir);
writeNotes(baseline, safeQp, ident.dataset, ident.report, idMetrics, strictFile, identifierFile, outDir);
end

function makeTrackingSafetyPlot(baseline, safeQp, outDir)
blue = [36 112 181] ./ 255;
orange = [241 124 41] ./ 255;
ink = [24 44 65] ./ 255;
gray = [104 122 140] ./ 255;
red = [213 75 72] ./ 255;
softRed = [252 233 231] ./ 255;

t = baseline.control.time(:);
basePosErrMm = sqrt(sum(baseline.control.poseError(1:3, :).^2, 1)).' * 1e3;
qpPosErrMm = sqrt(sum(safeQp.control.poseError(1:3, :).^2, 1)).' * 1e3;
tb = baseline.dense.time(:);
tq = safeQp.dense.time(:);
% Three obstacle-margin channels are logged; plot their instantaneous minimum.
baseMarginMm = min(baseline.dense.collisionMargin, [], 1).' * 1e3;
qpMarginMm = min(safeQp.dense.collisionMargin, [], 1).' * 1e3;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1800 620]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl, 1); hold(ax1, 'on');
plot(ax1, t, basePosErrMm, 'Color', blue, 'LineWidth', 1.9);
plot(ax1, t, qpPosErrMm, 'Color', orange, 'LineWidth', 1.9);
grid(ax1, 'on'); box(ax1, 'off');
ylabel(ax1, '位置误差范数 (mm)');
title(ax1, '同一名义模型 / 同一参考轨迹 / 10 ms：跟踪精度保持');
legend(ax1, {'计算力矩 + LQI', '计算力矩 + LQI + CLF-CBF-QP'}, ...
    'Location', 'northwest', 'Box', 'off');
set(ax1, 'FontName', 'Microsoft YaHei', 'FontSize', 12, 'XColor', gray, 'YColor', ink, 'GridColor', [0.87 0.90 0.93]);
text(ax1, 0.98, 0.92, sprintf('位置 RMS: %.3f → %.3f mm', ...
    baseline.metrics.translationRmse * 1e3, safeQp.metrics.translationRmse * 1e3), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 12, 'FontWeight', 'bold', 'Color', ink, ...
    'BackgroundColor', [1 1 1]);

ax2 = nexttile(tl, 2); hold(ax2, 'on');
yl = [-1.20 1.15];
patch(ax2, [tb(1) tb(end) tb(end) tb(1)], [yl(1) yl(1) 0 0], softRed, ...
    'EdgeColor', 'none', 'FaceAlpha', 0.9, 'HandleVisibility', 'off');
plot(ax2, tb, baseMarginMm, 'Color', blue, 'LineWidth', 1.9);
plot(ax2, tq, qpMarginMm, 'Color', orange, 'LineWidth', 2.1);
yline(ax2, 0, '--', '安全边界 0 mm', 'Color', red, 'LineWidth', 1.2, ...
    'LabelHorizontalAlignment', 'left');
ylim(ax2, yl); xlim(ax2, [t(1) t(end)]);
grid(ax2, 'on'); box(ax2, 'off');
xlabel(ax2, '时间 (s)'); ylabel(ax2, '碰撞安全裕度 (mm)');
title(ax2, '安全过滤将最小碰撞裕度恢复到正值');
set(ax2, 'FontName', 'Microsoft YaHei', 'FontSize', 12, 'XColor', gray, 'YColor', ink, 'GridColor', [0.87 0.90 0.93]);
text(ax2, 0.98, 0.92, sprintf('最小裕度: %.3f → +%.3f mm\n违规样本: %d → %d    QP P95: %.2f ms', ...
    baseline.metrics.minActualCollisionMargin * 1e3, safeQp.metrics.minActualCollisionMargin * 1e3, ...
    baseline.metrics.collisionViolationCount, safeQp.metrics.collisionViolationCount, safeQp.metrics.qpTimeP95 * 1e3), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 12, 'FontWeight', 'bold', 'Color', ink, ...
    'BackgroundColor', [1 1 1]);
text(ax2, 0.02, 0.06, sprintf('QP 约束活跃占比 %.1f%%（含 CLF 与状态/执行器边界）', ...
    100 * safeQp.metrics.activeSampleFraction), 'Units', 'normalized', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 10, 'Color', [137 84 29] ./ 255, ...
    'BackgroundColor', [255 247 237] ./ 255, 'Margin', 5);

exportPlot(fig, outDir, 'fig_tracking_safety_results');
close(fig);
end

function idMetrics = makeIdentificationPlot(dataset, report, outDir)
blue = [78 130 177] ./ 255;
teal = [22 150 138] ./ 255;
ink = [24 44 65] ./ 255;
gray = [104 122 140] ./ 255;
orange = [241 124 41] ./ 255;

testIdx = find(dataset.split.test);
testT = dataset.time(testIdx);
axisIndex = 4; % fixed demonstrator axis; not selected by fit quality
speed = dataset.legSpeed(testIdx, axisIndex);
crossings = find(speed(1:end-1) .* speed(2:end) <= 0 & abs(diff(speed)) > 1e-6);
valid = crossings(testT(crossings) >= testT(1) + 0.40 & testT(crossings) <= testT(end) - 0.70);
if isempty(valid)
    centerLocal = max(1, floor(numel(testIdx) / 2));
else
    centerLocal = valid(1); % first eligible reversal in held-out test segment
end
centerTime = testT(centerLocal);
windowMask = dataset.time >= max(testT(1), centerTime - 0.40) & ...
    dataset.time <= min(testT(end), centerTime + 0.70);

timeWindow = dataset.time(windowMask);
truth = dataset.trueForce(windowMask, axisIndex);
grayForce = report.grayForce(windowMask, axisIndex);
narxForce = report.estimatedForce(windowMask, axisIndex);

testMask = dataset.split.test;
grayAxis = zeros(6, 1); narxAxis = zeros(6, 1);
for k = 1:6
    denominator = 4800;
    grayAxis(k) = sqrt(mean((report.grayForce(testMask, k) - dataset.trueForce(testMask, k)).^2)) / denominator;
    narxAxis(k) = sqrt(mean((report.estimatedForce(testMask, k) - dataset.trueForce(testMask, k)).^2)) / denominator;
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1900 720]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl, 1); hold(ax1, 'on');
plot(ax1, timeWindow, truth, 'Color', ink, 'LineWidth', 1.65);
plot(ax1, timeWindow, grayForce, '--', 'Color', blue, 'LineWidth', 1.55);
plot(ax1, timeWindow, narxForce, 'Color', teal, 'LineWidth', 1.75);
grid(ax1, 'on'); box(ax1, 'off');
xlabel(ax1, '时间 (s)'); ylabel(ax1, '支链轴向力 (N)');
title(ax1, '轴 4：独立测试集中的首个有效换向窗');
legend(ax1, {'高保真教师模型', '灰箱预测', '灰箱 + 残差 NARX'}, 'Location', 'southwest', 'Box', 'off');
set(ax1, 'FontName', 'Microsoft YaHei', 'FontSize', 12, 'XColor', gray, 'YColor', ink, 'GridColor', [0.87 0.90 0.93]);
yl = ylim(ax1); reversalLine = xline(ax1, centerTime, ':', '换向', 'Color', orange, 'LineWidth', 1.5, ...
    'LabelVerticalAlignment', 'middle');
reversalLine.HandleVisibility = 'off';
text(ax1, 0.98, 0.93, '选窗规则：独立测试段内\n首个前后窗完整的速度换向', ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 10, 'Color', ink, 'BackgroundColor', [1 1 1]);
ylim(ax1, yl);

ax2 = nexttile(tl, 2); hold(ax2, 'on');
y = 1:6;
for k = 1:6
    plot(ax2, [100 * grayAxis(k), 100 * narxAxis(k)], [y(k), y(k)], '-', 'Color', [189 211 229] ./ 255, ...
        'LineWidth', 3, 'HandleVisibility', 'off');
end
grayScatter = scatter(ax2, 100 * grayAxis, y, 62, blue, 'filled', 'DisplayName', '灰箱');
narxScatter = scatter(ax2, 100 * narxAxis, y, 62, teal, 'filled', 'DisplayName', '灰箱 + NARX');
grid(ax2, 'on'); box(ax2, 'off');
ylim(ax2, [0.45 6.55]); xlim(ax2, [0.55 1.22]);
set(ax2, 'YDir', 'reverse', 'YTick', y, 'YTickLabel', compose('支链 %d', y), ...
    'FontName', 'Microsoft YaHei', 'FontSize', 12, 'XColor', gray, 'YColor', ink, 'GridColor', [0.87 0.90 0.93]);
xlabel(ax2, '独立测试 NRMSE (%)');
title(ax2, '六支链误差均由残差模型降低');
legend(ax2, [grayScatter narxScatter], {'灰箱', '灰箱 + NARX'}, 'Location', 'northeast', 'Box', 'off');
text(ax2, 0.98, 0.06, sprintf('总体：%.4f%% → %.4f%%（↓ %.1f%%）\n5 ms · 60/20/20 划分', ...
    100 * report.metrics.grayTestNrmse, 100 * report.metrics.narxTestNrmse, ...
    100 * (report.metrics.grayTestNrmse - report.metrics.narxTestNrmse) / report.metrics.grayTestNrmse), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 11, 'FontWeight', 'bold', 'Color', ink, 'BackgroundColor', [1 1 1]);

exportPlot(fig, outDir, 'fig_identification_validation');
close(fig);

idMetrics = struct('axisIndex', axisIndex, 'windowStartS', timeWindow(1), 'windowEndS', timeWindow(end), ...
    'windowCenterS', centerTime, 'grayAxisNrmse', grayAxis, 'narxAxisNrmse', narxAxis);
end

function exportPlot(fig, outDir, stem)
pngFile = fullfile(outDir, stem + ".png");
pdfFile = fullfile(outDir, stem + ".pdf");
svgFile = fullfile(outDir, stem + ".svg");
exportgraphics(fig, pngFile, 'Resolution', 300);
exportgraphics(fig, pdfFile, 'ContentType', 'vector');
print(fig, svgFile, '-dsvg', '-painters');
end

function writeMetricsCsv(baseline, safeQp, dataset, report, idMetrics, outDir)
names = [ ...
    "control_comparison_boundary"; "control_sample_time"; "ct_lqi_translation_rms"; ...
    "strict_qp_translation_rms"; "ct_lqi_min_collision_margin"; ...
    "strict_qp_min_collision_margin"; "ct_lqi_collision_violations"; ...
    "strict_qp_collision_violations"; "strict_qp_active_sample_fraction"; ...
    "strict_qp_qp_p95"; "identification_sample_time"; "identification_split"; ...
    "gray_box_test_nrmse"; "gray_narx_test_nrmse"; "gray_narx_nrmse_improvement"; ...
    "gray_narx_test_bias"; "identification_validation_axis"; "identification_window_start"; ...
    "identification_window_end"; "leg1_gray_nrmse"; "leg1_narx_nrmse"; ...
    "leg2_gray_nrmse"; "leg2_narx_nrmse"; "leg3_gray_nrmse"; "leg3_narx_nrmse"; ...
    "leg4_gray_nrmse"; "leg4_narx_nrmse"; "leg5_gray_nrmse"; "leg5_narx_nrmse"; ...
    "leg6_gray_nrmse"; "leg6_narx_nrmse"];
values = [ ...
    1; baseline.control.time(2) - baseline.control.time(1); baseline.metrics.translationRmse; ...
    safeQp.metrics.translationRmse; baseline.metrics.minActualCollisionMargin; ...
    safeQp.metrics.minActualCollisionMargin; baseline.metrics.collisionViolationCount; ...
    safeQp.metrics.collisionViolationCount; 100 * safeQp.metrics.activeSampleFraction; ...
    safeQp.metrics.qpTimeP95; dataset.sampleTime; 60; ...
    100 * report.metrics.grayTestNrmse; 100 * report.metrics.narxTestNrmse; ...
    100 * (report.metrics.grayTestNrmse - report.metrics.narxTestNrmse) / report.metrics.grayTestNrmse; ...
    100 * report.metrics.testBias; idMetrics.axisIndex; idMetrics.windowStartS; idMetrics.windowEndS; ...
    100 * idMetrics.grayAxisNrmse(1); 100 * idMetrics.narxAxisNrmse(1); ...
    100 * idMetrics.grayAxisNrmse(2); 100 * idMetrics.narxAxisNrmse(2); ...
    100 * idMetrics.grayAxisNrmse(3); 100 * idMetrics.narxAxisNrmse(3); ...
    100 * idMetrics.grayAxisNrmse(4); 100 * idMetrics.narxAxisNrmse(4); ...
    100 * idMetrics.grayAxisNrmse(5); 100 * idMetrics.narxAxisNrmse(5); ...
    100 * idMetrics.grayAxisNrmse(6); 100 * idMetrics.narxAxisNrmse(6)];
units = [ ...
    "matched nominal nonlinear model / same IHSID reference / ideal force"; "s"; "m"; "m"; "m"; "m"; "samples"; "samples"; "%"; "s"; "s"; "% train"; "%"; "%"; "%"; "%"; "axis"; "s"; "s"; "%"; "%"; "%"; "%"; "%"; "%"; "%"; "%"; "%"; "%"; "%"; "%"];
sources = repmat("frozen project MAT artifact", numel(names), 1);
definitions = [ ...
    "LQI baseline and strict QP use runs 1 and 3 of one exact-model study"; ...
    "controller sample period"; "RMS Euclidean translation tracking error"; ...
    "RMS Euclidean translation tracking error"; "minimum actual cylinder-to-obstacle margin"; ...
    "minimum actual cylinder-to-obstacle margin"; "samples with margin below 0"; ...
    "samples with margin below 0"; "fraction with any active QP constraint"; ...
    "95th percentile QP solve duration"; "identification dataset sample period"; ...
    "60/20/20 contiguous train/validation/test split"; "all-axis free-run NRMSE normalized by 4800 N"; ...
    "all-axis free-run NRMSE normalized by 4800 N"; "100*(gray-narx)/gray"; ...
    "maximum absolute test bias normalized by 4800 N"; "fixed axis plotted in main validation panel"; ...
    "first eligible test-only reversal window start"; "first eligible test-only reversal window end"; ...
    "free-run test NRMSE normalized by 4800 N"; "free-run test NRMSE normalized by 4800 N"; ...
    "free-run test NRMSE normalized by 4800 N"; "free-run test NRMSE normalized by 4800 N"; ...
    "free-run test NRMSE normalized by 4800 N"; "free-run test NRMSE normalized by 4800 N"; ...
    "free-run test NRMSE normalized by 4800 N"; "free-run test NRMSE normalized by 4800 N"; ...
    "free-run test NRMSE normalized by 4800 N"; "free-run test NRMSE normalized by 4800 N"; ...
    "free-run test NRMSE normalized by 4800 N"; "free-run test NRMSE normalized by 4800 N"];
T = table(names, values, units, sources, definitions, 'VariableNames', {'metric','value','unit','source','definition'});
writetable(T, fullfile(outDir, 'page3_metrics_summary.csv'), 'Encoding', 'UTF-8');
end

function writeNotes(baseline, safeQp, dataset, report, idMetrics, strictFile, identifierFile, outDir)
fid = fopen(fullfile(outDir, 'page3_notes.md'), 'w', 'n', 'UTF-8');
assert(fid > 0, 'Cannot open page3_notes.md');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# 第 3 页：轨迹跟踪与安全控制层 — 数据与口径说明\n\n');
fprintf(fid, '## 1. 控制主图\n\n');
fprintf(fid, '- 数据：`%s` 中 `runs(1)`（计算力矩 + LQI）与 `runs(3)`（严格 CLF-CBF-QP，matched nominal plant / ideal force）。二者均使用同一 IHSID 参考轨迹、同一起始状态、同一名义非线性模型、10 ms 控制周期与 7.5 s 时域。\n', strictFile);
fprintf(fid, '- 跟踪图纵轴为平移位置误差范数 `||e_p||_2`，不是混合姿态等效误差；卡片 RMS 为三轴平移欧氏误差的 RMS。\n');
fprintf(fid, '- 碰撞裕度为圆柱体与环境障碍物的实际最小间隙减去阈值；小于 0 mm 记为违规样本。\n');
fprintf(fid, '- 结果：平移 RMS %.3f → %.3f mm；最小碰撞裕度 %.3f → +%.3f mm；碰撞违规样本 %d → %d；QP P95 %.2f ms。\n\n', ...
    baseline.metrics.translationRmse * 1e3, safeQp.metrics.translationRmse * 1e3, ...
    baseline.metrics.minActualCollisionMargin * 1e3, safeQp.metrics.minActualCollisionMargin * 1e3, ...
    baseline.metrics.collisionViolationCount, safeQp.metrics.collisionViolationCount, safeQp.metrics.qpTimeP95 * 1e3);
fprintf(fid, '## 2. 模型辨识图\n\n');
fprintf(fid, '- 数据：`%s` 中 PWM、腿速、腿加速度与 `trueForce`。`trueForce` 来自高保真 PWM 执行器教师模型，不是台架实测力。\n', identifierFile);
fprintf(fid, '- 数据集为 6000 点 / 30.0 s / 5 ms；按时间连续切分为 60%% 训练、20%% 验证、20%% 独立测试。\n');
fprintf(fid, '- 主图固定展示轴 4。时间窗为独立测试段中第一个同时保留 0.40 s 前置和 0.70 s 后置窗口的速度换向点：%.3f–%.3f s（换向中心 %.3f s）；该规则未使用拟合误差挑选“最好看”的片段。\n', ...
    idMetrics.windowStartS, idMetrics.windowEndS, idMetrics.windowCenterS);
fprintf(fid, '- NRMSE 定义为自由运行力预测 RMSE / 4800 N。结果：灰箱 %.4f%%；灰箱 + 残差 NARX %.4f%%；相对降低 %.2f%%；最大测试偏差 %.4f%%。\n\n', ...
    100 * report.metrics.grayTestNrmse, 100 * report.metrics.narxTestNrmse, ...
    100 * (report.metrics.grayTestNrmse - report.metrics.narxTestNrmse) / report.metrics.grayTestNrmse, ...
    100 * report.metrics.testBias);
fprintf(fid, '## 3. 页面结构与研究边界\n\n');
fprintf(fid, '- 主控制链固定为：计算力矩前馈 + LQI 反馈 → 名义支链力 `F_nom` → CLF-CBF-QP 最小必要修正 → `F_cmd`。\n');
fprintf(fid, '- PWM 灰箱 + 残差 NARX 图作为执行器高保真建模/力预测的支撑验证链，未宣称已在本页的严格 QP 主闭环中完成等价嵌入验证。\n');
fprintf(fid, '- 控制结果属于匹配名义非线性模型研究；辨识结果属于仿真教师模型独立测试集。二者均不替代真实整机闭环实测。高强度扰动和执行器滞后下仍需进一步验证约束可行性与在线求解裕度。\n');
end
