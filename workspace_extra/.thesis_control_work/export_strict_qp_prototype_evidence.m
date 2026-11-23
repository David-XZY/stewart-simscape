function exported = export_strict_qp_prototype_evidence(resultFile, outputDir)
% Export thesis-ready evidence from the exact nonlinear closed-loop rollout.
arguments
    resultFile {mustBeTextScalar} = fullfile('D:', 'NPU', 'Matlab_code', ...
        'stewart-simscape', 'results', 'reports', 'strict_clf_cbf_qp', ...
        'prototype_rollout_collision12.mat')
    outputDir {mustBeTextScalar} = fullfile('D:', 'NPU', 'Matlab_code', ...
        '.thesis_control_work', 'strict_evidence')
end
resultFile = char(resultFile);
outputDir = char(outputDir);
if ~exist(outputDir, 'dir'), mkdir(outputDir); end
loaded = load(resultFile, 'result');
result = loaded.result;
b = result.baseline;
s = result.strict;
t = s.time(:).';

translationBaseline = vecnorm(b.state(1:3,:)-referenceState(b, result, 1:3), 2, 1)*1e3;
translationStrict = vecnorm(s.state(1:3,:)-referenceState(s, result, 1:3), 2, 1)*1e3;
attitudeBaseline = rad2deg(vecnorm(b.state(4:6,:)-referenceState(b, result, 4:6), 2, 1));
attitudeStrict = rad2deg(vecnorm(s.state(4:6,:)-referenceState(s, result, 4:6), 2, 1));

colors = struct('baseline',[0.35 0.35 0.35], 'strict',[0.00 0.35 0.65], ...
    'limit',[0.75 0.12 0.12], 'secondary',[0.15 0.55 0.35]);

f1 = newFigure();
layout = tiledlayout(f1,2,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,translationBaseline,'--','Color',colors.baseline,'LineWidth',1.2);
plot(ax,t,translationStrict,'-','Color',colors.strict,'LineWidth',1.3);
grid(ax,'on'); ylabel(ax,'平移误差范数 / mm'); legend(ax,{'计算力矩-LQI','严格CLF-CBF-QP'},'Location','best');
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,attitudeBaseline,'--','Color',colors.baseline,'LineWidth',1.2);
plot(ax,t,attitudeStrict,'-','Color',colors.strict,'LineWidth',1.3);
grid(ax,'on'); ylabel(ax,'姿态误差范数 / (°)'); xlabel(ax,'时间 / s');
file1 = fullfile(outputDir,'strict_01_tracking_comparison.png');
exportgraphics(f1,file1,'Resolution',240); close(f1);

f2 = newFigure();
layout = tiledlayout(f2,2,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,min(s.collisionMargin,[],1)*1e3,'Color',colors.strict,'LineWidth',1.3);
yline(ax,0,'--','Color',colors.limit,'LineWidth',1.0);
grid(ax,'on'); ylabel(ax,'最小碰撞裕度 / mm');
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,s.sigmaMin,'Color',colors.strict,'LineWidth',1.3);
plot(ax,t,sqrt(max(s.sigmaLower,0)),'-.','Color',colors.secondary,'LineWidth',1.1);
yline(ax,0.10,'--','Color',colors.limit,'LineWidth',1.0);
grid(ax,'on'); ylabel(ax,'奇异值指标'); xlabel(ax,'时间 / s');
legend(ax,{'实际最小奇异值','严格下界','安全阈值'},'Location','best');
file2 = fullfile(outputDir,'strict_02_collision_singularity.png');
exportgraphics(f2,file2,'Resolution',240); close(f2);

f3 = newFigure();
layout = tiledlayout(f3,3,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(layout); hold(ax,'on');
lengthMargin = min([s.legLength-result.limits.lengthMin(:); ...
    result.limits.lengthMax(:)-s.legLength],[],1);
plot(ax,t,lengthMargin*1e3,'Color',colors.strict,'LineWidth',1.3);
yline(ax,0,'--','Color',colors.limit,'LineWidth',1.0);
grid(ax,'on'); ylabel(ax,'最小腿长裕度 / mm');
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,max(abs(s.legSpeed),[],1),'Color',colors.strict,'LineWidth',1.3);
yline(ax,max(result.limits.legSpeed),'--','Color',colors.limit,'LineWidth',1.0);
grid(ax,'on'); ylabel(ax,'最大腿速 / (m·s^{-1})');
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,max(abs(s.legAcceleration),[],1),'Color',colors.strict,'LineWidth',1.3);
yline(ax,max(result.limits.legAcceleration),'--','Color',colors.limit,'LineWidth',1.0);
grid(ax,'on'); ylabel(ax,'最大腿加速度 / (m·s^{-2})'); xlabel(ax,'时间 / s');
file3 = fullfile(outputDir,'strict_03_leg_constraints.png');
exportgraphics(f3,file3,'Resolution',240); close(f3);

f4 = newFigure();
layout = tiledlayout(f4,2,2,'TileSpacing','compact','Padding','compact');
ax = nexttile(layout); plot(ax,t,s.hardCbfResidual,'Color',colors.strict,'LineWidth',1.1); hold(ax,'on');
yline(ax,0,'--','Color',colors.limit); grid(ax,'on'); ylabel(ax,'硬CBF残差');
ax = nexttile(layout); plot(ax,t,s.psi1,'Color',colors.strict,'LineWidth',1.1); hold(ax,'on');
yline(ax,0,'--','Color',colors.limit); grid(ax,'on'); ylabel(ax,'最小 \psi_1');
ax = nexttile(layout); plot(ax,t,s.clfSlack,'Color',colors.secondary,'LineWidth',1.1);
grid(ax,'on'); ylabel(ax,'CLF松弛量'); xlabel(ax,'时间 / s');
ax = nexttile(layout); plot(ax,t,s.qpTime*1e3,'Color',colors.strict,'LineWidth',1.0); hold(ax,'on');
yline(ax,10,'--','Color',colors.limit); grid(ax,'on'); ylabel(ax,'QP求解时间 / ms'); xlabel(ax,'时间 / s');
file4 = fullfile(outputDir,'strict_04_qp_diagnostics.png');
exportgraphics(f4,file4,'Resolution',240); close(f4);

f5 = newFigure();
layout = tiledlayout(f5,2,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(layout); plot(ax,t,b.force.','Color',[0.55 0.55 0.55],'LineWidth',0.8);
grid(ax,'on'); ylabel(ax,'基线支链力 / N');
ax = nexttile(layout); plot(ax,t,s.force.','LineWidth',0.9);
grid(ax,'on'); ylabel(ax,'严格QP支链力 / N'); xlabel(ax,'时间 / s');
file5 = fullfile(outputDir,'strict_05_force_history.png');
exportgraphics(f5,file5,'Resolution',240); close(f5);

method = ["计算力矩-LQI"; "严格CLF-CBF-QP"];
position_rmse_mm = 1e3*[b.metrics.positionRmse; s.metrics.positionRmse];
attitude_rmse_deg = rad2deg([b.metrics.attitudeRmse; s.metrics.attitudeRmse]);
position_peak_mm = 1e3*[b.metrics.maxPositionError; s.metrics.maxPositionError];
attitude_peak_deg = rad2deg([b.metrics.maxAttitudeError; s.metrics.maxAttitudeError]);
leg_acceleration_peak = [b.metrics.maxLegAcceleration; s.metrics.maxLegAcceleration];
leg_length_min = [b.metrics.minLegLength; s.metrics.minLegLength];
leg_length_max = [b.metrics.maxLegLength; s.metrics.maxLegLength];
collision_margin_min_mm = 1e3*[b.metrics.minCollisionMargin; s.metrics.minCollisionMargin];
sigma_min = [b.metrics.minSigma; s.metrics.minSigma];
force_peak = [b.metrics.maxForce; s.metrics.maxForce];
force_rate_peak = [b.metrics.maxForceRate; s.metrics.maxForceRate];
control_energy = [b.metrics.controlEnergy; s.metrics.controlEnergy];
fallback_count = [b.metrics.fallbackCount; s.metrics.fallbackCount];
passed = [b.metrics.passed; s.metrics.passed];
summary = table(method,position_rmse_mm,attitude_rmse_deg,position_peak_mm, ...
    attitude_peak_deg,leg_length_min,leg_length_max,leg_acceleration_peak, ...
    collision_margin_min_mm,sigma_min, ...
    force_peak,force_rate_peak,control_energy,fallback_count,passed);
csvFile = fullfile(outputDir,'strict_metrics_summary.csv');
writetable(summary,csvFile,'Encoding','UTF-8');

qp = table(s.metrics.minCbfMargin,s.metrics.minPsi1,s.metrics.minHardCbfResidual, ...
    s.metrics.maxClfSlack,s.metrics.infeasibleCount,s.metrics.fallbackCount, ...
    1e3*s.metrics.qpMean,1e3*s.metrics.qpP95,1e3*s.metrics.qpMax, ...
    'VariableNames',{'minimum_state_margin','minimum_psi1','minimum_hard_cbf_residual', ...
    'maximum_clf_slack','infeasible_count','fallback_count','qp_mean_ms','qp_p95_ms','qp_max_ms'});
qpCsvFile = fullfile(outputDir,'strict_qp_diagnostics.csv');
writetable(qp,qpCsvFile,'Encoding','UTF-8');

exported = struct('figures',string({file1,file2,file3,file4,file5}), ...
    'summaryCsv',string(csvFile),'diagnosticsCsv',string(qpCsvFile));
end

function reference = referenceState(run, result, indices)
% The two rollouts share the trajectory; recover it from state-error data is
% intentionally avoided. The reference is reconstructed from the stored
% strict and baseline runs only when an explicit reference field exists.
if isfield(run,'reference')
    reference = run.reference(indices,:);
elseif isfield(result,'reference')
    reference = result.reference.q(indices,:);
else
    error('export_strict_qp_prototype_evidence:MissingReference', ...
        'The rollout result must store the reference trajectory.');
end
end

function fig = newFigure()
fig = figure('Visible','off','Color','w','Position',[100 100 1050 720]);
set(fig,'DefaultAxesFontName','Microsoft YaHei', ...
    'DefaultTextFontName','Microsoft YaHei','DefaultAxesFontSize',10.5);
end
