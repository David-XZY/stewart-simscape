function analyze_page2_pidik_comparison()
% ANALYZE_PAGE2_PIDIK_COMPARISON
% Compare the accepted 61-node IHSID reference against the stored Simscape
% pose-feedback / PID length-execution record that used the same reference.
% Only signals available in both datasets are compared.  The execution record
% does not log actuator force, so force is deliberately excluded.

set(groot, 'defaultFigureVisible', 'off');
rootDir = 'D:/NPU/Matlab_code/stewart-simscape';
outDir = 'D:/NPU/Matlab_code/output/page2_trajectory_planning';
workDir = 'D:/NPU/Matlab_code/.codex_work/page2_trajectory_planning';
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'opt_minimal', 'core'));
if ~exist(outDir, 'dir'), mkdir(outDir); end

referenceFile = fullfile(rootDir, 'opt_minimal', 'examples', ...
    'ihsid_40x20_limited_memory', 'simscape_references.mat');
pidFile = fullfile(rootDir, 'results', 'controller', ...
    'result_simscape_pose_length_control_20260829_114607.mat');
baselineFile = fullfile(outDir, 'page2_pose_pid_ik_baseline_references.mat');
referenceData = load(referenceFile, 'refs', 'denseReport');
baselineData = load(baselineFile, 'refs');
pidData = load(pidFile, 'setup', 'report');
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);

ihsid = evaluateIhsidNodes(referenceData.refs, model, scene);
pidik = evaluatePidIkNodes(baselineData.refs, pidData.setup, pidData.report, model, scene);
[Mihsid, badIhsid] = hardMargins(ihsid, model, scene);
[Mpidik, badPidik] = hardMargins(pidik, model, scene);
cost = accelerationMetrics(pidik.Ldd, ihsid.Ldd);

summary = struct();
summary.referenceFile = referenceFile;
summary.baselineFile = baselineFile;
summary.pidFile = pidFile;
summary.nodeCount = numel(referenceData.refs.t);
summary.ihsid = ihsid;
summary.pidik = pidik;
summary.Mihsid = Mihsid;
summary.Mpidik = Mpidik;
summary.badIhsid = badIhsid;
summary.badPidik = badPidik;
summary.cost = cost;
summary.denseReport = referenceData.denseReport;
summary.controlDefinition = ['Stored Simscape pose-feedback length execution: platform pose-error feedback, ', ...
    'time-varying Jacobian mapping to leg-length correction, outer length P plus inner velocity PIDF, ideal length input.'];
summary.forceComparisonAvailable = false;
save(fullfile(outDir, 'page2_pidik_ihsid_comparison_data.mat'), 'summary', '-v7');

drawHeatmapPair(Mpidik, Mihsid, badPidik, badIhsid, outDir);
drawCostBars(cost, outDir);
writeMetricsCsv(summary, outDir);
writeNotes(summary, outDir);
writeSummary(summary, outDir);
fprintf('PAGE2_PIDIK_COMPARISON_OK N=%d pidBad=%d ihsidBad=%d meanPct=%.9f rmsPct=%.9f p95Pct=%.9f\n', ...
    summary.nodeCount, sum(badPidik), sum(badIhsid), cost.ihsidPercent.mean, ...
    cost.ihsidPercent.rms, cost.ihsidPercent.p95);
end

function data = evaluateIhsidNodes(refs, model, scene)
data.t = refs.t(:).'; data.Q = refs.q;
data.L = refs.L;
[data.Ld, data.Ldd] = finiteDiff(data.L, data.t(2)-data.t(1));
data.F = refs.Fleg;
data = attachKinematics(data, model, scene);
end

function data = evaluatePidIkNodes(refs, setup, report, model, scene)
t = refs.t(:).';
poseErr = interp1(report.poseTime, report.poseError, t(:), 'linear', 'extrap').';
data.t = t;
data.Q = refs.q - poseErr;
relativeLength = interp1(report.time, report.Lactual, t(:), 'linear', 'extrap').';
data.L = relativeLength + setup.refs.Lref(:, 1);
[data.Ld, data.Ldd] = finiteDiff(data.L, t(2)-t(1));
data.F = nan(6, numel(t));
data = attachKinematics(data, model, scene);
end

function data = attachKinematics(data, model, scene)
n = numel(data.t);
data.sigma = nan(1,n); data.cond = nan(1,n); data.clearance = nan(3,n); data.ikFailed = false(1,n);
for k = 1:n
    try
        J = sgpJacobian(data.Q(:,k), model);
        data.sigma(k) = J.sigmaMin;
        data.cond(k) = J.condJ;
        data.clearance(:,k) = evaluateCylinderBoxDistanceNumeric(data.Q(:,k), scene).distances(:);
        check = sgpIK(data.Q(:,k), model);
        if any(~isfinite(check.L)), data.ikFailed(k) = true; end
    catch
        data.ikFailed(k) = true;
    end
end
end

function [M, bad] = hardMargins(data, model, scene)
n = numel(data.t); collision = nan(1,n);
for k = 1:n
    if data.t(k) <= 5 + 1e-12
        roofMin = scene.collision.stage1ConstraintDistance;
    else
        roofMin = scene.collision.finalGap;
    end
    roofM = (data.clearance(1,k) - roofMin) / max(roofMin,1e-6);
    sideM = min((data.clearance(2:3,k) - scene.collision.safeDistance) ./ scene.collision.safeDistance);
    collision(k) = min(roofM, sideM);
end
sigM = (data.sigma - model.singularity.sigmaMinSafe) / model.singularity.sigmaMinSafe;
condM = (model.singularity.condWarning - data.cond) / model.singularity.condWarning;
lenM = min([(data.L-model.lmin)./(model.lmax-model.lmin); ...
    (model.lmax-data.L)./(model.lmax-model.lmin)], [], 1);
velM = min((model.actuator.ldotMax - abs(data.Ld)) ./ model.actuator.ldotMax, [], 1);
accM = min((model.actuator.lddotMax - abs(data.Ldd)) ./ model.actuator.lddotMax, [], 1);
M = [collision; sigM; condM; lenM; velM; accM];
bad = data.ikFailed | any(M < -1e-9, 1) | any(~isfinite(M), 1);
end

function cost = accelerationMetrics(pidAcceleration, ihsidAcceleration)
pid = [mean(abs(pidAcceleration), 'all'), sqrt(mean(pidAcceleration.^2, 'all')), ...
    prctile(abs(pidAcceleration(:)), 95)];
ihsid = [mean(abs(ihsidAcceleration), 'all'), sqrt(mean(ihsidAcceleration.^2, 'all')), ...
    prctile(abs(ihsidAcceleration(:)), 95)];
cost.names = {'Mean |a|','RMS(a)','P95(|a|)'};
cost.pidik = pid;
cost.ihsid = ihsid;
cost.ihsidPercent.mean = 100 * ihsid(1) / pid(1);
cost.ihsidPercent.rms = 100 * ihsid(2) / pid(2);
cost.ihsidPercent.p95 = 100 * ihsid(3) / pid(3);
cost.improvement = 100 * (1 - ihsid ./ pid);
end

function drawHeatmapPair(Mpidik, Mihsid, badPidik, badIhsid, outDir)
fig = figure('Color', 'w', 'Position', [100 100 980 360]);
tl = tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
labels = {'碰撞间隙 d_{col}','\sigma_{min}(J)','\kappa(J)','支链长度 l','支链速度 |l̇|','支链加速度 |l̈|'};
allMargins = {Mpidik, Mihsid}; names = {'Pose PID + IK','IHSID'}; counts = [sum(badPidik),sum(badIhsid)];
nameColors = {[0.9294 0.5451 0.2275],[0.0627 0.2431 0.4314]};
nodeProgress = linspace(0,100,size(Mpidik,2));
nodeEdges = [0, 0.5*(nodeProgress(1:end-1)+nodeProgress(2:end)), 100];
xEdges = taskAxisMap(nodeEdges);
tickProgress = [0 25 50 75 100];
tickPosition = taskAxisMap(tickProgress);
marginEdges = safetyMarginEdges();
for index = 1:2
    ax = nexttile(tl);
    hold(ax,'on');
    drawMarginCells(ax, xEdges, marginToBins(allMargins{index}, marginEdges));
    set(ax,'YDir','reverse','YTick',1:6,'YTickLabel',labels,'FontName','Microsoft YaHei', ...
        'FontSize',12,'TickLength',[0 0],'Box','off');
    xlim(ax,[0 1]); ylim(ax,[0.5 6.5]); xticks(ax,tickPosition); xticklabels(ax,{'0','25','50','75','100'});
    for lineIndex = 2:numel(tickPosition)-1
        xline(ax,tickPosition(lineIndex),'-','Color',[0.84 0.88 0.91],'LineWidth',0.6);
    end
    xlabel(ax,'任务进度 / %','FontName','Microsoft YaHei','FontSize',12);
    title(ax,names{index},'FontName','Aptos','FontSize',17,'FontWeight','bold','Color',nameColors{index});
    colormap(ax,marginMap()); clim(ax,[0.5 numel(marginEdges)-0.5]);
    text(ax,0.025,0.76,sprintf('VIOLATION NODES  %d',counts(index)), ...
        'FontName','Aptos','FontSize',10,'FontWeight','bold','Color',nameColors{index});
end
cb = colorbar; cb.Layout.Tile = 'east'; cb.Label.String = '归一化安全裕度  m';
cb.Label.FontName = 'Microsoft YaHei'; cb.Label.FontSize = 12;
cb.Ticks = [1 2 4 6 8 10 12 13];
cb.TickLabels = {'m<0 违规','0–0.05','0.1–0.2','0.4–0.6','0.8–1.0','1.5–2.0','3.0–4.5','≥4.5 远离'};
cb.FontSize = 8.5;
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.png'),'Resolution',260);
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.pdf'),'ContentType','vector');
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.svg'),'ContentType','vector');
close(fig);
end

function drawCostBars(cost,outDir)
fig = figure('Color','w','Position',[100 100 820 360]);
ax = axes(fig,'Position',[0.13 0.18 0.80 0.68]);
percent = [cost.ihsidPercent.mean cost.ihsidPercent.rms cost.ihsidPercent.p95];
values = [100*ones(1,3);percent].';
b = bar(ax,values,'grouped','BarWidth',0.70);
b(1).FaceColor=[0.9294 0.5451 0.2275]; b(2).FaceColor=[0.0627 0.2431 0.4314];
b(1).EdgeColor='none'; b(2).EdgeColor='none';
ylim(ax,[0 110]); yticks(ax,0:25:100); yticklabels(ax,{'0%','25%','50%','75%','100%'});
grid(ax,'on'); box(ax,'off'); ax.YGrid='on'; ax.XGrid='off'; ax.GridAlpha=0.22; ax.GridColor=[0.55 0.60 0.66];
set(ax,'XTickLabel',cost.names,'FontName','Aptos','FontSize',14,'LineWidth',1);
ylabel(ax,'相对代价指标 / %','FontName','Microsoft YaHei','FontSize',14);
text(ax,-0.60,104,'↓ LOWER','FontName','Aptos','FontSize',11,'FontWeight','bold','Color',[0.35 0.42 0.50]);
for index = 1:3
    delta = 100-percent(index);
    text(ax,index+0.15,percent(index)+3.2,sprintf('%.1f%%',percent(index)), ...
        'HorizontalAlignment','center','FontName','Aptos','FontSize',16,'FontWeight','bold','Color',[0.0627 0.2431 0.4314]);
    text(ax,index+0.15,percent(index)+11.5,sprintf('↓%.1f%%',delta), ...
        'HorizontalAlignment','center','FontName','Aptos','FontSize',12.5,'FontWeight','bold','Color',[0.0627 0.2431 0.4314]);
end
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.png'),'Resolution',260);
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.pdf'),'ContentType','vector');
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.svg'),'ContentType','vector');
close(fig);
end

function cmap = marginMap()
% 13 discrete levels are based on the observed combined range m=[-0.0247,
% 5.9099]: one violation level plus twelve safe-margin levels.
cmap = [
    0.70 0.04 0.08;  % m<0, violation
    0.02 0.15 0.30;  % 0–0.05, nearest safe edge
    0.05 0.23 0.41;
    0.10 0.32 0.52;
    0.17 0.41 0.61;
    0.26 0.50 0.68;
    0.36 0.59 0.74;
    0.47 0.67 0.80;
    0.58 0.75 0.85;
    0.68 0.82 0.90;
    0.77 0.88 0.94;
    0.86 0.93 0.97;
    0.94 0.97 0.99]; % m≥4.5, far from every boundary
end

function drawMarginCells(ax, xEdges, margin)
for rowIndex = 1:size(margin,1)
    for columnIndex = 1:size(margin,2)
        patch(ax, [xEdges(columnIndex), xEdges(columnIndex+1), xEdges(columnIndex+1), xEdges(columnIndex)], ...
            [rowIndex-0.5, rowIndex-0.5, rowIndex+0.5, rowIndex+0.5], margin(rowIndex,columnIndex), ...
            'EdgeColor','none','FaceColor','flat');
    end
end
end

function edges = safetyMarginEdges()
% Lower bins are intentionally tighter because every small increment close to
% a hard bound matters more.  Upper bins still span the full checked maximum.
edges = [-Inf, 0, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00, 1.50, 2.00, 3.00, 4.50, Inf];
end

function bins = marginToBins(margin, edges)
bins = ones(size(margin));
for edgeIndex = 2:numel(edges)-1
    bins(margin >= edges(edgeIndex)) = edgeIndex;
end
end

function x = taskAxisMap(progress)
progress = max(0,min(100,progress)); x = zeros(size(progress));
mask = progress <= 25;
x(mask) = 0.10 * progress(mask) / 25;
mask = progress > 25 & progress <= 50;
x(mask) = 0.10 + 0.20 * (progress(mask)-25) / 25;
mask = progress > 50 & progress <= 75;
x(mask) = 0.30 + 0.30 * (progress(mask)-50) / 25;
mask = progress > 75;
x(mask) = 0.60 + 0.40 * (progress(mask)-75) / 25;
end

function [d1,d2] = finiteDiff(x,h)
n=size(x,2); d1=zeros(size(x)); d2=zeros(size(x));
d1(:,1)=(-3*x(:,1)+4*x(:,2)-x(:,3))/(2*h); d1(:,end)=(3*x(:,end)-4*x(:,end-1)+x(:,end-2))/(2*h);
d2(:,1)=(2*x(:,1)-5*x(:,2)+4*x(:,3)-x(:,4))/(h^2); d2(:,end)=(2*x(:,end)-5*x(:,end-1)+4*x(:,end-2)-x(:,end-3))/(h^2);
d1(:,2:end-1)=(x(:,3:end)-x(:,1:end-2))/(2*h);
d2(:,2:end-1)=(x(:,3:end)-2*x(:,2:end-1)+x(:,1:end-2))/(h^2);
end

function writeSummary(s, outDir)
fid = fopen(fullfile(outDir, 'page2_pidik_ihsid_recomputed.txt'), 'w');
clean = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'referenceFile: %s\n', s.referenceFile);
fprintf(fid, 'baselineReferenceFile: %s\n', s.baselineFile);
fprintf(fid, 'pidExecutionFile: %s\n', s.pidFile);
fprintf(fid, 'nodeCount: %d\n', s.nodeCount);
fprintf(fid, 'pidikViolationNodes: %d\n', sum(s.badPidik));
fprintf(fid, 'ihsidViolationNodes: %d\n', sum(s.badIhsid));
fprintf(fid, 'pidikMeanAbsAcceleration: %.12g\n', s.cost.pidik(1));
fprintf(fid, 'ihsidMeanAbsAcceleration: %.12g\n', s.cost.ihsid(1));
fprintf(fid, 'ihsidMeanPercent: %.12g\n', s.cost.ihsidPercent.mean);
fprintf(fid, 'pidikRmsAcceleration: %.12g\n', s.cost.pidik(2));
fprintf(fid, 'ihsidRmsAcceleration: %.12g\n', s.cost.ihsid(2));
fprintf(fid, 'ihsidRmsPercent: %.12g\n', s.cost.ihsidPercent.rms);
fprintf(fid, 'pidikP95AbsAcceleration: %.12g\n', s.cost.pidik(3));
fprintf(fid, 'ihsidP95AbsAcceleration: %.12g\n', s.cost.ihsid(3));
fprintf(fid, 'ihsidP95Percent: %.12g\n', s.cost.ihsidPercent.p95);
fprintf(fid, 'forceComparisonAvailable: %d\n', s.forceComparisonAvailable);
fprintf(fid, 'controlDefinition: %s\n', s.controlDefinition);
end

function writeMetricsCsv(s, outDir)
metrics = {
    'minimum_collision_clearance_ihsid_dense', s.denseReport.minClearance, 'm', 'accepted IHSID dense post-check';
    'minimum_sigma_ihsid_dense', s.denseReport.minSigmaMin, '-', 'accepted IHSID dense post-check';
    'maximum_condition_number_ihsid_dense', s.denseReport.maxCondJ, '-', 'accepted IHSID dense post-check';
    'maximum_leg_speed_ihsid_dense', s.denseReport.maxAbsLd, 'm/s', 'accepted IHSID dense post-check';
    'maximum_leg_acceleration_ihsid_dense', s.denseReport.maxAbsLdd, 'm/s^2', 'accepted IHSID dense post-check';
    'minimum_collision_clearance_pose_pid_ik_node', min(s.pidik.clearance(1,:)), 'm', 'actual Simscape execution; 61-node task grid';
    'minimum_sigma_pose_pid_ik_node', min(s.pidik.sigma), '-', 'actual Simscape execution; 61-node task grid';
    'maximum_condition_number_pose_pid_ik_node', max(s.pidik.cond), '-', 'actual Simscape execution; 61-node task grid';
    'maximum_leg_speed_pose_pid_ik_node', max(abs(s.pidik.Ld), [], 'all'), 'm/s', 'actual Simscape execution; 61-node finite difference';
    'maximum_leg_acceleration_pose_pid_ik_node', max(abs(s.pidik.Ldd), [], 'all'), 'm/s^2', 'actual Simscape execution; 61-node finite difference';
    'pose_pid_ik_violation_nodes', sum(s.badPidik), 'nodes', '61 common task nodes; six logged kinematic hard constraints';
    'ihsid_violation_nodes', sum(s.badIhsid), 'nodes', '61 common task nodes; six logged kinematic hard constraints';
    'mean_abs_accel_pose_pid_ik', s.cost.pidik(1), 'm/s^2', 'six legs x 61 nodes; actual Simscape execution';
    'mean_abs_accel_ihsid', s.cost.ihsid(1), 'm/s^2', 'six legs x 61 nodes; accepted IHSID reference';
    'mean_abs_accel_ihsid_percent_of_pose_pid_ik', s.cost.ihsidPercent.mean, '%', 'Pose PID + IK = 100%';
    'rms_accel_pose_pid_ik', s.cost.pidik(2), 'm/s^2', 'six legs x 61 nodes; actual Simscape execution';
    'rms_accel_ihsid', s.cost.ihsid(2), 'm/s^2', 'six legs x 61 nodes; accepted IHSID reference';
    'rms_accel_ihsid_percent_of_pose_pid_ik', s.cost.ihsidPercent.rms, '%', 'Pose PID + IK = 100%';
    'p95_abs_accel_pose_pid_ik', s.cost.pidik(3), 'm/s^2', 'six legs x 61 nodes; actual Simscape execution';
    'p95_abs_accel_ihsid', s.cost.ihsid(3), 'm/s^2', 'six legs x 61 nodes; accepted IHSID reference';
    'p95_abs_accel_ihsid_percent_of_pose_pid_ik', s.cost.ihsidPercent.p95, '%', 'Pose PID + IK = 100%';
    'force_comparison', NaN, 'N/A', 'not logged by the ideal-length Pose PID + IK run; intentionally excluded from C/D'};
T = cell2table(metrics,'VariableNames',{'metric','value','unit','definition'});
writetable(T,fullfile(outDir,'page2_metrics_summary.csv'),'Encoding','UTF-8');
end

function writeNotes(s, outDir)
fid = fopen(fullfile(outDir,'page2_notes.md'),'w'); clean = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'# Page 2 — data definition\n\n');
fprintf(fid,'- **A** uses the IHSID optimization formulation. Force-rate remains an objective term, not a hard constraint.\n');
fprintf(fid,'- **B** is redrawn from the project''s MATLAB geometry: `buildCylinderBoxTransferScene`, with the start platform, complete target platform/cylinder, and the actual roof/side-skirt constrained hood. The base and lower legs are cropped out.\n');
fprintf(fid,'- **Pose PID + IK**: a two-stage Cartesian pose reference (`q0 → qWaypoint → qGoal`) with SLERP orientation and phase-wise quintic timing; it is executed by the stored Simscape position/length cascade (pose-error feedback → time-varying Jacobian mapping → outer length P + inner velocity PIDF) on ideal leg-length inputs. Its 61-node metrics use the actual execution trace.\n');
fprintf(fid,'- **IHSID**: accepted `ihsid_40x20_limited_memory` 61-node reference, evaluated on the same task grid.\n');
fprintf(fid,'- **C** uses six mutually available kinematic hard-constraint margins. `m<0` is a violation, `m=0` the boundary, and `m>0` safe. Pose PID + IK has %d violating node(s); IHSID has %d. Force is not logged by the ideal-length PID execution and is therefore excluded rather than inferred.\n',sum(s.badPidik),sum(s.badIhsid));
fprintf(fid,'- The single Pose PID + IK violation is terminal collision clearance %.3f mm against the 5.000 mm terminal boundary.\n',min(s.pidik.clearance(1,:))*1e3);
fprintf(fid,'- **D** uses all six legs × 61 nodes. IHSID relative to Pose PID + IK: Mean |a| %.2f%%, RMS(a) %.2f%%, P95(|a|) %.2f%%.\n',s.cost.ihsidPercent.mean,s.cost.ihsidPercent.rms,s.cost.ihsidPercent.p95);
fprintf(fid,'- Bottom index retains accepted IHSID dense post-check values: d_min %.1f mm terminal, sigma_min %.3f, kappa_max %.2f, v_max %.3f m/s, a_max %.3f m/s^2.\n',s.denseReport.minClearance*1e3,s.denseReport.minSigmaMin,s.denseReport.maxCondJ,s.denseReport.maxAbsLd,s.denseReport.maxAbsLdd);
end
