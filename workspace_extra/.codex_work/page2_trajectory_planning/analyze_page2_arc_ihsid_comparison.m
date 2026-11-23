function analyze_page2_arc_ihsid_comparison()
% PAGE 2: fixed quadratic-Bezier arc + pointwise IK versus accepted IHSID.
% The baseline contains no obstacle, singularity, or actuator correction.

set(groot, 'defaultFigureVisible', 'off');
rootDir = 'D:/NPU/Matlab_code/stewart-simscape';
outDir = 'D:/NPU/Matlab_code/output/page2_trajectory_planning';
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'opt_minimal', 'core'));
if ~exist(outDir, 'dir'), mkdir(outDir); end

referenceFile = fullfile(rootDir, 'opt_minimal', 'examples', ...
    'ihsid_40x20_limited_memory', 'simscape_references.mat');
referenceData = load(referenceFile, 'refs', 'denseReport');
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
constraintPolicy = struct('accelerationLimit', 0.25);

baseline = buildFixedArcBaseline(model, scene, referenceData.refs.t(:).');
ihsid = buildIhsidData(referenceData.refs, model, scene);
[Mbase, badBase] = hardMargins(baseline, model, scene, constraintPolicy);
[Mihsid, badIhsid] = hardMargins(ihsid, model, scene, constraintPolicy);
cost = trajectoryCostMetrics(baseline, ihsid);

summary = struct();
summary.referenceFile = referenceFile;
summary.nodeCount = numel(referenceData.refs.t);
summary.duration = referenceData.refs.t(end) - referenceData.refs.t(1);
summary.controlPointOffset = scene.box.size(1);
summary.constraintPolicy = constraintPolicy;
summary.baseline = baseline;
summary.ihsid = ihsid;
summary.Mbase = Mbase;
summary.Mihsid = Mihsid;
summary.badBase = badBase;
summary.badIhsid = badIhsid;
summary.cost = cost;
summary.denseReport = referenceData.denseReport;
summary.baselineDefinition = ['Single quadratic Bezier arc from start to goal; ', ...
    'control point is the midpoint plus the 0.8 m scene-box long-side height; ', ...
    'quintic smooth timing and quaternion SLERP; pointwise analytical IK only.'];
save(fullfile(outDir, 'page2_arc_ihsid_comparison_data.mat'), 'summary', '-v7');

drawHeatmapPair(Mbase, Mihsid, baseline.ikFailed, ihsid.ikFailed, outDir);
drawCostBars(cost, outDir);
writeMetricsCsv(summary, outDir);
writeNotes(summary, outDir);
writeSummary(summary, outDir);

fprintf(['PAGE2_ARC_IHSID_COMPARISON_OK N=%d baselineBad=%d ihsidBad=%d ', ...
    'meanPct=%.9f rmsPct=%.9f p95Pct=%.9f forcePct=%.9f\n'], ...
    summary.nodeCount, sum(badBase), sum(badIhsid), cost.ihsidPercent.meanAccel, ...
    cost.ihsidPercent.rmsJerk, cost.ihsidPercent.legTravel, cost.ihsidPercent.deltaF);
end

function data = buildFixedArcBaseline(model, scene, t)
n = numel(t);
duration = t(end) - t(1);
u = (t - t(1)) / duration;
Q = zeros(6,n);
p0 = scene.q0(1:3); pGoal = scene.qGoal(1:3);
control = 0.5 * (p0 + pGoal) + [0; 0; scene.box.size(1)];
qR0 = rotmToQuat(rpy2rotmZYX(scene.q0(4:6)));
qRg = rotmToQuat(rpy2rotmZYX(scene.qGoal(4:6)));
for k = 1:n
    s = quinticProgress(u(k));
    p = (1-s)^2 * p0 + 2 * (1-s) * s * control + s^2 * pGoal;
    Q(:,k) = [p; rotmToRpyZYX(quatToRotm(quatSlerp(qR0, qRg, s)))];
end
[Qd,Qdd] = finiteDiff(Q, t(2)-t(1));
data = evaluateNodeSeries(t, Q, Qd, Qdd, model, scene, true);
data.name = '固定弧线 + 逐点逆解';
data.controlPoint = control;
end

function data = buildIhsidData(refs, model, scene)
t = refs.t(:).'; Q = refs.q;
[Qd,Qdd] = finiteDiff(Q, t(2)-t(1));
data = evaluateNodeSeries(t, Q, Qd, Qdd, model, scene, false);
data.F = refs.Fleg;
end

function data = evaluateNodeSeries(t, Q, Qd, Qdd, model, scene, useInverseDynamics)
n = numel(t);
data = struct('t',t,'Q',Q,'Qd',Qd,'Qdd',Qdd, ...
    'L',nan(6,n),'Ld',nan(6,n),'Ldd',nan(6,n),'F',nan(6,n), ...
    'sigma',nan(1,n),'cond',nan(1,n),'clearance',nan(3,n),'ikFailed',false(1,n));
for k = 1:n
    try
        kin = sgpIK(Q(:,k), model);
        if any(~isfinite(kin.L)), error('Non-finite IK result'); end
        data.L(:,k) = kin.L;
        J = sgpJacobian(Q(:,k), model);
        data.sigma(k) = J.sigmaMin;
        data.cond(k) = J.condJ;
        clearance = evaluateCylinderBoxDistanceNumeric(Q(:,k), scene);
        data.clearance(:,k) = clearance.distances(:);
        if useInverseDynamics
            data.F(:,k) = inverseDynamicsCompositeRigidBody(Q(:,k), Qd(:,k), Qdd(:,k), model);
        end
    catch
        data.ikFailed(k) = true;
    end
end
[data.Ld,data.Ldd] = finiteDiff(data.L, t(2)-t(1));
end

function [M, bad] = hardMargins(data, model, scene, constraintPolicy)
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
accM = min((constraintPolicy.accelerationLimit - abs(data.Ldd)) ./ constraintPolicy.accelerationLimit, [], 1);
forceM = min([(data.F-model.actuator.forceMin)./(model.actuator.forceMax-model.actuator.forceMin); ...
    (model.actuator.forceMax-data.F)./(model.actuator.forceMax-model.actuator.forceMin)], [], 1);
M = [collision; sigM; condM; lenM; velM; accM; forceM];
bad = data.ikFailed | any(M < -1e-9, 1) | any(~isfinite(M), 1);
end

function cost = trajectoryCostMetrics(base, ihsid)
baseAccel = [mean(abs(base.Ldd), 'all'), sqrt(mean(base.Ldd.^2, 'all')), ...
    prctile(abs(base.Ldd(:)),95)];
ihsidAccel = [mean(abs(ihsid.Ldd), 'all'), sqrt(mean(ihsid.Ldd.^2, 'all')), ...
    prctile(abs(ihsid.Ldd(:)),95)];
[baseJerk,~] = finiteDiff(base.Ldd, base.t(2)-base.t(1));
[ihsidJerk,~] = finiteDiff(ihsid.Ldd, ihsid.t(2)-ihsid.t(1));
baseLegTravel = sum(abs(diff(base.L,1,2)), 'all');
ihsidLegTravel = sum(abs(diff(ihsid.L,1,2)), 'all');
baseForceRate = sqrt(mean(diff(base.F,1,2).^2, 'all'));
ihsidForceRate = sqrt(mean(diff(ihsid.F,1,2).^2, 'all'));
cost.names = {'平均加速度','加加速度均方根','累计支链行程'};
cost.baseAccel = baseAccel;
cost.ihsidAccel = ihsidAccel;
cost.baseJerk = baseJerk;
cost.ihsidJerk = ihsidJerk;
cost.baseRmsJerk = sqrt(mean(baseJerk.^2, 'all'));
cost.ihsidRmsJerk = sqrt(mean(ihsidJerk.^2, 'all'));
cost.baseLegTravel = baseLegTravel;
cost.ihsidLegTravel = ihsidLegTravel;
cost.baseForceRate = baseForceRate;
cost.ihsidForceRate = ihsidForceRate;
cost.ihsidPercent.meanAccel = 100 * ihsidAccel(1) / baseAccel(1);
cost.ihsidPercent.rmsJerk = 100 * cost.ihsidRmsJerk / cost.baseRmsJerk;
cost.ihsidPercent.legTravel = 100 * ihsidLegTravel / baseLegTravel;
cost.ihsidPercent.deltaF = 100 * ihsidForceRate / baseForceRate;
cost.improvement = 100 * (1 - [cost.ihsidPercent.meanAccel, cost.ihsidPercent.rmsJerk, cost.ihsidPercent.legTravel] / 100);
end

function drawHeatmapPair(Mbase, Mihsid, ikBase, ikIhsid, outDir)
fig = figure('Color','w','Position',[100 100 1040 420]);
tl = tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
labels = {'障碍净距','奇异裕度','机构稳定性','支链行程','支链速度','支链加速度','驱动力'};
allMargins = {Mbase, Mihsid}; allIkFail = {ikBase, ikIhsid};
names = {'二次贝塞尔','IHSID'};
nameColors = {[0.06 0.24 0.43],[0.06 0.24 0.43]};
nodeProgress = linspace(0,100,size(Mbase,2));
nodeEdges = [0, 0.5*(nodeProgress(1:end-1)+nodeProgress(2:end)), 100];
xEdges = taskAxisMap(nodeEdges);
tickProgress = [20 40 60 80 100]; tickPosition = taskAxisMap(tickProgress);
marginEdges = safetyMarginEdges();
for a = 1:2
    ax = nexttile(tl); hold(ax,'on');
    drawMarginCells(ax, xEdges, marginToBins(allMargins{a}, marginEdges));
    for failedIndex = find(allIkFail{a})
        for rowIndex = 1:7
            x0 = 0.5 * (xEdges(failedIndex) + xEdges(failedIndex+1));
            plot(ax,x0,rowIndex,'x','Color','w','MarkerSize',7,'LineWidth',1.4);
        end
    end
    set(ax,'YDir','reverse','YTick',1:7,'YTickLabel',labels,'FontName','Microsoft YaHei', ...
        'FontSize',11.5,'TickLength',[0 0],'Box','off','Layer','top');
    xlim(ax,[0 1]); ylim(ax,[0.5 7.5]); xticks(ax,tickPosition); xticklabels(ax,{'20','40','60','80','100'});
    for lineIndex = 1:numel(tickPosition)-1
        xline(ax,tickPosition(lineIndex),'-','Color',[0.84 0.88 0.91],'LineWidth',0.55);
    end
    xlabel(ax,'任务进度 / %','FontName','Microsoft YaHei','FontSize',12);
    title(ax,names{a},'FontName','Microsoft YaHei','FontSize',15,'FontWeight','bold','Color',nameColors{a});
    colormap(ax,marginMap()); clim(ax,[0.5 13.5]);
end
cb = colorbar; cb.Layout.Tile = 'east'; cb.Label.String = '安全裕度';
cb.Label.FontName = 'Microsoft YaHei'; cb.Label.FontSize = 11.5;
cb.Ticks = [1 2 4 6 8 10 12 13];
cb.TickLabels = {'违规','贴近边界','低裕度','中低裕度','中裕度','高裕度','充分裕度','远离边界'};
cb.FontName = 'Microsoft YaHei'; cb.FontSize = 8.5;
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.png'),'Resolution',260);
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.pdf'),'ContentType','vector');
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.svg'),'ContentType','vector');
close(fig);
end

function drawCostBars(cost,outDir)
fig = figure('Color','w','Position',[100 100 820 360]);
ax = axes(fig,'Position',[0.14 0.19 0.80 0.67]);
percent = [cost.ihsidPercent.meanAccel cost.ihsidPercent.rmsJerk cost.ihsidPercent.legTravel];
values = [100*ones(1,3); percent].';
b = bar(ax,values,'grouped','BarWidth',0.70);
b(1).FaceColor=[0.95 0.68 0.42]; b(2).FaceColor=[0.06 0.24 0.43];
b(1).EdgeColor='none'; b(2).EdgeColor='none';
ylim(ax,[0 110]); yticks(ax,0:25:100); yticklabels(ax,{'0%','25%','50%','75%','100%'});
grid(ax,'on'); box(ax,'off'); ax.YGrid='on'; ax.XGrid='off'; ax.GridAlpha=0.24; ax.GridColor=[0.60 0.65 0.70];
set(ax,'XTickLabel',cost.names,'FontName','Microsoft YaHei','FontSize',13,'LineWidth',0.9);
ylabel(ax,'相对执行代价 / %','FontName','Microsoft YaHei','FontSize',13);
text(ax,-0.59,103,'↓  越低越好','FontName','Microsoft YaHei','FontSize',10.5,'FontWeight','bold','Color',[0.35 0.42 0.50]);
for k = 1:3
    text(ax,k+0.15,percent(k)+13.5,sprintf('%.1f%%',percent(k)), ...
        'HorizontalAlignment','center','FontName','Aptos','FontSize',15,'FontWeight','bold','Color',[0.06 0.24 0.43]);
end
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.png'),'Resolution',260);
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.pdf'),'ContentType','vector');
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.svg'),'ContentType','vector');
close(fig);
end

function cmap = marginMap()
% 13 discrete levels: one red violation bin, then a navy-to-pale-blue safe scale.
cmap = [
    0.74 0.05 0.08;
    0.03 0.14 0.28;
    0.05 0.21 0.37;
    0.08 0.29 0.47;
    0.12 0.37 0.56;
    0.19 0.45 0.64;
    0.28 0.53 0.70;
    0.38 0.61 0.76;
    0.48 0.69 0.82;
    0.59 0.76 0.87;
    0.69 0.83 0.92;
    0.80 0.89 0.96;
    0.91 0.95 0.98];
end

function drawMarginCells(ax, xEdges, marginBins)
for rowIndex = 1:size(marginBins,1)
    for columnIndex = 1:size(marginBins,2)
        patch(ax,[xEdges(columnIndex),xEdges(columnIndex+1),xEdges(columnIndex+1),xEdges(columnIndex)], ...
            [rowIndex-0.5,rowIndex-0.5,rowIndex+0.5,rowIndex+0.5],marginBins(rowIndex,columnIndex), ...
            'EdgeColor','none','FaceColor','flat');
    end
end
end

function edges = safetyMarginEdges()
edges = [-Inf,0,0.05,0.10,0.20,0.40,0.60,0.80,1.00,1.50,2.00,3.00,4.50,Inf];
end

function bins = marginToBins(margin, edges)
bins = ones(size(margin));
for edgeIndex = 2:numel(edges)-1
    bins(margin >= edges(edgeIndex)) = edgeIndex;
end
bins(~isfinite(margin)) = 1;
end

function x = taskAxisMap(progress)
progress = max(0,min(100,progress)); x = zeros(size(progress));
breaksP = [0 20 40 60 80 100]; breaksX = [0 .10 .25 .40 .60 1.00];
for k = 1:numel(breaksP)-1
    isFinal = k == numel(breaksP)-1;
    mask = progress >= breaksP(k) & (progress < breaksP(k+1) | (isFinal & progress <= breaksP(k+1)));
    x(mask) = breaksX(k) + (breaksX(k+1)-breaksX(k)) * ...
        (progress(mask)-breaksP(k)) / (breaksP(k+1)-breaksP(k));
end
end

function [d1,d2] = finiteDiff(x,h)
n=size(x,2); d1=zeros(size(x)); d2=zeros(size(x));
d1(:,1)=(-3*x(:,1)+4*x(:,2)-x(:,3))/(2*h); d1(:,end)=(3*x(:,end)-4*x(:,end-1)+x(:,end-2))/(2*h);
d2(:,1)=(2*x(:,1)-5*x(:,2)+4*x(:,3)-x(:,4))/(h^2); d2(:,end)=(2*x(:,end)-5*x(:,end-1)+4*x(:,end-2)-x(:,end-3))/(h^2);
d1(:,2:end-1)=(x(:,3:end)-x(:,1:end-2))/(2*h);
d2(:,2:end-1)=(x(:,3:end)-2*x(:,2:end-1)+x(:,1:end-2))/(h^2);
end

function s = quinticProgress(tau)
tau=max(0,min(1,tau)); s=10*tau^3-15*tau^4+6*tau^5;
end

function q = rotmToQuat(R)
tr=trace(R);
if tr>0
    s=sqrt(tr+1)*2; q=[0.25*s;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
elseif R(1,1)>R(2,2) && R(1,1)>R(3,3)
    s=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2; q=[(R(3,2)-R(2,3))/s;0.25*s;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
elseif R(2,2)>R(3,3)
    s=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2; q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;0.25*s;(R(2,3)+R(3,2))/s];
else
    s=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2; q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;0.25*s];
end
q=q/norm(q);
end

function q = quatSlerp(q0,q1,s)
q0=q0(:); q1=q1(:); d=dot(q0,q1);
if d<0, q1=-q1; d=-d; end
if d>0.9995, q=(1-s)*q0+s*q1; q=q/norm(q); return; end
theta=acos(max(-1,min(1,d))); q=(sin((1-s)*theta)*q0+sin(s*theta)*q1)/sin(theta);
end

function R = quatToRotm(q)
q=q/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
R=[1-2*(y^2+z^2),2*(x*y-z*w),2*(x*z+y*w); ...
   2*(x*y+z*w),1-2*(x^2+z^2),2*(y*z-x*w); ...
   2*(x*z-y*w),2*(y*z+x*w),1-2*(x^2+y^2)];
end

function rpy = rotmToRpyZYX(R)
pitch=asin(max(-1,min(1,-R(3,1)))); roll=atan2(R(3,2),R(3,3)); yaw=atan2(R(2,1),R(1,1));
rpy=[roll;pitch;yaw];
end

function writeSummary(s,outDir)
fid=fopen(fullfile(outDir,'page2_arc_ihsid_recomputed.txt'),'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'reference_file: %s\n',s.referenceFile);
fprintf(fid,'node_count: %d\n',s.nodeCount);
fprintf(fid,'duration_s: %.6f\n',s.duration);
fprintf(fid,'arc_control_offset_m: %.6f\n',s.controlPointOffset);
fprintf(fid,'heatmap_acceleration_limit_mps2: %.6f\n',s.constraintPolicy.accelerationLimit);
fprintf(fid,'baseline_violation_nodes: %d\n',sum(s.badBase));
fprintf(fid,'ihsid_violation_nodes: %d\n',sum(s.badIhsid));
fprintf(fid,'baseline_min_signed_clearance_m: %.12g\n',min(s.baseline.clearance(1,:)));
fprintf(fid,'baseline_min_sigma: %.12g\n',min(s.baseline.sigma));
fprintf(fid,'baseline_max_condition: %.12g\n',max(s.baseline.cond));
fprintf(fid,'baseline_max_leg_speed_mps: %.12g\n',max(abs(s.baseline.Ld),[],'all'));
fprintf(fid,'baseline_max_leg_accel_mps2: %.12g\n',max(abs(s.baseline.Ldd),[],'all'));
fprintf(fid,'baseline_max_force_N: %.12g\n',max(abs(s.baseline.F),[],'all'));
fprintf(fid,'ihsid_mean_accel_percent: %.12g\n',s.cost.ihsidPercent.meanAccel);
fprintf(fid,'ihsid_rms_jerk_percent: %.12g\n',s.cost.ihsidPercent.rmsJerk);
fprintf(fid,'ihsid_total_leg_travel_percent: %.12g\n',s.cost.ihsidPercent.legTravel);
fprintf(fid,'ihsid_force_rate_percent: %.12g\n',s.cost.ihsidPercent.deltaF);
end

function writeMetricsCsv(s,outDir)
definition = '同场景、同任务时长、同一 61 节点；固定弧线为单段二次贝塞尔弧线加逐点解析逆解';
metrics = {
    'baseline_method','固定弧线 + 逐点逆解','-', '起点至目标点的单段二次贝塞尔上拱；控制点为中点上方 0.8 m；姿态平滑插值；不加入约束修正';
    'common_node_count',s.nodeCount,'nodes',definition;
    'common_duration',s.duration,'s',definition;
    'heatmap_acceleration_limit',s.constraintPolicy.accelerationLimit,'m/s^2','C 区左右统一采用的收紧支链加速度工程上限';
    'baseline_violation_nodes',sum(s.badBase),'nodes','七类硬约束的节点级检查；逆解失败亦计入';
    'ihsid_violation_nodes',sum(s.badIhsid),'nodes','七类硬约束的节点级检查；IHSID 参考保持不变';
    'baseline_minimum_signed_collision_clearance',min(s.baseline.clearance(1,:)),'m','61 节点几何距离复算；负值为穿透';
    'baseline_minimum_sigma',min(s.baseline.sigma),'-','61 节点 Jacobian 复算';
    'baseline_maximum_condition_number',max(s.baseline.cond),'-','61 节点 Jacobian 复算';
    'baseline_maximum_leg_speed',max(abs(s.baseline.Ld),[],'all'),'m/s','六支链、61 节点有限差分';
    'baseline_maximum_leg_acceleration',max(abs(s.baseline.Ldd),[],'all'),'m/s^2','六支链、61 节点有限差分';
    'baseline_maximum_force',max(abs(s.baseline.F),[],'all'),'N','逐节点复算逆动力学驱动力';
    'ihsid_minimum_collision_clearance_dense',s.denseReport.minClearance,'m','IHSID 已验收的密集后验检查';
    'ihsid_minimum_sigma_dense',s.denseReport.minSigmaMin,'-','IHSID 已验收的密集后验检查';
    'ihsid_maximum_condition_number_dense',s.denseReport.maxCondJ,'-','IHSID 已验收的密集后验检查';
    'ihsid_maximum_leg_speed_dense',s.denseReport.maxAbsLd,'m/s','IHSID 已验收的密集后验检查';
    'ihsid_maximum_leg_acceleration_dense',s.denseReport.maxAbsLdd,'m/s^2','IHSID 已验收的密集后验检查';
    'baseline_mean_absolute_acceleration',s.cost.baseAccel(1),'m/s^2','六支链、61 节点';
    'ihsid_mean_absolute_acceleration',s.cost.ihsidAccel(1),'m/s^2','六支链、61 节点';
    'ihsid_mean_absolute_acceleration_percent',s.cost.ihsidPercent.meanAccel,'%','固定弧线 + 逐点逆解 = 100%';
    'baseline_rms_acceleration',s.cost.baseAccel(2),'m/s^2','六支链、61 节点';
    'ihsid_rms_acceleration',s.cost.ihsidAccel(2),'m/s^2','六支链、61 节点';
    'ihsid_rms_acceleration_percent',100*s.cost.ihsidAccel(2)/s.cost.baseAccel(2),'%','固定弧线 + 逐点逆解 = 100%；未单独绘制';
    'baseline_p95_absolute_acceleration',s.cost.baseAccel(3),'m/s^2','六支链、61 节点';
    'ihsid_p95_absolute_acceleration',s.cost.ihsidAccel(3),'m/s^2','六支链、61 节点';
    'ihsid_p95_absolute_acceleration_percent',100*s.cost.ihsidAccel(3)/s.cost.baseAccel(3),'%','固定弧线 + 逐点逆解 = 100%；未单独绘制';
    'baseline_rms_jerk',s.cost.baseRmsJerk,'m/s^3','六支链加加速度 RMS，61 节点有限差分';
    'ihsid_rms_jerk',s.cost.ihsidRmsJerk,'m/s^3','六支链加加速度 RMS，61 节点有限差分';
    'ihsid_rms_jerk_percent',s.cost.ihsidPercent.rmsJerk,'%','固定弧线 + 逐点逆解 = 100%';
    'baseline_total_leg_travel',s.cost.baseLegTravel,'m','六支链在全程的累计绝对长度变化';
    'ihsid_total_leg_travel',s.cost.ihsidLegTravel,'m','六支链在全程的累计绝对长度变化';
    'ihsid_total_leg_travel_percent',s.cost.ihsidPercent.legTravel,'%','固定弧线 + 逐点逆解 = 100%';
    'baseline_rms_force_change',s.cost.baseForceRate,'N','相邻节点力差的 RMS；单独保留，不进入 D 区';
    'ihsid_rms_force_change',s.cost.ihsidForceRate,'N','相邻节点力差的 RMS；单独保留，不进入 D 区';
    'ihsid_rms_force_change_percent',s.cost.ihsidPercent.deltaF,'%','固定弧线 + 逐点逆解 = 100%；不作为平顺性柱图项'};
T=cell2table(metrics,'VariableNames',{'metric','value','unit','definition'});
writetable(T,fullfile(outDir,'page2_metrics_summary.csv'),'Encoding','UTF-8');
end

function writeNotes(s,outDir)
fid=fopen(fullfile(outDir,'page2_notes.md'),'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'# Page 2 数据口径\n\n');
fprintf(fid,'- **A 区**保留 IHSID 优化模型；力变化率仍为优化代价项，不作为硬约束。\n');
fprintf(fid,'- **B 区**使用项目 MATLAB 场景几何的裁剪重绘：保留起始上半机构、完整目标动平台与圆柱，以及实际约束罩；下平台和下半支链不进入图中。\n');
fprintf(fid,'- **固定弧线 + 逐点逆解**：起点至目标点的单段二次贝塞尔上拱弧线，控制点取起终点中点的全局竖直上方 %.1f m（挂装箱体长边）。姿态使用起终点四元数平滑插值；全程 %.1f s、%d 节点，逐点解析逆解。该基线不引入途径点、避障、奇异性或执行器约束修正。\n',s.controlPointOffset,s.duration,s.nodeCount);
fprintf(fid,'- **IHSID**：使用已验收的 `ihsid_40x20_limited_memory` 参考轨迹；场景、节点数、时长、约束定义和密集后验检查保持不变。\n');
fprintf(fid,'- **C 区**的七行依次为障碍净距、奇异裕度、机构稳定性、支链行程、支链速度、支链加速度和驱动力。两侧均采用支链加速度上限 %.2f m/s^2；所有量先统一为安全裕度。红色表示违规，深蓝表示贴近边界，随裕度增大逐档变浅。\n',s.constraintPolicy.accelerationLimit);
fprintf(fid,'- 固定弧线基线的最小有符号障碍净距为 %.3f m；其节点级复算的最小奇异值为 %.3f、最大条件数为 %.3f、最大支链速度为 %.3f m/s、最大支链加速度为 %.3f m/s^2、最大驱动力为 %.1f N。\n',min(s.baseline.clearance(1,:)),min(s.baseline.sigma),max(s.baseline.cond),max(abs(s.baseline.Ld),[],'all'),max(abs(s.baseline.Ldd),[],'all'),max(abs(s.baseline.F),[],'all'));
fprintf(fid,'- **D 区**为选取的轨迹执行代价，而非全部优化目标汇总。以固定弧线 + 逐点逆解为 100%%，IHSID 的平均加速度、加加速度均方根、累计支链行程分别为 %.2f%%、%.2f%%、%.2f%%。\n',s.cost.ihsidPercent.meanAccel,s.cost.ihsidPercent.rmsJerk,s.cost.ihsidPercent.legTravel);
fprintf(fid,'- jerk 的高值统计仍已保存但未绘入 D 区：IHSID 的 P95 为 %.3f m/s^3（基线 %.3f m/s^3），峰值为 %.3f m/s^3（基线 %.3f m/s^3）；峰值位于 5.0 s 阶段衔接处，因而不能仅据 jerk 声称峰值冲击更低。\n',prctile(abs(s.cost.ihsidJerk(:)),95),prctile(abs(s.cost.baseJerk(:)),95),max(abs(s.cost.ihsidJerk),[],'all'),max(abs(s.cost.baseJerk),[],'all'));
fprintf(fid,'- 驱动力变化已重新计算并写入 CSV：固定弧线 + 逐点逆解的相邻节点力差 RMS 为 %.3f N，IHSID 为 %.3f N（%.2f%%）。因 D 区明确限定为加速度平顺性，该项不并入柱状图；驱动力本身仍作为 C 区硬约束保留。\n',s.cost.baseForceRate,s.cost.ihsidForceRate,s.cost.ihsidPercent.deltaF);
fprintf(fid,'- 页底索引使用 IHSID 已验收的密集后验检查值：最小净距 %.1f mm（终端）、最低奇异值 %.3f、最大条件数 %.2f、最高支链速度 %.3f m/s、最高支链加速度 %.3f m/s^2。\n',s.denseReport.minClearance*1e3,s.denseReport.minSigmaMin,s.denseReport.maxCondJ,s.denseReport.maxAbsLd,s.denseReport.maxAbsLdd);
fprintf(fid,'- 本页仅比较离线轨迹规划；不包含控制器或闭环仿真。\n');
end
