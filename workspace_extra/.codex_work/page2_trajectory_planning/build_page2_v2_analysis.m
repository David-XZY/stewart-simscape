function build_page2_v2_analysis()
% PAGE 2 V2: exact Linear+IK vs accepted IHSID comparison on 61 common nodes.
% Linear+IK is a piecewise Cartesian straight path (q0 -> waypoint -> goal),
% SLERP attitude interpolation, quintic time law, and pointwise analytical IK.

set(groot, 'defaultFigureVisible', 'off');
rootDir = 'D:/NPU/Matlab_code/stewart-simscape';
outDir = 'D:/NPU/Matlab_code/output/page2_trajectory_planning';
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'opt_minimal', 'core'));
addpath(fullfile(rootDir, 'opt_minimal', 'ihsid'));
if ~exist(outDir, 'dir'), mkdir(outDir); end

sampleFile = fullfile(rootDir, 'opt_minimal', 'examples', 'ihsid_40x20_limited_memory', 'simscape_references.mat');
S = load(sampleFile, 'refs', 'denseReport', 'solverResult', 'result');
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildTwoPhaseIHSDiscretization(scene, 40, 20);

base = buildLinearIkBaseline(model, scene, disc);
ihsid = buildIhsidNodeData(S.refs, model, scene, disc);
[Mbase, badBase] = hardMargins(base, model, scene, disc);
[Mihsid, badIhsid] = hardMargins(ihsid, model, scene, disc);

cost = makeCostMetrics(base, ihsid);
cards = struct('dmin', S.denseReport.minClearance, 'sigmin', S.denseReport.minSigmaMin, ...
    'condmax', S.denseReport.maxCondJ, 'vmax', S.denseReport.maxAbsLd, ...
    'amax', S.denseReport.maxAbsLdd, 'n', numel(disc.tNode));
summary = struct('baseline', base, 'ihsid', ihsid, 'Mbase', Mbase, 'Mihsid', Mihsid, ...
    'badBase', badBase, 'badIhsid', badIhsid, 'cost', cost, 'cards', cards, ...
    'model', model, 'scene', scene, 'disc', disc, 'sourceFile', sampleFile);
save(fullfile(outDir, 'page2_v2_comparison_data.mat'), 'summary', '-v7');

drawHeatmapPair(Mbase, Mihsid, badBase, badIhsid, disc, outDir);
drawCostBars(cost, outDir);
writeV2Csv(summary, outDir);
writeV2Notes(summary, outDir);
fprintf('PAGE2_V2_ANALYSIS_OK baselineBad=%d ihsidBad=%d meanRatio=%.6f rmsRatio=%.6f p95Ratio=%.6f dFRatio=%.6f\n', ...
    sum(badBase), sum(badIhsid), cost.ihsidPercent.mean, cost.ihsidPercent.rms, cost.ihsidPercent.p95, cost.ihsidPercent.deltaF);
end

function data = buildLinearIkBaseline(model, scene, disc)
n = numel(disc.tNode);
Q = zeros(6,n);
R0 = rpy2rotmZYX(scene.q0(4:6));
Rway = rpy2rotmZYX(scene.qWaypoint(4:6));
Rgoal = rpy2rotmZYX(scene.qGoal(4:6));
for k = 1:n
    t = disc.tNode(k);
    if t <= disc.durationApproach + 1e-12
        s = quinticProgress(t / disc.durationApproach);
        p = (1-s)*scene.q0(1:3) + s*scene.qWaypoint(1:3);
        R = quatSlerp(rotmToQuat(R0), rotmToQuat(Rway), s);
    else
        s = quinticProgress((t-disc.durationApproach) / disc.durationInsertion);
        p = (1-s)*scene.qWaypoint(1:3) + s*scene.qGoal(1:3);
        R = quatSlerp(rotmToQuat(Rway), rotmToQuat(Rgoal), s);
    end
    Q(:,k) = [p; rotmToRpyZYX(quatToRotm(R))];
end
[Qd, Qdd] = finiteDiff(Q, disc.h);
data = evaluateNodeSeries(Q, Qd, Qdd, model, scene, disc, true);
data.name = 'Linear + IK';
data.definition = 'two-phase Cartesian linear path (start to waypoint to goal) + SLERP attitude + pointwise analytical IK; phase-wise quintic timing';
end

function data = buildIhsidNodeData(refs, model, scene, disc)
Q = refs.q; Qd = refs.qd;
[~, Qdd] = finiteDiff(Q, disc.h);
data = evaluateNodeSeries(Q, Qd, Qdd, model, scene, disc, false);
data.F = refs.Fleg;
% Match all kinematic margins to the stated finite-difference convention.
data.L = zeros(6, size(Q,2));
for k = 1:size(Q,2), data.L(:,k) = sgpIK(Q(:,k), model).L; end
[data.Ld, data.Ldd] = finiteDiff(data.L, disc.h);
data.name = 'IHSID';
data.definition = 'accepted standard IHSID 40+20 reference, re-evaluated on saved nodes';
end

function data = evaluateNodeSeries(Q, Qd, Qdd, model, scene, disc, useInverseDynamics)
n = size(Q,2);
data = struct('t',disc.tNode,'Q',Q,'Qd',Qd,'Qdd',Qdd, ...
    'L',zeros(6,n),'Ld',zeros(6,n),'Ldd',zeros(6,n),'F',zeros(6,n), ...
    'sigma',zeros(1,n),'cond',zeros(1,n),'clearance',zeros(3,n),'ikFailed',false(1,n));
for k = 1:n
    try
        kin = sgpIK(Q(:,k), model);
        data.L(:,k) = kin.L;
        J = sgpJacobian(Q(:,k), model);
        data.sigma(k) = J.sigmaMin; data.cond(k) = J.condJ;
        clear = evaluateCylinderBoxDistanceNumeric(Q(:,k), scene);
        data.clearance(:,k) = clear.distances(:);
        if useInverseDynamics
            data.F(:,k) = inverseDynamicsCompositeRigidBody(Q(:,k), Qd(:,k), Qdd(:,k), model);
        end
    catch
        data.ikFailed(k) = true;
        data.L(:,k) = NaN; data.sigma(k) = NaN; data.cond(k) = NaN; data.clearance(:,k) = NaN;
        if useInverseDynamics, data.F(:,k) = NaN; end
    end
end
[data.Ld, data.Ldd] = finiteDiff(data.L, disc.h);
end

function [M, bad] = hardMargins(data, model, scene, disc)
n = numel(data.t); collision = zeros(1,n);
for k = 1:n
    if data.t(k) <= disc.durationApproach + 1e-12, roofMin = scene.collision.stage1ConstraintDistance; else, roofMin = scene.collision.finalGap; end
    roofM = (data.clearance(1,k)-roofMin)/max(roofMin,1e-6);
    sideM = min((data.clearance(2:3,k)-scene.collision.safeDistance)/scene.collision.safeDistance);
    collision(k) = min(roofM, sideM);
end
sigM = (data.sigma-model.singularity.sigmaMinSafe)/model.singularity.sigmaMinSafe;
condM = (model.singularity.condWarning-data.cond)/model.singularity.condWarning;
lenM = min([(data.L-model.lmin)./(model.lmax-model.lmin); (model.lmax-data.L)./(model.lmax-model.lmin)],[],1);
velM = min((model.actuator.ldotMax-abs(data.Ld))./model.actuator.ldotMax,[],1);
accM = min((model.actuator.lddotMax-abs(data.Ldd))./model.actuator.lddotMax,[],1);
forceM = min([(data.F-model.actuator.forceMin)./(model.actuator.forceMax-model.actuator.forceMin); (model.actuator.forceMax-data.F)./(model.actuator.forceMax-model.actuator.forceMin)],[],1);
M = [collision;sigM;condM;lenM;velM;accM;forceM];
bad = data.ikFailed | any(M < -1e-9, 1) | any(~isfinite(M),1);
end

function cost = makeCostMetrics(base, ihsid)
metricBase = [mean(abs(base.Ldd),'all'), sqrt(mean(base.Ldd.^2,'all')), prctile(abs(base.Ldd(:)),95), sqrt(mean(diff(base.F,1,2).^2,'all'))];
metricIhsid = [mean(abs(ihsid.Ldd),'all'), sqrt(mean(ihsid.Ldd.^2,'all')), prctile(abs(ihsid.Ldd(:)),95), sqrt(mean(diff(ihsid.F,1,2).^2,'all'))];
cost.names = {'Mean |a|','RMS(a)','P95(|a|)','RMS(ΔF)'};
cost.baseline = metricBase; cost.ihsid = metricIhsid;
cost.ihsidPercent = struct('mean',100*metricIhsid(1)/metricBase(1), 'rms',100*metricIhsid(2)/metricBase(2), ...
    'p95',100*metricIhsid(3)/metricBase(3), 'deltaF',100*metricIhsid(4)/metricBase(4));
cost.improvement = 100*(1-metricIhsid./metricBase);
end

function drawHeatmapPair(M1,M2,bad1,bad2,disc,outDir)
fig = figure('Color','w','Position',[100 100 980 360]);
tl = tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
labels = {'碰撞间隙 d_{col}','σ_{min}(J)','κ(J)','支链长度 l','支链速度 |l̇|','支链加速度 |l̈|','驱动力 F'};
allM = {M1,M2}; names = {'Linear + IK','IHSID'}; badAll = {bad1,bad2};
for a = 1:2
    ax = nexttile(tl); imagesc(ax,linspace(0,100,size(allM{a},2)),1:7,allM{a}); hold(ax,'on');
    set(ax,'YDir','reverse','YTick',1:7,'YTickLabel',labels,'FontName','Microsoft YaHei','FontSize',13,'TickLength',[0 0]);
    xticks(ax,[0 25 50 75 100]); xlabel(ax,'任务进度 / %','FontName','Microsoft YaHei','FontSize',13);
    title(ax,names{a},'FontName','Microsoft YaHei','FontSize',18,'FontWeight','bold','Color',a==1*[0.9294 0.5451 0.2275]+(a==2)*[0.0627 0.2431 0.4314]);
    colormap(ax,marginMap()); clim(ax,[-1 1]);
    % A white × is reserved for an actual IK failure. Constraint violations are
    % represented only by their red margin cells, preserving the stated semantics.
    text(ax,2,0.42,sprintf('VIOLATION NODES  %d',sum(badAll{a})),'FontName','Aptos','FontSize',10,'FontWeight','bold','Color',a==1*[0.9294 0.5451 0.2275]+(a==2)*[0.0627 0.2431 0.4314]);
end
cb=colorbar; cb.Layout.Tile='east'; cb.Label.String='归一化安全裕度  m'; cb.Label.FontName='Microsoft YaHei'; cb.Label.FontSize=13; cb.Ticks=[-1 0 1]; cb.TickLabels={'m<0 违反','m=0 边界','m>0 满足'}; cb.FontSize=10;
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.png'),'Resolution',260);
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.pdf'),'ContentType','vector');
exportgraphics(fig,fullfile(outDir,'fig_constraint_feasibility_comparison.svg'),'ContentType','vector');
close(fig);
end

function drawCostBars(cost,outDir)
fig=figure('Color','w','Position',[100 100 820 360]); ax=axes(fig,'Position',[0.13 0.18 0.80 0.68]);
vals=[100*ones(1,4); [cost.ihsidPercent.mean cost.ihsidPercent.rms cost.ihsidPercent.p95 cost.ihsidPercent.deltaF]].';
b=bar(ax,vals,'grouped','BarWidth',0.72); b(1).FaceColor=[0.9294 0.5451 0.2275]; b(2).FaceColor=[0.0627 0.2431 0.4314]; b(1).EdgeColor='none';b(2).EdgeColor='none';
ylim(ax,[0 110]); yticks(ax,0:25:100); ytickformat(ax,'percentage'); grid(ax,'on'); box(ax,'off'); ax.YGrid='on';ax.XGrid='off';ax.GridAlpha=0.18;ax.GridColor=[0.55 0.60 0.66];
set(ax,'XTickLabel',cost.names,'FontName','Aptos','FontSize',15,'LineWidth',1);
ylabel(ax,'相对代价指标 / %','FontName','Microsoft YaHei','FontSize',16);
% The slide owns the compact top-right legend, avoiding any overlap with bar labels.
text(ax,-0.62,104,'↓ LOWER','FontName','Aptos','FontSize',12,'FontWeight','bold','Color',[0.35 0.42 0.50]);
for i=1:4
    text(ax,i+0.15,vals(i,2)+4,sprintf('%.1f%%',vals(i,2)),'HorizontalAlignment','center','FontName','Aptos','FontSize',17,'FontWeight','bold','Color',[0.0627 0.2431 0.4314]);
    text(ax,i+0.15,vals(i,2)-11,sprintf('↓%.1f%%',100-vals(i,2)),'HorizontalAlignment','center','FontName','Aptos','FontSize',16,'FontWeight','bold','Color',[1 1 1]);
end
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.png'),'Resolution',260);
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.pdf'),'ContentType','vector');
exportgraphics(fig,fullfile(outDir,'fig_cost_bars.svg'),'ContentType','vector');
close(fig);
end

function [d1,d2]=finiteDiff(x,h)
d1=zeros(size(x));d2=zeros(size(x));n=size(x,2);
d1(:,1)=(-3*x(:,1)+4*x(:,2)-x(:,3))/(2*h); d1(:,end)=(3*x(:,end)-4*x(:,end-1)+x(:,end-2))/(2*h);
d2(:,1)=(2*x(:,1)-5*x(:,2)+4*x(:,3)-x(:,4))/(h^2); d2(:,end)=(2*x(:,end)-5*x(:,end-1)+4*x(:,end-2)-x(:,end-3))/(h^2);
d1(:,2:end-1)=(x(:,3:end)-x(:,1:end-2))/(2*h); d2(:,2:end-1)=(x(:,3:end)-2*x(:,2:end-1)+x(:,1:end-2))/(h^2);
end

function s=quinticProgress(tau), tau=max(0,min(1,tau)); s=10*tau^3-15*tau^4+6*tau^5; end
function q=rotmToQuat(R)
tr=trace(R); if tr>0, s=sqrt(tr+1)*2; q=[0.25*s;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
elseif R(1,1)>R(2,2) && R(1,1)>R(3,3), s=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2; q=[(R(3,2)-R(2,3))/s;0.25*s;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
elseif R(2,2)>R(3,3), s=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2; q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;0.25*s;(R(2,3)+R(3,2))/s];
else, s=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2; q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;0.25*s]; end; q=q/norm(q); end
function q=quatSlerp(q0,q1,s)
q0=q0(:);q1=q1(:);d=dot(q0,q1);if d<0,q1=-q1;d=-d;end;if d>0.9995,q=(1-s)*q0+s*q1; q=q/norm(q);return;end;th=acos(max(-1,min(1,d)));q=(sin((1-s)*th)*q0+sin(s*th)*q1)/sin(th);end
function R=quatToRotm(q)
q=q/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);R=[1-2*(y^2+z^2),2*(x*y-z*w),2*(x*z+y*w);2*(x*y+z*w),1-2*(x^2+z^2),2*(y*z-x*w);2*(x*z-y*w),2*(y*z+x*w),1-2*(x^2+y^2)];end
function rpy=rotmToRpyZYX(R)
pitch=asin(max(-1,min(1,-R(3,1)))); roll=atan2(R(3,2),R(3,3));yaw=atan2(R(2,1),R(1,1));rpy=[roll;pitch;yaw];end
function cmap=marginMap()
cmap=[linspace(0.55,0.98,64)',linspace(0.03,0.95,64)',linspace(0.05,0.95,64)';linspace(0.98,0.18,64)',linspace(0.95,0.47,64)',linspace(0.95,0.72,64)';linspace(0.18,0.04,64)',linspace(0.47,0.23,64)',linspace(0.72,0.43,64)'];end
function writeV2Csv(s,outDir)
metric={'linear_ik_violation_nodes';'ihsid_violation_nodes';'mean_abs_accel_linear_ik';'mean_abs_accel_ihsid';'mean_abs_accel_ihsid_percent';'rms_accel_linear_ik';'rms_accel_ihsid';'rms_accel_ihsid_percent';'p95_abs_accel_linear_ik';'p95_abs_accel_ihsid';'p95_abs_accel_ihsid_percent';'rms_delta_force_linear_ik';'rms_delta_force_ihsid';'rms_delta_force_ihsid_percent'};
value=[sum(s.badBase);sum(s.badIhsid);s.cost.baseline(1);s.cost.ihsid(1);s.cost.ihsidPercent.mean;s.cost.baseline(2);s.cost.ihsid(2);s.cost.ihsidPercent.rms;s.cost.baseline(3);s.cost.ihsid(3);s.cost.ihsidPercent.p95;s.cost.baseline(4);s.cost.ihsid(4);s.cost.ihsidPercent.deltaF];
unit={'nodes';'nodes';'m/s^2';'m/s^2';'%';'m/s^2';'m/s^2';'%';'m/s^2';'m/s^2';'%';'N';'N';'%'};
definition=repmat({'61-node common grid; Linear+IK is a two-phase Cartesian linear path through the mounting waypoint, SLERP orientation, pointwise IK, and phase-wise quintic timing'},numel(metric),1);
writetable(table(metric,value,unit,definition),fullfile(outDir,'page2_v2_metrics_summary.csv'),'Encoding','UTF-8');
end
function writeV2Notes(s,outDir)
fid=fopen(fullfile(outDir,'page2_v2_notes.md'),'w');c=onCleanup(@()fclose(fid));
fprintf(fid,'# Page 2 V2 — data definition\n\n');
fprintf(fid,'- **Linear + IK**: two-phase Cartesian linear path (`q0→qWaypoint→qGoal`), SLERP attitude, the same 7.5 s / 61-node grid and phase-wise quintic time law, then pointwise analytical inverse kinematics.\n');
fprintf(fid,'- **IHSID**: accepted standard IHSID 40+20 reference stored in `%s`, re-evaluated at the same 61 nodes.\n',strrep(s.sourceFile,'\\','/'));
fprintf(fid,'- The seven heatmap rows are normalized hard-constraint margins; m<0 is a violation, m=0 is the boundary. The force-rate term is intentionally excluded from this hard-constraint heatmap because it is an objective term, not a hard constraint in this run.\n');
fprintf(fid,'- Violation nodes: Linear + IK = %d; IHSID = %d.\n',sum(s.badBase),sum(s.badIhsid));
fprintf(fid,'- Relative objective values (Baseline = 100%%): Mean |a| %.2f%%, RMS(a) %.2f%%, P95(|a|) %.2f%%, RMS(ΔF) %.2f%%.\n',s.cost.ihsidPercent.mean,s.cost.ihsidPercent.rms,s.cost.ihsidPercent.p95,s.cost.ihsidPercent.deltaF);
fprintf(fid,'- Bottom index uses accepted dense post-check metrics: d_min %.1f mm terminal, sigma_min %.3f, kappa_max %.2f, v_max %.3f m/s, a_max %.3f m/s^2.\n',s.cards.dmin*1e3,s.cards.sigmin,s.cards.condmax,s.cards.vmax,s.cards.amax);
end
