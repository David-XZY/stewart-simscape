function build_page3_evidence()
% Recompute all audience-facing metrics and export the four data evidence figures.
outDir = 'D:\NPU\Matlab_code\output\page3_control_tracking_revised';
if ~exist(outDir, 'dir'), mkdir(outDir); end
strictMat = 'D:\NPU\Matlab_code\stewart-simscape-disturbance-experiment\results\reports\strict_clf_cbf_qp\exact_model_study\strict_clf_cbf_qp_exact_model_study.mat';
idMat = 'D:\NPU\Matlab_code\stewart-simscape\results\controller\pwm_force_identifier_20260613_232220.mat';

S = load(strictMat, 'result'); result = S.result;
ids = string(arrayfun(@(x) x.scenario.id, result.runs, 'UniformOutput', false));
iLqi = find(ids == "ct_lqi", 1); iQp = find(ids == "full_qp_nominal", 1);
assert(~isempty(iLqi) && ~isempty(iQp), 'Required strict control runs were not found.');
lqi = result.runs(iLqi); qp = result.runs(iQp);
assert(strcmp(lqi.scenario.controllerMode, 'baseline') && strcmp(qp.scenario.controllerMode, 'full'));
assert(qp.scenario.guaranteeApplicable && strcmp(qp.scenario.disturbance, 'matched nominal plant, ideal force'));
assert(isequaln(lqi.control.time, qp.control.time), 'Control runs do not share the same time grid.');
assert(isfield(result, 'reference') && isfield(result, 'initialState'), 'Shared reference or initial-state metadata is missing.');

I = load(idMat, 'dataset', 'report', 'identified');
dataset = I.dataset; report = I.report; identified = I.identified;
assert(nnz(dataset.split.train) == 3600 && nnz(dataset.split.validation) == 1200 && nnz(dataset.split.test) == 1200, 'Unexpected identification split.');

blue = [39 106 154]/255; gray = [0.43 0.43 0.43]; red = [0.75 0.08 0.08]; paleRed = [1.0 0.94 0.94]; black = [0.08 0.08 0.08]; lightGray = [0.83 0.83 0.83];
dtQp = result.strictConfig.dt;
marginLqi = min(lqi.dense.collisionMargin, [], 1).';
marginQp = min(qp.dense.collisionMargin, [], 1).';
tDense = lqi.dense.time(:);
tCtl = qp.control.time(:);
active = qp.diagnostics.activeConstraintCount(:) > 0;
qptime = qp.diagnostics.qpSolveTime(:);
eLqi = lqi.control.poseError(1:3, :);
eQp = qp.control.poseError(1:3, :);

% Safety and tracking metrics, recomputed from the stored signals.
[minMarginLqi, ixMin] = min(marginLqi);
[minMarginQp, ixMinQp] = min(marginQp);
negMask = marginLqi < 0;
denseDt = median(diff(tDense));
negDuration = nnz(negMask) * denseDt;
lqiPosRms = sqrt(mean(sum(eLqi.^2,1))) * 1e3;
lqiPosPeak = max(sqrt(sum(eLqi.^2,1))) * 1e3;
qpPosRms = sqrt(mean(sum(eQp.^2,1))) * 1e3;
qpPosPeak = max(sqrt(sum(eQp.^2,1))) * 1e3;
qpP50 = projectPercentile(qptime, 50)*1e3; qpP95 = projectPercentile(qptime,95)*1e3; qpP99 = projectPercentile(qptime,99)*1e3;
qpMax = max(qptime)*1e3; qpMean = mean(qptime)*1e3; qpDeadlineMiss = nnz(qptime > dtQp);
qpActiveRatio = mean(active)*100; qpActiveSamples = nnz(active);

% Identfication metrics from test-only samples, independent of the QP loop.
testMask = dataset.split.test(:);
trueF = dataset.trueForce;
grayF = report.grayForce;
narxF = report.estimatedForce;
assert(isequal(size(trueF), size(grayF), size(narxF)), 'Identifier arrays are inconsistent.');
scaleN = 4800;
legGray = sqrt(mean((grayF(testMask,:) - trueF(testMask,:)).^2, 1)) / scaleN;
legNarx = sqrt(mean((narxF(testMask,:) - trueF(testMask,:)).^2, 1)) / scaleN;
overallGray = sqrt(mean((grayF(testMask,:) - trueF(testMask,:)).^2, 'all')) / scaleN;
overallNarx = sqrt(mean((narxF(testMask,:) - trueF(testMask,:)).^2, 'all')) / scaleN;
overallGrayRmse = sqrt(mean((grayF(testMask,:) - trueF(testMask,:)).^2, 'all'));
overallNarxRmse = sqrt(mean((narxF(testMask,:) - trueF(testMask,:)).^2, 'all'));
improvePct = (overallGray - overallNarx) / overallGray * 100;
assert(abs(overallGray - report.metrics.grayTestNrmse) < 1e-10, 'Gray NRMSE mismatch.');
assert(abs(overallNarx - report.metrics.narxTestNrmse) < 1e-10, 'NARX NRMSE mismatch.');

% Test-only representative direction reversal: fixed leg 4, first valid crossing
% with 0.25 s preceding/following context and above-median 0.5 s force swing.
legIdx = 4;
time = dataset.time(:); speed = dataset.trueLegSpeed(:,legIdx); tTest = time(testMask);
ixTest = find(testMask); halfN = round(0.25 / dataset.sampleTime); allRanges = [];
for k = 1+halfN:numel(ixTest)-halfN
    c = ixTest(k); w = (c-halfN):(c+halfN);
    if any(~isfinite([speed(w); trueF(w,legIdx); grayF(w,legIdx); narxF(w,legIdx)]), 'all'), continue; end
    allRanges(end+1,1) = range(trueF(w,legIdx)); %#ok<AGROW>
end
forceVariationMedian = median(allRanges);
reversalCenter = NaN;
for k = 2:numel(ixTest)-1
    c = ixTest(k); w = (c-halfN):(c+halfN);
    if c-halfN < ixTest(1) || c+halfN > ixTest(end), continue; end
    isCross = speed(c-1) * speed(c+1) < 0;
    finiteWindow = all(isfinite([speed(w); trueF(w,legIdx); grayF(w,legIdx); narxF(w,legIdx)]), 'all');
    swingOk = range(trueF(w,legIdx)) > forceVariationMedian;
    if isCross && finiteWindow && swingOk, reversalCenter = c; break; end
end
assert(isfinite(reversalCenter), 'No valid test-only direction reversal was found.');
reversalWindow = (reversalCenter-halfN):(reversalCenter+halfN);
reversalTime = time(reversalCenter);
revMask = false(size(testMask)); revMask(reversalWindow) = true;
revMask = revMask & testMask;
nonRevMask = testMask & ~revMask;
revGray = sqrt(mean((grayF(revMask,legIdx)-trueF(revMask,legIdx)).^2)) / scaleN;
revNarx = sqrt(mean((narxF(revMask,legIdx)-trueF(revMask,legIdx)).^2)) / scaleN;
nonRevGray = sqrt(mean((grayF(nonRevMask,legIdx)-trueF(nonRevMask,legIdx)).^2)) / scaleN;
nonRevNarx = sqrt(mean((narxF(nonRevMask,legIdx)-trueF(nonRevMask,legIdx)).^2)) / scaleN;

% C1 window: global LQI minimum plus/minus 0.25 s; only expand to 0.8 s total
% if the QP intervention stripe would otherwise be absent.
tStar = tDense(ixMin); tL = max(tDense(1), tStar - 0.25); tR = min(tDense(end), tStar + 0.25);
if ~any(active(tCtl >= tL & tCtl <= tR))
    tL = max(tDense(1), tStar - 0.40); tR = min(tDense(end), tStar + 0.40);
end
marginMask = tDense >= tL & tDense <= tR;
activeMask = tCtl >= tL & tCtl <= tR;

% ---- Figure C1: collision margin near the strict global minimum ----
fig = figure('Visible','off','Color','w','Position',[80 80 1500 680]); ax = axes(fig); hold(ax,'on'); box(ax,'off');
yLo = min([marginLqi(marginMask); marginQp(marginMask); -0.15e-3]) * 1e3 - 0.12; yHi = max([marginLqi(marginMask); marginQp(marginMask)]) * 1e3 + 0.16;
patch(ax, [tL tR tR tL], [yLo yLo 0 0], paleRed, 'EdgeColor','none');
plot(ax, tDense(marginMask), marginLqi(marginMask)*1e3, '--', 'Color',gray,'LineWidth',2.0);
plot(ax, tDense(marginMask), marginQp(marginMask)*1e3, '-', 'Color',blue,'LineWidth',2.6);
yline(ax,0,'--','Color',red,'LineWidth',1.0);
plot(ax,tDense(ixMin),minMarginLqi*1e3,'o','Color',gray,'MarkerFaceColor','w','LineWidth',1.8,'MarkerSize',7);
plot(ax,tDense(ixMinQp),minMarginQp*1e3,'o','Color',blue,'MarkerFaceColor',blue,'MarkerSize',6);
text(ax,tDense(ixMin)-0.004,minMarginLqi*1e3+0.32,'LQI  -0.783 mm','Color',gray,'FontName','Microsoft YaHei','FontSize',13,'HorizontalAlignment','right','VerticalAlignment','bottom');
text(ax,tDense(ixMinQp)-0.004,minMarginQp*1e3+0.06,'LQI+QP  +0.422 mm','Color',blue,'FontName','Microsoft YaHei','FontSize',13,'HorizontalAlignment','right','VerticalAlignment','bottom');
stripeY = yLo + 0.03*(yHi-yLo); stripeH = 0.055*(yHi-yLo);
for j = find(activeMask).'
    rectangle(ax,'Position',[tCtl(j)-dtQp/2 stripeY dtQp stripeH],'FaceColor',lightGray,'EdgeColor','none');
end
text(ax,tL+0.01*(tR-tL),stripeY+stripeH*1.45,'QP/CBF active','Color',[.35 .35 .35],'FontName','Arial','FontSize',11);
xlim(ax,[tL tR]); ylim(ax,[yLo yHi]); grid(ax,'on'); ax.GridColor=[.88 .88 .88]; ax.FontName='Arial'; ax.FontSize=13; ax.LineWidth=0.8;
xlabel(ax,'t / s','FontSize',14); ylabel(ax,'collision margin / mm','FontSize',14); title(ax,'C1  最危险时刻的碰撞裕度','FontName','Microsoft YaHei','FontSize',16,'FontWeight','bold','Color',blue,'HorizontalAlignment','left');
exportPair(fig,outDir,'fig3C_collision_margin'); close(fig);

% ---- Figure C2: safety / accuracy plane ----
fig = figure('Visible','off','Color','w','Position',[80 80 1100 680]); ax = axes(fig); hold(ax,'on'); box(ax,'off');
xVals = [lqiPosRms; qpPosRms]; yVals = [minMarginLqi; minMarginQp] * 1e3; xPad = 0.10; yLo2 = min(-1.0, min(yVals)-0.22); yHi2 = max(0.65, max(yVals)+0.22);
patch(ax,[min(xVals)-xPad max(xVals)+xPad max(xVals)+xPad min(xVals)-xPad],[yLo2 yLo2 0 0],paleRed,'EdgeColor','none'); yline(ax,0,'--','Color',red,'LineWidth',1.0);
plot(ax,xVals(1),yVals(1),'o','Color',gray,'MarkerFaceColor','w','MarkerSize',10,'LineWidth',2);
plot(ax,xVals(2),yVals(2),'o','Color',blue,'MarkerFaceColor',blue,'MarkerSize',10,'LineWidth',1.6);
quiver(ax,xVals(1),yVals(1),xVals(2)-xVals(1),yVals(2)-yVals(1),0,'Color',[.25 .25 .25],'LineWidth',1.2,'MaxHeadSize',0.6);
text(ax,xVals(1)-0.004,yVals(1)+0.08,'LQI','Color',gray,'FontName','Arial','FontSize',14,'HorizontalAlignment','right');
text(ax,xVals(2)+0.004,yVals(2)+0.09,'LQI+QP','Color',blue,'FontName','Arial','FontSize',14,'FontWeight','bold');
text(ax,mean(xVals),yLo2+0.10,'违规点 21→0     QP P95 4.04 ms','Color',[.18 .18 .18],'FontName','Microsoft YaHei','FontSize',13,'HorizontalAlignment','center');
xlim(ax,[min(xVals)-xPad max(xVals)+xPad]); ylim(ax,[yLo2 yHi2]); grid(ax,'on'); ax.GridColor=[.88 .88 .88]; ax.FontName='Arial'; ax.FontSize=13; ax.LineWidth=0.8;
xlabel(ax,'position RMS / mm','FontSize',14); ylabel(ax,'min collision margin / mm','FontSize',14); title(ax,'C2  精度—安全平面','FontName','Microsoft YaHei','FontSize',16,'FontWeight','bold','Color',blue,'HorizontalAlignment','left');
exportPair(fig,outDir,'fig3C_safety_accuracy_plane'); close(fig);

% ---- Figure D1: test-only force validation at representative reversal ----
fig = figure('Visible','off','Color','w','Position',[80 80 1500 650]); ax = axes(fig); hold(ax,'on'); box(ax,'off');
plot(ax,time(reversalWindow),trueF(reversalWindow,legIdx),'Color',black,'LineWidth',2.2);
plot(ax,time(reversalWindow),grayF(reversalWindow,legIdx),'--','Color',gray,'LineWidth',2.0);
plot(ax,time(reversalWindow),narxF(reversalWindow,legIdx),'-','Color',blue,'LineWidth',2.5);
xline(ax,reversalTime,'--','换向','Color',[.45 .45 .45],'FontName','Microsoft YaHei','FontSize',12,'LabelVerticalAlignment','middle','LabelHorizontalAlignment','left');
yTop = max([trueF(reversalWindow,legIdx); grayF(reversalWindow,legIdx); narxF(reversalWindow,legIdx)]); ySpan = range([trueF(reversalWindow,legIdx); grayF(reversalWindow,legIdx); narxF(reversalWindow,legIdx)]);
text(ax,time(reversalWindow(1))+0.01,yTop-0.08*ySpan,'F_{true}','Color',black,'FontName','Arial','FontSize',13,'FontWeight','bold');
text(ax,time(reversalWindow(1))+0.01,yTop-0.14*ySpan,'F_{gray}','Color',gray,'FontName','Arial','FontSize',13);
text(ax,time(reversalWindow(1))+0.01,yTop-0.20*ySpan,'F_{NARX}','Color',blue,'FontName','Arial','FontSize',13,'FontWeight','bold');
grid(ax,'on'); ax.GridColor=[.88 .88 .88]; ax.FontName='Arial'; ax.FontSize=13; ax.LineWidth=0.8;
xlabel(ax,'t / s','FontSize',14); ylabel(ax,'force / N','FontSize',14); title(ax,'D1  测试段代表性换向（轴 4）','FontName','Microsoft YaHei','FontSize',16,'FontWeight','bold','Color',blue,'HorizontalAlignment','left');
exportPair(fig,outDir,'fig3D_force_validation'); close(fig);

% ---- Figure D2: per-leg dumbbell test NRMSE ----
fig = figure('Visible','off','Color','w','Position',[80 80 1100 670]); ax = axes(fig); hold(ax,'on'); box(ax,'off'); y = 6:-1:1;
for k=1:6
    plot(ax,[legGray(k) legNarx(k)]*100,[y(k) y(k)],'-','Color',lightGray,'LineWidth',2.0);
    plot(ax,legGray(k)*100,y(k),'o','Color',gray,'MarkerFaceColor','w','MarkerSize',8,'LineWidth',1.7);
    plot(ax,legNarx(k)*100,y(k),'o','Color',blue,'MarkerFaceColor',blue,'MarkerSize',8,'LineWidth',1.2);
    text(ax,max([legGray(k) legNarx(k)])*100+0.014,y(k),sprintf('%.2f → %.2f%%',legGray(k)*100,legNarx(k)*100),'Color',[.22 .22 .22],'FontName','Arial','FontSize',12,'VerticalAlignment','middle');
end
xlim(ax,[0.60 1.22]); ylim(ax,[0.45 6.55]); grid(ax,'on'); ax.XGrid='on'; ax.YGrid='off'; ax.GridColor=[.88 .88 .88]; ax.FontName='Arial'; ax.FontSize=13; ax.LineWidth=0.8;
ax.YAxis.Visible = 'off';
for k=1:6, text(ax,0.602,y(k),sprintf('leg %d',k),'Color',[.2 .2 .2],'FontName','Arial','FontSize',12,'VerticalAlignment','middle'); end
xlabel(ax,'test NRMSE / %FS','FontSize',14); title(ax,'D2  六支链测试 NRMSE','FontName','Microsoft YaHei','FontSize',16,'FontWeight','bold','Color',blue,'HorizontalAlignment','left');
text(ax,0.61,0.68,sprintf('overall  %.4f%% → %.4f%%   ↓%.1f%%',overallGray*100,overallNarx*100,improvePct),'Color',blue,'FontName','Arial','FontSize',14,'FontWeight','bold');
exportPair(fig,outDir,'fig3D_leg_nrmse'); close(fig);

% CSV: one numeric row per audit metric, directly used by the deck builder.
rows = table();
rows = addMetric(rows,'lqi_position_rms',lqiPosRms,'mm','translation only','RMS of the first three pose-error coordinates');
rows = addMetric(rows,'lqi_position_peak',lqiPosPeak,'mm','translation only','max Euclidean norm of the first three pose-error coordinates');
rows = addMetric(rows,'lqi_min_collision_margin',minMarginLqi*1e3,'mm','all collision channels','minimum over time of min(collisionMargin_1..3)');
rows = addMetric(rows,'lqi_negative_margin_samples',nnz(negMask),'samples','dense trajectory','number of dense samples with collision margin below zero');
rows = addMetric(rows,'lqi_negative_margin_duration',negDuration,'s','dense trajectory','negative samples multiplied by dense sample interval');
rows = addMetric(rows,'qp_position_rms',qpPosRms,'mm','translation only','RMS of the first three pose-error coordinates');
rows = addMetric(rows,'qp_position_peak',qpPosPeak,'mm','translation only','max Euclidean norm of the first three pose-error coordinates');
rows = addMetric(rows,'qp_min_collision_margin',minMarginQp*1e3,'mm','all collision channels','minimum over time of min(collisionMargin_1..3)');
rows = addMetric(rows,'qp_negative_margin_samples',nnz(marginQp < 0),'samples','dense trajectory','number of dense samples with collision margin below zero');
rows = addMetric(rows,'qp_qp_p50',qpP50,'ms','control tick','50th percentile of logged QP solve time');
rows = addMetric(rows,'qp_qp_p95',qpP95,'ms','control tick','95th percentile of logged QP solve time');
rows = addMetric(rows,'qp_qp_p99',qpP99,'ms','control tick','99th percentile of logged QP solve time');
rows = addMetric(rows,'qp_qp_max',qpMax,'ms','control tick','maximum logged QP solve time');
rows = addMetric(rows,'qp_qp_mean',qpMean,'ms','control tick','mean logged QP solve time');
rows = addMetric(rows,'qp_deadline_misses',qpDeadlineMiss,'ticks','threshold 10 ms','number of logged QP solve times above the 10 ms controller period');
rows = addMetric(rows,'qp_active_ratio',qpActiveRatio,'percent','control tick','fraction of ticks with activeConstraintCount > 0');
rows = addMetric(rows,'qp_active_samples',qpActiveSamples,'samples','control tick','number of ticks with activeConstraintCount > 0');
rows = addMetric(rows,'qp_controller_period',dtQp*1e3,'ms','strict QP configuration','strictConfig.dt');
for k=1:6
    rows = addMetric(rows,sprintf('leg%d_gray_test_nrmse',k),legGray(k)*100,'%FS',sprintf('test leg %d',k),'RMSE divided by 4800 N');
    rows = addMetric(rows,sprintf('leg%d_narx_test_nrmse',k),legNarx(k)*100,'%FS',sprintf('test leg %d',k),'RMSE divided by 4800 N');
end
rows = addMetric(rows,'gray_overall_test_nrmse',overallGray*100,'%FS','all six test legs','RMSE divided by 4800 N');
rows = addMetric(rows,'narx_overall_test_nrmse',overallNarx*100,'%FS','all six test legs','RMSE divided by 4800 N');
rows = addMetric(rows,'gray_overall_test_rmse',overallGrayRmse,'N','all six test legs','raw force RMSE');
rows = addMetric(rows,'narx_overall_test_rmse',overallNarxRmse,'N','all six test legs','raw force RMSE');
rows = addMetric(rows,'narx_relative_improvement',improvePct,'%','all six test legs','(gray NRMSE - NARX NRMSE)/gray NRMSE');
rows = addMetric(rows,'reversal_time',reversalTime,'s','test leg 4','first valid sign-crossing satisfying the fixed-window rule');
rows = addMetric(rows,'reversal_force_swing_threshold',forceVariationMedian,'N','test leg 4','median 0.5 s test-window force variation');
rows = addMetric(rows,'reversal_gray_nrmse',revGray*100,'%FS','test leg 4, +/-0.25 s','RMSE divided by 4800 N');
rows = addMetric(rows,'reversal_narx_nrmse',revNarx*100,'%FS','test leg 4, +/-0.25 s','RMSE divided by 4800 N');
rows = addMetric(rows,'nonreversal_gray_nrmse',nonRevGray*100,'%FS','test leg 4, remaining test','RMSE divided by 4800 N');
rows = addMetric(rows,'nonreversal_narx_nrmse',nonRevNarx*100,'%FS','test leg 4, remaining test','RMSE divided by 4800 N');
writetable(rows, fullfile(outDir,'page3_metrics_summary.csv'));

notes = {
    '# 第 3 页：数据与口径说明'
    ''
    '## 1. 轨迹跟踪与安全控制（严格可比）'
    sprintf('- MAT：`%s`。对照为 `ct_lqi`（Computed torque + LQI），安全控制为 `full_qp_nominal`（Strict CLF-CBF-QP, nominal）。两组均由同一 MAT 中的同一参考/初始状态与控制时间栅格读取；脚本已断言其控制时间与初态一致。', strictMat)
    sprintf('- QP 组为 matched nominal plant + ideal force 的严格模型研究；该页不把它表述为台架/实机结论。QP 控制周期为 %.0f ms。', dtQp*1e3)
    '- 碰撞裕度：每个时刻取 `dense.collisionMargin` 的三个碰撞通道最小值；图 C1 再取 LQI 全程的全局最小点并显示其附近窗口。红线为零裕度，红色背景只表示负裕度。'
    '- 位置 RMS/峰值：仅使用 `poseError(1:3,:)` 的平移分量欧氏范数，单位 mm；不将后三个姿态分量混入或等价换算。'
    '- QP 耗时：`diagnostics.qpSolveTime` 的每控制 tick 求解时间；P50/P95/P99/最大值为全程分位数/极值，deadline miss 定义为 solve time > strictConfig.dt。active ratio 定义为 `activeConstraintCount>0` 的 tick 比例。'
    ''
    '## 2. 灰盒 + NARX 残差辨识（独立链路）'
    sprintf('- MAT：`%s`。数据为 6000 点、5 ms 采样；按连续时间段 train/validation/test = %d/%d/%d（60/20/20）划分，图 D 与所有 NRMSE 仅使用 test。', idMat, nnz(dataset.split.train), nnz(dataset.split.validation), nnz(dataset.split.test))
    '- `F_true` 来自项目的高保真 PWM 电机/丝杠 teacher model，不是实测力；灰盒输出为 `F_gray`，NARX 仅学习残差 `F_true-F_gray`，在线估计为 `F_hat=F_gray+r_hat`。该辨识链路与 CLF-CBF-QP 主控制环独立，未把 NARX 叙述为嵌入 QP。'
    '- NRMSE：`RMSE / 4800 N`；overall 对所有测试段六支链样本共同计算，逐支链按各自测试样本计算。'
    sprintf('- D1 固定展示轴 %d 的第一处测试段有效换向：速度相邻样本变号、前后各 %.2f s 无缺失，且该 %.2f s 窗口真实力峰峰值高于测试窗口峰峰值的中位数；换向时刻为 %.3f s。', legIdx, halfN*dataset.sampleTime, 2*halfN*dataset.sampleTime, reversalTime)
    '- CSV 为所有图中数字的唯一导出源，包含每腿 NRMSE、换向/非换向误差、QP 耗时、激活率与安全裕度。'
    ''
    '[Sources]'
    sprintf('- Strict control MAT: %s', strictMat)
    sprintf('- Identifier MAT: %s', idMat)
    '- Visual template: D:\NPU\简历及相关资料\702\线下面试\2-2.pptx'
    };
fid = fopen(fullfile(outDir,'page3_notes.md'),'w','n','UTF-8'); fprintf(fid,'%s\n',notes{:}); fclose(fid);

fprintf('EXPORT_DIR=%s\n',outDir);
fprintf('LQI_MIN_MARGIN_MM=%.6f\nQP_MIN_MARGIN_MM=%.6f\nQP_P95_MS=%.6f\nQP_DEADLINE_MISSES=%d\nID_OVERALL=%.6f_to_%.6f\nREVERSAL_T=%.6f\n', minMarginLqi*1e3, minMarginQp*1e3, qpP95, nnz(qptime > dtQp), overallGray*100, overallNarx*100, reversalTime);
end

function exportPair(fig,outDir,stem)
set(fig,'PaperPositionMode','auto');
exportgraphics(fig,fullfile(outDir,[stem '.png']),'Resolution',110);
exportgraphics(fig,fullfile(outDir,[stem '.svg']),'ContentType','vector');
end

function t = addMetric(t,name,value,unit,scope,definition)
new = table(string(name),double(value),string(unit),string(scope),string(definition), 'VariableNames',{'metric','value','unit','scope','definition'});
if isempty(t), t = new; else, t = [t; new]; end
end

function value = projectPercentile(data, probability)
data = sort(data(isfinite(data)));
if isempty(data), value = NaN; return; end
location = 1 + (numel(data)-1) * probability / 100;
low = floor(location); high = ceil(location);
if low == high, value = data(low); else, value = data(low) + (location-low) * (data(high)-data(low)); end
end
