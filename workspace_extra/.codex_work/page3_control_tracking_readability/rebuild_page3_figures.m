function rebuild_page3_figures(outDir)
% Re-read raw MAT evidence and export tight vector figures for page 3.
if nargin < 1 || isempty(outDir)
    outDir = 'D:\NPU\Matlab_code\output\page3_control_tracking_readability';
end
if ~exist(outDir,'dir'), mkdir(outDir); end
strictMat = 'D:\NPU\Matlab_code\stewart-simscape-disturbance-experiment\results\reports\strict_clf_cbf_qp\exact_model_study\strict_clf_cbf_qp_exact_model_study.mat';
idMat = 'D:\NPU\Matlab_code\stewart-simscape\results\controller\pwm_force_identifier_20260613_232220.mat';

S = load(strictMat,'result'); result = S.result;
ids = string(arrayfun(@(x)x.scenario.id,result.runs,'UniformOutput',false));
iLqi = find(ids=="ct_lqi",1); iQp = find(ids=="full_qp_nominal",1);
assert(~isempty(iLqi) && ~isempty(iQp),'Required strict-control runs not found.');
lqi = result.runs(iLqi); qp = result.runs(iQp);
assert(strcmp(lqi.scenario.controllerMode,'baseline') && strcmp(qp.scenario.controllerMode,'full'));
assert(qp.scenario.guaranteeApplicable && strcmp(qp.scenario.disturbance,'matched nominal plant, ideal force'));
assert(isequaln(lqi.control.time,qp.control.time),'Control grids differ.');
dtQp = result.strictConfig.dt;

I = load(idMat,'dataset','report','identified'); dataset = I.dataset; report = I.report; %#ok<NASGU>
assert(nnz(dataset.split.train)==3600 && nnz(dataset.split.validation)==1200 && nnz(dataset.split.test)==1200,'Unexpected ID split.');

blue=[39 106 154]/255; gray=[.43 .43 .43]; black=[.08 .08 .08]; red=[.78 .13 .13]; paleRed=[1 .95 .95]; lightGray=[.82 .82 .82];
tDense=lqi.dense.time(:); marginLqi=min(lqi.dense.collisionMargin,[],1).'; marginQp=min(qp.dense.collisionMargin,[],1).';
tCtl=qp.control.time(:); active=qp.diagnostics.activeConstraintCount(:)>0; qptime=qp.diagnostics.qpSolveTime(:);
eLqi=lqi.control.poseError(1:3,:); eQp=qp.control.poseError(1:3,:);
[minLqi,ixMin]=min(marginLqi); [minQp,ixQp]=min(marginQp);
lqiRms=sqrt(mean(sum(eLqi.^2,1)))*1e3; qpRms=sqrt(mean(sum(eQp.^2,1)))*1e3;
neg=marginLqi<0; negDuration=nnz(neg)*median(diff(tDense));
p95=projectPercentile(qptime,95)*1e3;

testMask=dataset.split.test(:); trueF=dataset.trueForce; grayF=report.grayForce; narxF=report.estimatedForce; scaleN=4800;
assert(isequal(size(trueF),size(grayF),size(narxF)),'Identifier arrays differ.');
legGray=sqrt(mean((grayF(testMask,:)-trueF(testMask,:)).^2,1))/scaleN;
legNarx=sqrt(mean((narxF(testMask,:)-trueF(testMask,:)).^2,1))/scaleN;
overallGray=sqrt(mean((grayF(testMask,:)-trueF(testMask,:)).^2,'all'))/scaleN;
overallNarx=sqrt(mean((narxF(testMask,:)-trueF(testMask,:)).^2,'all'))/scaleN;
improve=(overallGray-overallNarx)/overallGray*100;
assert(abs(overallGray-report.metrics.grayTestNrmse)<1e-10 && abs(overallNarx-report.metrics.narxTestNrmse)<1e-10,'Stored NRMSE differs.');

% Fixed test-only leg-4 reversal selector: one 0.25 s window before and after.
legIdx=4; time=dataset.time(:); speed=dataset.trueLegSpeed(:,legIdx); halfN=round(.25/dataset.sampleTime); ixTest=find(testMask); ranges=[];
for k=1+halfN:numel(ixTest)-halfN
    w=(ixTest(k)-halfN):(ixTest(k)+halfN);
    if all(isfinite([speed(w);trueF(w,legIdx);grayF(w,legIdx);narxF(w,legIdx)]),'all'), ranges(end+1,1)=range(trueF(w,legIdx)); end %#ok<AGROW>
end
threshold=median(ranges); center=NaN;
for k=2:numel(ixTest)-1
    c=ixTest(k); w=(c-halfN):(c+halfN);
    valid=c-halfN>=ixTest(1) && c+halfN<=ixTest(end) && all(isfinite([speed(w);trueF(w,legIdx);grayF(w,legIdx);narxF(w,legIdx)]),'all');
    if valid && speed(c-1)*speed(c+1)<0 && range(trueF(w,legIdx))>threshold, center=c; break; end
end
assert(isfinite(center),'No valid test-only reversal.'); win=(center-halfN):(center+halfN); reversalTime=time(center);

% C1: global safety-critical window, raster preview at 300 dpi plus SVG source.
tStar=tDense(ixMin); tL=max(tDense(1),tStar-.25); tR=min(tDense(end),tStar+.25); m=tDense>=tL & tDense<=tR;
fig=mkfig(619,278); ax=axes(fig,'Position',[.105 .175 .875 .755]); hold(ax,'on'); box(ax,'off');
patch(ax,[tL tR tR tL],[-1 -1 0 0],paleRed,'EdgeColor','none');
plot(ax,tDense(m),marginLqi(m)*1e3,'--','Color',gray,'LineWidth',2.1); plot(ax,tDense(m),marginQp(m)*1e3,'-','Color',blue,'LineWidth',2.5); yline(ax,0,'-','Color',red,'LineWidth',1.0);
plot(ax,tDense(ixMin),minLqi*1e3,'o','Color',gray,'MarkerFaceColor','w','LineWidth',1.3,'MarkerSize',5); plot(ax,tDense(ixQp),minQp*1e3,'o','Color',blue,'MarkerFaceColor',blue,'MarkerSize',5);
text(ax,tDense(ixMin)-.003,minLqi*1e3+.18,'LQI  −0.783 mm','FontName','Arial','FontSize',10,'Color',gray,'HorizontalAlignment','right','BackgroundColor','w','EdgeColor','none');
text(ax,tDense(ixQp)-.003,minQp*1e3+.14,'LQI+QP  +0.422 mm','FontName','Arial','FontSize',10,'Color',blue,'FontWeight','bold','HorizontalAlignment','right','BackgroundColor','w','EdgeColor','none');
xlim(ax,[tL tR]); ylim(ax,[-1 5]); yticks(ax,-1:1:5); styleax(ax,10); xlabel(ax,'t / s','FontSize',10); ylabel(ax,'collision margin / mm','FontSize',10);
text(ax,.01,.985,'C1  碰撞安全裕度','Units','normalized','Color',blue,'FontName','Microsoft YaHei','FontWeight','bold','FontSize',11,'VerticalAlignment','top'); exportfig(fig,outDir,'fig3C_collision_margin'); close(fig);

% C2: one-metric tracking-accuracy view. The magnified, shared scale makes
% the small difference honest and readable without implying a large gain.
fig=mkfig(365,278); ax=axes(fig,'Position',[.14 .20 .82 .62]); hold(ax,'on'); box(ax,'off');
plot(ax,[lqiRms qpRms],[1 1],'-','Color',lightGray,'LineWidth',2.2);
plot(ax,lqiRms,1,'o','Color',gray,'MarkerFaceColor','w','MarkerSize',8,'LineWidth',1.6);
plot(ax,qpRms,1,'o','Color',blue,'MarkerFaceColor',blue,'MarkerSize',8);
xlim(ax,[1.158 1.167]); ylim(ax,[.72 1.35]); yticks(ax,[]); xticks(ax,1.158:.003:1.167); styleax(ax,9);
text(ax,lqiRms,1.245,{'LQI','1.164 mm'},'FontName','Arial','FontSize',9.7,'Color',gray,'HorizontalAlignment','center','VerticalAlignment','top');
text(ax,qpRms,1.245,{'LQI+QP','1.161 mm'},'FontName','Arial','FontSize',9.7,'Color',blue,'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','top');
text(ax,mean([lqiRms qpRms]),.81,'Δ −0.0029 mm（−0.25%）','FontName','Arial','FontSize',10.0,'Color',blue,'FontWeight','bold','HorizontalAlignment','center');
annotation(fig,'textbox',[.01 .925 .98 .06],'String','C2  平移跟踪精度（RMS）','LineStyle','none','Color',blue,'FontName','Microsoft YaHei','FontWeight','bold','FontSize',11.5,'VerticalAlignment','top');
exportfig(fig,outDir,'fig3C_tracking_rms'); close(fig);

% D1: independent test-only force trajectory comparison around the selected window.
fig=mkfig(403,278); ax=axes(fig,'Position',[.13 .175 .85 .755]); hold(ax,'on'); box(ax,'off');
h1=plot(ax,time(win),trueF(win,legIdx),'-','Color',black,'LineWidth',1.6); h2=plot(ax,time(win),grayF(win,legIdx),'--','Color',gray,'LineWidth',1.2); h3=plot(ax,time(win),narxF(win,legIdx),'-','Color',blue,'LineWidth',1.4);
lg=legend(ax,[h1 h2 h3],{'F_{true}','F_{gray}','F_{hat}'},'Location','northwest','FontName','Arial','FontSize',8); lg.Box='off';
styleax(ax,9); xlabel(ax,'t / s','FontSize',9); ylabel(ax,'force / N','FontSize',9); exportfig(fig,outDir,'fig3D_force_validation'); close(fig);

% D2: concise six-leg horizontal dumbbell with title and legend on one row.
fig=mkfig(263,278); ax=axes(fig,'Position',[.20 .15 .77 .68]); hold(ax,'on'); box(ax,'off'); yy=6:-1:1;
for k=1:6
  plot(ax,[legGray(k) legNarx(k)]*100,[yy(k) yy(k)],'-','Color',lightGray,'LineWidth',1.3); plot(ax,legGray(k)*100,yy(k),'s','Color',gray,'MarkerFaceColor','w','MarkerSize',4,'LineWidth',1); plot(ax,legNarx(k)*100,yy(k),'o','Color',blue,'MarkerFaceColor',blue,'MarkerSize',4);
  text(ax,legNarx(k)*100-.012,yy(k)+.17,sprintf('%.2f',legNarx(k)*100),'Color',blue,'FontName','Arial','FontSize',7.4,'HorizontalAlignment','right');
end
xlim(ax,[.60 1.18]); ylim(ax,[.45 7.4]); yticks(ax,1:6); yticklabels(ax,{'leg 6','leg 5','leg 4','leg 3','leg 2','leg 1'}); styleax(ax,8); ax.YGrid='off'; xlabel(ax,'test NRMSE / %','FontSize',8);
text(ax,.01,.985,'D2  测试 NRMSE','Units','normalized','FontName','Microsoft YaHei','FontSize',7.8,'FontWeight','bold','Color',blue,'VerticalAlignment','top');
text(ax,.43,.985,'○ 灰箱   ● 灰箱+NARX','Units','normalized','FontName','Microsoft YaHei','FontSize',6.9,'Color',[.25 .25 .25],'VerticalAlignment','top');
exportfig(fig,outDir,'fig3D_leg_nrmse'); close(fig);

metrics=table(string({'lqi_position_rms';'qp_position_rms';'lqi_min_collision_margin';'qp_min_collision_margin';'lqi_negative_margin_samples';'lqi_negative_margin_duration';'qp_qp_p95';'qp_deadline_misses';'gray_overall_test_nrmse';'narx_overall_test_nrmse';'narx_relative_improvement';'reversal_time';'reversal_force_swing_threshold'}),[lqiRms;qpRms;minLqi*1e3;minQp*1e3;nnz(neg);negDuration;p95;nnz(qptime>dtQp);overallGray*100;overallNarx*100;improve;reversalTime;threshold],string({'mm';'mm';'mm';'mm';'samples';'s';'ms';'ticks';'%FS';'%FS';'%';'s';'N'}),'VariableNames',{'metric','value','unit'});
for k=1:6, metrics=[metrics; table(string({sprintf('leg%d_gray_test_nrmse',k);sprintf('leg%d_narx_test_nrmse',k)}),[legGray(k);legNarx(k)]*100,repmat("%FS",2,1),'VariableNames',{'metric','value','unit'})]; end
writetable(metrics,fullfile(outDir,'page3_metrics_summary.csv'));
fid=fopen(fullfile(outDir,'page3_notes.md'),'w','n','UTF-8'); fprintf(fid,['# 第3页数据口径\n\n- Strict control MAT: `%s`；`ct_lqi` 对照 `full_qp_nominal`，QP 为 matched nominal plant + ideal force 的严格模型研究。\n- 碰撞裕度为每时刻 3 通道最小值；平移 RMS 只使用 poseError 前三维。\n- ID MAT: `%s`；D 图均为测试集。F_true 为高保真 teacher model，不是实测力。NARX 使用 F_hat = F_gray + r_hat。\n- 图 D1 固定轴4，测试集有效换向 t=%.3f s；总体 NRMSE %.6f%%FS → %.6f%%FS。\n\n[Sources]\n- %s\n- %s\n- D:\\NPU\\简历及相关资料\\702\\线下面试\\2-2.pptx\n'],strictMat,idMat,reversalTime,overallGray*100,overallNarx*100,strictMat,idMat); fclose(fid);
fprintf('LQI_MIN=%.6f QP_MIN=%.6f P95=%.6f ID=%.6f_to_%.6f REV=%.6f\n',minLqi*1e3,minQp*1e3,p95,overallGray*100,overallNarx*100,reversalTime);
end

function fig=mkfig(w,h), fig=figure('Visible','off','Color','w','Units','pixels','Position',[80 80 w h]); end
function styleax(ax,fs), grid(ax,'on'); ax.GridColor=[.88 .88 .88]; ax.GridAlpha=.75; ax.FontName='Arial'; ax.FontSize=fs; ax.LineWidth=.7; ax.TickDir='out'; end
function exportfig(fig,outDir,stem), exportgraphics(fig,fullfile(outDir,[stem '.png']),'Resolution',300); exportgraphics(fig,fullfile(outDir,[stem '.svg']),'ContentType','vector'); end
function v=projectPercentile(d,p), d=sort(d(isfinite(d))); k=1+(numel(d)-1)*p/100; lo=floor(k); hi=ceil(k); if lo==hi,v=d(lo);else,v=d(lo)+(k-lo)*(d(hi)-d(lo));end,end
