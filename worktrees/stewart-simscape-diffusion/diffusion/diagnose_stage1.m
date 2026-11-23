function diagnose_stage1()
c=setupDiffusion(); s=load(fullfile(c.output,'stage1','simple_01.mat')); b=s.result.bundle; task=s.task;
for label={'minClearance','maxLegViolation','maxForceViolation','minSigma','maxCondition','boundaryError'}
    fprintf('%s=%g\n',label{1},s.result.report.(label{1}));
end
[worst,idx]=min(s.result.report.clearance(:)); t=s.result.report.time(idx);
fprintf('Worst %g at t=%g\n',worst,t);
d=zeros(1,size(b.X,2)); for k=1:numel(d), d(k)=min(dpClearance(b.X(1:6,k),task)); end
fprintf('NLP node/midpoint minimum independent clearance %g\n',min(d));
tr=dpReconstruct(b,b.tNode); fprintf('Node reconstruction mismatch %g\n',max(max(abs(tr.q-b.X(1:6,1:61)))));
[~,ix]=min(d); q=b.X(1:6,ix); disp(q.');
p=dpPhysics(task.model); [ds,ns]=dpClearance(q,task,true); nn=ns(:,1);
expr=full(dpGapExpr(q,nn,task.scene.hood.obstacles(1),task,p));
fprintf('Support numeric %g expression %g norm %g\n',ds(1),expr,norm(nn));
end
