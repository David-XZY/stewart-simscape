function test_sg_repair()
c=sgSetup(); s=load(fullfile(c.source,'benchmark','simple_20262041','diffusion','benchmark.mat'));
task=s.task; cp=s.candidates(:,:,1); tr=dpSpline(cp,task.T*.5,task.T); fun=sgRepairPoint(task);
[~,ns]=sgClearanceBatch(tr.q,task); normals=zeros(3,6); centers=zeros(3,6); supports=zeros(1,6); mask=zeros(1,6);
for j=1:numel(task.scene.hood.obstacles)
    o=task.scene.hood.obstacles(j); normals(:,j)=ns(:,j,1); centers(:,j)=o.center_S;
    supports(j)=o.halfSize.'*abs(o.R_S.'*normals(:,j)); mask(j)=1;
end
x=[tr.q;tr.v;tr.a];
% Enumerated axis normals sit on abs-support kinks. Test the smooth AD branch
% off these kinks while retaining exactly the same fixed support planes.
x(4:6)=x(4:6)+[.012;-.018;.009];
[~,g]=fun(x(1:6),x(7:12),x(13:18),normals,centers,supports,mask); g=full(g); fd=zeros(18,1);
for j=1:18
    delta=zeros(18,1); delta(j)=1e-6; a=x+delta; b=x-delta;
    fd(j)=(full(fun(a(1:6),a(7:12),a(13:18),normals,centers,supports,mask))- ...
        full(fun(b(1:6),b(7:12),b(13:18),normals,centers,supports,mask)))/(2e-6);
end
err=norm(fd-g)/max(1,norm(fd)); assert(err<1e-4,'Support-plane gradient mismatch.');
[scores,order]=sgRank(s.candidates,task); %#ok<ASGLU>
clock=tic; [repaired,info]=sgRepair(task,s.candidates(:,:,order(1)),c,clock);
assert(all(isfinite(repaired(:))) && isequal(repaired(:,[1:3,14:16]),s.candidates(:,[1:3,14:16],order(1))));
assert(info.normalizedChangeRms<=.25+1e-12);
fast=sgValidate(struct('kind','spline','cp',repaired,'T',task.T),task,c);
gold=sgAudit(struct('kind','spline','cp',repaired,'T',task.T),task,c);
assert(fast.passed==gold.passed);
first=sgQuality(s.outcome.best.bundle,task,c); c.qualityStep=c.qualityStep/2; second=sgQuality(s.outcome.best.bundle,task,c);
relative=abs(first.commonCostMean-second.commonCostMean)/max(1,abs(second.commonCostMean)); assert(relative<.02);
report=struct('passed',true,'gradientRelativeError',err,'repair',info,'repairAccepted',gold.passed,'qualityGridRelativeChange',relative);
dpWriteJson(fullfile(c.output,'repair_tests.json'),report);
fprintf('SG_REPAIR_PASSED grad=%.3g repair=%.3fs feasible=%d qualityGridChange=%.3g\n',err,info.seconds,gold.passed,relative);
end
