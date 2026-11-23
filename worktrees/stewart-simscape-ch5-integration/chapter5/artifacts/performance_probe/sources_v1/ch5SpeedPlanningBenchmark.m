function report=ch5SpeedPlanningBenchmark()
[c,root]=ch5Init();addpath(fullfile(root,'chapter5','performance'));[physical,scene,ad,strict]=ch5Physics(c);
r=struct('c',c,'physical',physical,'scene',scene,'ad',ad,'strict',strict,'collision',ch5CollisionModel(physical,scene,c));
saved=load(fullfile(root,'chapter5','artifacts','development','development_004_ideal_force_baseline_0.mat'),'records','plans');
first=saved.records{1};original=saved.plans{1}.plan;
times=0:c.internal_step:original.t(end);[q,v,a]=ch5SpeedReferenceBatch(original,times);error=0;
for j=1:numel(times)
    [oq,ov,oa]=ch5Reference(original,times(j));error=max(error,max(abs([q(:,j)-oq;v(:,j)-ov;a(:,j)-oa])));
end
assert(error<1e-9);oldCheck=ch5PlanPostcheck(original,first.observed,r);
newCheck=ch5SpeedPlanPostcheck(original,first.observed,r);assert(abs(oldCheck.minimum_margin-newCheck.minimum_margin)<1e-8);
timer=tic;ch5SpeedPlanPostcheck(original,first.observed,r);denseSeconds=toc(timer);
report=struct('rows',{{}},'reference_error',error,'dense_margin_error',abs(oldCheck.minimum_margin-newCheck.minimum_margin),'dense_seconds',denseSeconds);
for j=1:6
    t=(j-1)*.2;[~,index]=min(abs(cellfun(@(x)x.time,saved.records)-t));entry=saved.records{index};
    timer=tic;p=ch5SpeedIhsid(entry.q,entry.qd,entry.goal,entry.observed,r,6-t,t);seconds=toc(timer);
    report.rows{end+1}=struct('time',t,'seconds',seconds,'accepted',p.accepted,'plan',p);
    save(fullfile(root,'chapter5','artifacts','performance_probe','planning.mat'),'report','-v7');
    fprintf('Speed IHSID time %.1f: %.6fs accepted=%d\n',t,seconds,p.accepted);
end
report.completed=true;save(fullfile(root,'chapter5','artifacts','performance_probe','planning.mat'),'report','-v7');
fid=fopen(fullfile(root,'chapter5','artifacts','performance_probe','planning.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
