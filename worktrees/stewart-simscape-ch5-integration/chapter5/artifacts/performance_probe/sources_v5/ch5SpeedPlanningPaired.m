function report=ch5SpeedPlanningPaired()
[c,root]=ch5Init();[physical,scene,ad,strict]=ch5Physics(c);
r=struct('c',c,'physical',physical,'scene',scene,'ad',ad,'strict',strict,'collision',ch5CollisionModel(physical,scene,c));
out=fullfile(root,'chapter5','artifacts','performance_probe','planning_paired');if ~isfolder(out),mkdir(out);end
saved=load(fullfile(root,'chapter5','artifacts','development','development_004_ideal_force_baseline_0.mat'),'records','plans');
first=saved.records{1};original=saved.plans{1}.plan;times=0:c.internal_step:original.t(end);
[q,v,a]=ch5SpeedReferenceBatch(original,times);referenceError=0;
for j=1:numel(times)
    [oq,ov,oa]=ch5Reference(original,times(j));referenceError=max(referenceError,max(abs([q(:,j)-oq;v(:,j)-ov;a(:,j)-oa])));
end
assert(referenceError<1e-9);ch5SpeedPlanPostcheck(original,first.observed,r);
report=struct('pid',feature('getpid'),'code_identity',ch5SpeedIdentity(),'dense_rows',{{}},'planning_rows',{{}}, ...
    'reference_error',referenceError,'scope','Identical recorded state and task for cold/warm comparison');
for sample=1:3
    timer=tic;old=ch5PlanPostcheck(original,first.observed,r);oldSeconds=toc(timer);
    timer=tic;new=ch5SpeedPlanPostcheck(original,first.observed,r);newSeconds=toc(timer);
    assert(old.accepted==new.accepted&&abs(old.minimum_margin-new.minimum_margin)<1e-8);
    report.dense_rows{end+1}=struct('sample',sample,'original_seconds',oldSeconds,'batched_seconds',newSeconds, ...
        'minimum_margin_error',abs(old.minimum_margin-new.minimum_margin),'accepted',new.accepted,'samples',new.samples);
end
clear ch5SpeedIhsid
for sample=1:3
    if sample==1,time=0;label='initial_cold';else,time=.1;if sample==2,label='same_state_warm';else,label='same_state_cold';end;end
    [~,index]=min(abs(cellfun(@(s)s.time,saved.records)-time));entry=saved.records{index};
    timer=tic;plan=ch5SpeedIhsid(entry.q,entry.qd,entry.goal,entry.observed,r,6-time,time);seconds=toc(timer);
    report.planning_rows{end+1}=struct('label',label,'seconds',seconds,'accepted',plan.accepted,'state',[entry.q;entry.qd], ...
        'goal',entry.goal,'observed',entry.observed,'duration',6-time,'plan',plan);
    save(fullfile(out,'study.mat'),'report','-v7');
    fprintf('Planning paired %s %.4fs accepted=%d\n',label,seconds,plan.accepted);
end
assert(isequal(report.planning_rows{2}.state,report.planning_rows{3}.state));
report.completed=true;save(fullfile(out,'study.mat'),'report','-v7');
fid=fopen(fullfile(out,'study.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
