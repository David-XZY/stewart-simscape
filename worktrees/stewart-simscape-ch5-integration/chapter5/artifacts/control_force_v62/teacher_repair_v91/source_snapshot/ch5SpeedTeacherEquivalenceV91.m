function report=ch5SpeedTeacherEquivalenceV91()
% All-sample force comparison against each physically admitted V90 full run.
% This is replay evidence, explicitly not a new Simscape execution/timing run.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','control_force_v62');
directory=fullfile(base,'teacher_repair_v91');file=fullfile(directory,'equivalence.json');
assert(~isfile(file),'ch5:ExistingAttempt','Preserve equivalence audits.');
summary=jsondecode(fileread(fullfile(base,'teacher_repair_v90','summary.json')));
assert(summary.completed,'ch5:EquivalenceGate','Require all V90 result states first.');
report=struct('completed',false,'formal',false,'rows',{{}},'pid',feature('getpid'));
for ci=1:numel(summary.rows)
    old=summary.rows(ci);if ~old.admitted,continue;end
    name=sprintf('%s_%s',old.task,strrep(old.mode,'-','_'));
    source=fullfile(base,'teacher_repair_v90',[name,'.mat']);data=load(source,'records','plans');
    r=ch5ReducedPrepareV48(4,string(old.mode));r.c.episode_seconds=15;
    times=cellfun(@(z)z.time,data.records);N=numel(times);difference=zeros(1,N);force=zeros(6,N);
    feasible=false(1,N);fallback=feasible;baselineDifference=zeros(1,N);changed=[];timer=tic;
    plan=[];planTime=0;pi=1;
    for j=1:N
        z=data.records{j};
        while pi<=numel(data.plans)&&data.plans{pi}.time<=z.time+1e-9
            p=data.plans{pi};
            if strcmp(p.reference_decision,'new_task_plan'),plan=p.plan;planTime=p.time;
            elseif strcmp(p.reference_decision,'new_backup_plan'),plan=p.backup_plan;planTime=p.time;end
            pi=pi+1;
        end
        assert(~isempty(plan),'ch5:EquivalenceGate','Missing executed reference.');
        [qr,vr,ar]=ch5Reference(plan,max(0,z.time-planTime));
        [u,d]=ch5TeacherSafetyV91(z.x,qr,vr,ar,z.nominal,z.previous_u,z.observed,r,z.time);
        force(:,j)=u;difference(j)=norm(u-z.u,inf);feasible(j)=d.feasible;fallback(j)=d.usedFallback;
        if difference(j)>1e-10
            [v89,~]=ch5TeacherSafetyV89(z.x,qr,vr,ar,z.nominal,z.previous_u,z.observed,r,z.time);
            baselineDifference(j)=norm(v89-z.u,inf);changed(end+1)=j; %#ok<AGROW>
        end
    end
    seconds=toc(timer);equivalent=max(difference)<=1e-10;
    row=struct('task',old.task,'mode',old.mode,'completed',true,'samples',N,'simulated_end_seconds',times(end), ...
        'force_equivalent',equivalent,'maximum_force_difference_N',max(difference),'changed_samples',numel(changed), ...
        'changed_times',times(changed),'maximum_v89_replay_difference_N',max(baselineDifference), ...
        'seconds',seconds,'source_full_run',source,'source_admitted',old.admitted, ...
        'new_fallback_count',sum(fallback),'original_fallback_count',old.metrics.fallback_count, ...
        'scope','Saved 751-state/controller-input replay; original V90 full Simscape trace and wall timing remain attributed to V90.');
    save(fullfile(directory,[name,'_equivalence.mat']),'times','force','difference','feasible','fallback','row','-v7');
    report.rows{end+1}=row;
    fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    fprintf('V91 force equivalence %s %s unchanged%d max%.3g N over%d samples\n',old.task,old.mode,equivalent,max(difference),N);
end
report.completed=true;fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
