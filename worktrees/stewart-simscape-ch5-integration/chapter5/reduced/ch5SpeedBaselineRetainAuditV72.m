function report=ch5SpeedBaselineRetainAuditV72()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'baseline_retain_audit_v72.json')),'ch5:ExistingAttempt','Preserve retained-reference audit.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'formal',false,'rows',{{}},'negative_checks',[]);example=[];
for mode=["ideal-force","nonideal-force"]
    d=load(fullfile(directory,'teacher_full_v67',sprintf('train_004_%s.mat',strrep(mode,'-','_'))),'records','plans');r=ch5ReducedPrepareV48(4,mode);
    s=struct('goal',nan(6,1),'plan',struct('accepted',false),'planTime',0);
    for pi=1:numel(d.plans)
        e=d.plans{pi};[~,ri]=min(abs(cellfun(@(z)z.time,d.records)-e.time));z=d.records{ri};
        if s.plan.accepted&&isfield(s.plan,'reference_mode')&&strcmp(s.plan.reference_mode,'task')
            timer=tic;[retained,entry]=ch5BaselineRetainV72(s,z.q,z.qd,z.goal,z.observed,r,e.time);seconds=toc(timer);
            report.rows{end+1}=struct('mode',char(mode),'time',e.time,'retained',retained,'check',entry,'seconds',seconds);
            if retained,example=struct('s',s,'z',z,'r',r,'time',e.time);end
        end
        if e.plan.accepted&&strcmp(e.reference_decision,'new_task_plan')
            s.plan=e.plan;s.plan.reference_mode='task';s.planTime=e.time;
        elseif ~isempty(e.backup_plan)&&e.backup_plan.accepted
            s.plan=e.backup_plan;s.plan.reference_mode='backup';s.planTime=e.time;
        end
        s.goal=z.goal;
    end
end
assert(~isempty(example),'ch5:RetainAudit','No original reference eligible for validated reuse.');
s=example.s;z=example.z;r=example.r;goal=z.goal;goal(1)=goal(1)+.002;
[changedGoal,~]=ch5BaselineRetainV72(s,z.q,z.qd,goal,z.observed,r,example.time);
o=z.observed;o(:,1)=0;o(1:3,1)=z.goal(1:3);o(4:6,1)=.2;o(7:15,1)=reshape(eye(3),9,1);o(22,1)=1;
[newCollision,~]=ch5BaselineRetainV72(s,z.q,z.qd,z.goal,o,r,example.time);
q=z.q;q(1)=q(1)+.005;[trackingDrift,~]=ch5BaselineRetainV72(s,q,z.qd,z.goal,z.observed,r,example.time);
report.negative_checks=struct('goal_change_rejected',~changedGoal,'new_goal_obstacle_rejected',~newCollision,'tracking_drift_rejected',~trackingDrift);
assert(~changedGoal&&~newCollision&&~trackingDrift,'ch5:RetainAudit','Changed task, obstacle or tracking state was wrongly reused.');
report.completed=true;fid=fopen(fullfile(directory,'baseline_retain_audit_v72.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(any(cellfun(@(z)z.retained,report.rows)),'ch5:RetainAudit','No original reference eligible for validated reuse.');
end
