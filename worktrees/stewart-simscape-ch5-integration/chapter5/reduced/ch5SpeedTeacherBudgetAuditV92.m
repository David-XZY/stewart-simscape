function report=ch5SpeedTeacherBudgetAuditV92()
% Verify scheduler equivalence on every retained orbit state; diagnose V91 hold.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','control_force_v62');
directory=fullfile(base,'teacher_repair_v92');file=fullfile(directory,'budget_audit.json');
assert(~isfile(file),'ch5:ExistingAttempt','Preserve scheduling audit.');
report=struct('completed',false,'formal',false,'pid',feature('getpid'),'rows',{{}});
for mode=["ideal-force","nonideal-force"]
    name=['train_012_',strrep(char(mode),'-','_'),'.mat'];source=fullfile(base,'teacher_repair_v90',name);
    data=load(source,'records','plans');r=ch5ReducedPrepareV48(4,mode);r.c.episode_seconds=15;
    s=struct('plan',struct('accepted',false),'planTime',-inf,'goal',nan(6,1),'arrivalTime',6,'attemptTime',-inf,'backup_pose',data.records{1}.q);
    retained=0;expiredRetained=0;seconds=0;
    for j=1:numel(data.plans)
        p=data.plans{j};index=round(p.time/r.c.Ts)+1;z=data.records{index};
        assert(abs(z.time-p.time)<1e-9,'ch5:BudgetAudit','Saved sample mismatch.');
        if strcmp(p.reference_decision,'retained_checked_retreat_while_goal_hold_obstructed')
            expiredRetained=expiredRetained+double(s.arrivalTime-p.time<.5);
            assert(s.arrivalTime-p.time>=.5,'ch5:BudgetAudit','Orbit retention would change; require a new full run.');
            timer=tic;[after,replanned,entry]=ch5BaselineReplanV92(s,z.q,z.qd,z.goal,z.observed,r,p.time);seconds=seconds+toc(timer);
            assert(~replanned&&strcmp(entry.reference_decision,p.reference_decision)&&isequaln(after.plan,s.plan),'ch5:BudgetAudit','Retained orbit branch changed.');
            retained=retained+1;
        end
        if isfield(p,'arrival_time'),s.arrivalTime=p.arrival_time;end
        if strcmp(p.reference_decision,'new_task_plan'),s.plan=p.plan;s.plan.reference_mode='task';s.planTime=p.time;
        elseif strcmp(p.reference_decision,'new_backup_plan'),s.plan=p.backup_plan;s.plan.reference_mode='temporary_initial_safe_pose';s.planTime=p.time;end
        s.goal=z.goal;s.attemptTime=p.time;
    end
    report.rows{end+1}=struct('task','train_012','mode',char(mode),'source_full_run',source, ...
        'completed',true,'retained_states_checked',retained,'expired_retained_states',expiredRetained, ...
        'scheduler_equivalent',expiredRetained==0,'seconds',seconds, ...
        'scope','Only budget-expired retention changes in V92. Every previously retained state is checked by the actual V92 function; all other branches call the identical V83 planner. The safety filter remains V89.');
end
data=load(fullfile(base,'teacher_repair_v91','train_016_nonideal_force.mat'),'output','r');
simout=data.output.get('simout');time=simout.x.Xr.Time(:);Q=squeeze(simout.x.Xr.Data);if size(Q,1)~=numel(time),Q=Q';end
vel=simout.ch5SpatialVelocity;vt=vel.Time(:);V=squeeze(vel.Data);if size(V,1)~=numel(vt),V=V';end
[vt,uniqueRows]=unique(vt,'stable');V=interp1(vt,V(uniqueRows,:),time,'linear');ok=false(size(time));r=data.r;
for j=1:numel(time)
    [q,qd]=ch5AbsoluteState(Q(j,:)',V(j,:)',r.framePose);[goal,~,~,~]=ch5Observe(r.task,time(j));
    rotation=acos(max(-1,min(1,(trace(ch5Rotation(goal(4:6))'*ch5Rotation(q(4:6)))-1)/2)));
    ok(j)=norm(q(1:3)-goal(1:3))<=r.c.position_tolerance&&rotation<=deg2rad(r.c.rotation_tolerance_degrees)&& ...
        norm(qd(1:3))<=r.c.linear_speed_tolerance&&norm(V(j,4:6))<=deg2rad(r.c.angular_speed_tolerance_degrees);
end
last=find(~ok,1,'last');start=nan;hold=0;if ~isempty(last)&&last<numel(time),start=time(last+1);hold=time(end)-start;end
report.v91_terminal_hold=struct('start_seconds',start,'observed_hold_seconds',hold,'required_hold_seconds',r.c.arrival_hold_seconds, ...
    'criterion_unchanged',true,'admitted',hold>=r.c.arrival_hold_seconds-1e-10);
report.completed=true;fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('V92 scheduler: both orbit branches unchanged; V91 terminal hold %.9fs < %.3fs.\n',hold,r.c.arrival_hold_seconds);
end
