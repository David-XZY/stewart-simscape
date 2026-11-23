function report=ch5SpeedTeacherSafetyReplayV88()
% Compare the complete public safety filter at identical stored inputs.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'safety_replay_v88.json')),'ch5:ExistingAttempt','Preserve public filter replay.');
d=load(fullfile(fileparts(directory),'teacher_pilot_v82','train_016_nonideal_force.mat'),'records','plans');
r=ch5ReducedPrepareV48(4,'nonideal-force');report=struct('completed',false,'formal',false,'pid',feature('getpid'),'rows',{{}});
for time=[1,5,13.72,13.76,13.78]
    [~,index]=min(abs(cellfun(@(z)z.time,d.records)-time));z=d.records{index};plan=[];planTime=0;
    for pi=1:numel(d.plans)
        p=d.plans{pi};if p.time>z.time+1e-9,break;end
        if strcmp(p.reference_decision,'new_task_plan'),plan=p.plan;planTime=p.time;
        elseif strcmp(p.reference_decision,'new_backup_plan'),plan=p.backup_plan;planTime=p.time;end
    end
    assert(~isempty(plan),'ch5:SafetyReplay','No original reference for replay.');
    [qr,vr,ar]=ch5Reference(plan,max(0,z.time-planTime));
    [old,base]=ch5TeacherSafetyV64(z.x,qr,vr,ar,z.nominal,z.previous_u,z.observed,r,z.time);
    timer=tic;[u,new]=ch5TeacherSafetyV88(z.x,qr,vr,ar,z.nominal,z.previous_u,z.observed,r,z.time);seconds=toc(timer);
    [check,~]=ch5ReducedHoldAuditV22(z.x,z.previous_u,u,z.observed,r);
    row=struct('time',z.time,'old_feasible',base.feasible,'new_feasible',new.feasible, ...
        'command_difference',norm(u-old,inf),'used_restart',new.feasible_seed_restart,'seconds',seconds, ...
        'check',check,'diagnostic',new);
    report.rows{end+1}=row;
    if base.feasible,assert(new.feasible&&norm(u-old,inf)<1e-8,'ch5:SafetyReplay','A successful original command changed.');end
    assert(new.feasible&&check.accepted&&new.hardConstraintViolation<=10*r.strict.solver.constraintTolerance,'ch5:SafetyReplay','Replay command did not pass all original checks.');
    fprintf('V88 public safety replay%.2f old%d new%d restart%d change%.3g\n',z.time,base.feasible,new.feasible,new.feasible_seed_restart,norm(u-old,inf));
end
assert(report.rows{end}.used_restart,'ch5:SafetyReplay','The actual failed case did not exercise restart.');
blocked=z.observed;blocked(:,1)=0;blocked(1:3,1)=z.q(1:3);blocked(4:6,1)=.5;
blocked(7:15,1)=reshape(eye(3),9,1);blocked(22,1)=1;
[~,negative]=ch5TeacherSafetyV88(z.x,qr,vr,ar,z.nominal,z.previous_u,blocked,r,z.time);
report.negative_check=struct('current_collision_rejected',~negative.feasible, ...
    'fallback_not_certified',~negative.fallbackSafetyCertified);
assert(~negative.feasible&&~negative.fallbackSafetyCertified,'ch5:SafetyReplay','An impossible current collision was labeled safe.');
report.completed=true;writeJSON(fullfile(directory,'safety_replay_v88.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
