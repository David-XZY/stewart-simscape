function report=ch5SpeedTeacherCausalReserveV91()
% Replay the first V90 failure and reject real physical/future violations.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','control_force_v62');
file=fullfile(base,'teacher_repair_v91','causal_reserve_replay.json');
assert(~isfile(file),'ch5:ExistingAttempt','Preserve reserve repair evidence.');
d=load(fullfile(base,'teacher_repair_v90','train_016_nonideal_force.mat'),'records','plans');
r=ch5ReducedPrepareV48(4,'nonideal-force');r.c.episode_seconds=15;rows={};
for time=[14.06,14.12,14.14]
    index=find(cellfun(@(z)abs(z.time-time)<1e-9,d.records),1);z=d.records{index};
    plan=[];planTime=0;
    for j=1:numel(d.plans)
        p=d.plans{j};if p.time>time+1e-10,break;end
        if strcmp(p.reference_decision,'new_task_plan'),plan=p.plan;planTime=p.time;
        elseif strcmp(p.reference_decision,'new_backup_plan'),plan=p.backup_plan;planTime=p.time;end
    end
    [qr,vr,ar]=ch5Reference(plan,time-planTime);
    [old,oldDiagnostic]=ch5TeacherSafetyV89(z.x,qr,vr,ar,z.nominal,z.previous_u,z.observed,r,time);
    timer=tic;[u,diagnostic]=ch5TeacherSafetyV91(z.x,qr,vr,ar,z.nominal,z.previous_u,z.observed,r,time);seconds=toc(timer);
    [checked,~]=ch5TeacherHoldAuditV91(z.x,z.previous_u,u,z.observed,r);
    assert(diagnostic.feasible&&checked.accepted,'ch5:CausalReserve','Repair must pass at %.2fs.',time);
    if oldDiagnostic.feasible
        assert(norm(u-old,inf)<1e-10,'ch5:CausalReserve','A successful original command changed.');
    else
        assert(abs(time-14.14)<1e-9&&diagnostic.feasible_seed_restart,'ch5:CausalReserve','Unexpected failed checkpoint.');
        [oldCheck,~]=ch5TeacherHoldAuditV91(z.x,z.previous_u,old,z.observed,r);
        assert(~oldCheck.accepted,'ch5:CausalReserve','An unsafe future command was accepted.');
    end
    rows{end+1}=struct('time',time,'seconds',seconds,'old_feasible',oldDiagnostic.feasible, ...
        'new_feasible',diagnostic.feasible,'force_difference',norm(u-old,inf),'check',checked,'diagnostic',diagnostic);
end
z=d.records{find(cellfun(@(z)abs(z.time-14.14)<1e-9,d.records),1)};
bad=z.x;bad(19:24)=0;
[physicalRejection,~]=ch5TeacherHoldAuditV91(bad,z.previous_u,z.previous_u,z.observed,r);
assert(~physicalRejection.accepted&&physicalRejection.internal_check.mechanical_violation>1,'ch5:CausalReserve','True current acceleration violation escaped.');
o=z.observed;o(1:3,1)=z.q(1:3);o(4:6,1)=ones(3,1)*.5;o(7:15,1)=reshape(eye(3),9,1);o(16:21,1)=0;o(22,1)=1;
[collisionRejection,~]=ch5TeacherHoldAuditV91(z.x,z.previous_u,z.previous_u,o,r);
assert(~collisionRejection.accepted&&collisionRejection.internal_check.collision_violation>0,'ch5:CausalReserve','Current collision escaped.');
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'rows',{rows}, ...
    'physical_violation_rejected',physicalRejection,'collision_rejected',collisionRejection, ...
    'physical_acceleration_limit',r.physical.actuator.lddotMax,'future_reserved_limit',ch5PredictionAccelerationLimit(r), ...
    'scope','Known delay samples use unchanged physical limits; original 1% reserve remains at all future and actuator transient samples.');
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('V91 causal reserve: failed state repaired; successful forces unchanged; physical/collision guards rejected.\n');
end
