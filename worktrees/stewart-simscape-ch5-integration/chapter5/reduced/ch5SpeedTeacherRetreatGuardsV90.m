function report=ch5SpeedTeacherRetreatGuardsV90()
% Exercise the actual V90 dispatch, using saved state and a closed solve budget.
% The closed budget suppresses unrelated NLP work only after reuse is rejected.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','control_force_v62');
file=fullfile(base,'teacher_repair_v90','retention_guards.json');
assert(~isfile(file),'ch5:ExistingAttempt','Preserve guard audit attempts.');
d=load(fullfile(base,'teacher_repair_v89','train_012_ideal_force_journal','chunk_00006.mat'),'newRecords','newPlans');
bp=find(cellfun(@(p)strcmp(p.reference_decision,'new_backup_plan'),d.newPlans),1);
initial=d.newPlans{bp};plan=initial.backup_plan;
index=find(cellfun(@(z)abs(z.time-5.68)<1e-9,d.newRecords),1);z=d.newRecords{index};
r=ch5ReducedPrepareV48(4,'ideal-force');r.c.episode_seconds=z.time+.01;
s=struct('plan',plan,'planTime',initial.time,'goal',z.goal,'arrivalTime',z.time, ...
    'attemptTime',5.48,'backup_pose',plan.q(:,end));s.plan.reference_mode='temporary_initial_safe_pose';
rows={};
for scenario=1:5
    q=z.q;qd=z.qd;goal=z.goal;o=z.observed;input=s;
    switch scenario
        case 1,label='unchanged_goal_and_valid_retreat';expected=true;
        case 2,label='changed_goal';goal(1)=goal(1)+.002;expected=false;
        case 3,label='tracking_error_exceeds_original_limit';q(1)=q(1)+.002;expected=false;
        case 4
            label='retreat_blocked_by_observed_obstacle';expected=false;
            o(1:3,1)=q(1:3);o(4:6,1)=ones(3,1)*.5;o(7:15,1)=reshape(eye(3),9,1);
            o(16:21,1)=0;o(22,1)=1;
        case 5,label='goal_hold_is_clear';o(22,:)=0;expected=false;
    end
    timer=tic;[after,replanned,entry]=ch5BaselineReplanV90(input,q,qd,goal,o,r,z.time);seconds=toc(timer);
    retained=strcmp(entry.reference_decision,'retained_checked_retreat_while_goal_hold_obstructed');
    assert(retained==expected,'ch5:RetentionGuard','Unexpected retention for %s.',label);
    if retained
        assert(~replanned&&isequaln(after.plan,input.plan)&&after.planTime==input.planTime,'ch5:RetentionGuard','Retained reference was modified.');
        assert(isequaln(after.goal,input.goal)&&entry.previous_reference_check.accepted&&~entry.goal_hold_check.accepted,'ch5:RetentionGuard','Lost task goal or physical checks.');
        assert(isempty(entry.plan.attempts)&&isempty(entry.backup_plan),'ch5:RetentionGuard','An IHSID call appeared in the retained branch.');
    end
    rows{end+1}=struct('scenario',label,'passed',true,'retained',retained,'seconds',seconds, ...
        'reference_decision',entry.reference_decision,'entry',entry);
end
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'rows',{rows}, ...
    'scope','Actual dispatch guards on saved physical state; closed planning budget avoids unrelated NLP solves after guard rejection. Full 15 s simulation is reported separately.');
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('V90 retention dispatch: all five guard scenarios passed.\n');
end
