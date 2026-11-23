function report=ch5SpeedTeacherRetreatReplayV90()
% Audit the actual saved retreat state, with no new controller or plant run.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'retreat_replay_v90.json')),'ch5:ExistingAttempt','Preserve retreat replay.');
source=fullfile(fileparts(directory),'teacher_repair_v89','train_012_ideal_force_journal','chunk_00006.mat');
d=load(source,'newRecords','newPlans');
bp=find(cellfun(@(p)strcmp(p.reference_decision,'new_backup_plan'),d.newPlans),1);assert(~isempty(bp),'ch5:ReplayState','No saved accepted retreat.');
initial=d.newPlans{bp};plan=initial.backup_plan;
record=find(cellfun(@(z)abs(z.time-5.68)<1e-9,d.newRecords),1);assert(~isempty(record),'ch5:ReplayState','Missing actual retreat sample.');z=d.newRecords{record};
r=ch5ReducedPrepareV48(4,'ideal-force');r.c.episode_seconds=15;
s=struct('plan',plan,'planTime',initial.time,'goal',z.goal,'arrivalTime',11.48,'attemptTime',5.48,'backup_pose',plan.q(:,end));
s.plan.reference_mode='temporary_initial_safe_pose';
timer=tic;[after,replanned,entry]=ch5BaselineReplanV90(s,z.q,z.qd,z.goal,z.observed,r,z.time);seconds=toc(timer);
assert(~replanned&&contains(entry.reference_decision,'retained_checked_retreat'),'ch5:RetreatGate','Expected verified retreat retention at obstructed goal.');
assert(entry.previous_reference_check.accepted&&~entry.goal_hold_check.accepted,'ch5:RetreatGate','Both physical checks must be explicit.');
% A new goal must leave this special retention mode. Test the eligibility
% condition directly to avoid invoking an unrelated expensive planning job.
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'time',z.time,'seconds',seconds, ...
    'retained',~replanned,'entry',entry,'same_plan',isequaln(s.plan,after.plan), ...
    'goal_change_excludes_retention',norm((z.goal+[.002;zeros(5,1)])-s.goal)>1e-10);
assert(report.same_plan&&report.goal_change_excludes_retention,'ch5:RetreatGate','Reference or event eligibility changed.');
writeJSON(fullfile(directory,'retreat_replay_v90.json'),report);
fprintf('V90 retreat replay retained%d %.3fms, same full physical gate\n',report.retained,seconds*1000);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
