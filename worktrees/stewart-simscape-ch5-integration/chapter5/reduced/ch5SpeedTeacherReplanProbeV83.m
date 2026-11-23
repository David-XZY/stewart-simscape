function report=ch5SpeedTeacherReplanProbeV83()
% Replan from the earliest rejected saved state before rerunning a full plant.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'replan_probe.json')),'ch5:ExistingAttempt','Preserve every planning attempt.');
replay=jsondecode(fileread(fullfile(directory,'replay.json')));assert(replay.completed,'ch5:ReplayGate','Require observation-only rejection replay.');
source=fullfile(fileparts(directory),'teacher_pilot_v82');
d=load(fullfile(source,'train_012_ideal_force.mat'),'records','plans');
time=replay.rows(1).new_first_hold_rejection;[~,index]=min(abs(cellfun(@(z)z.time,d.records)-time));z=d.records{index};
first=d.plans{find(cellfun(@(p)strcmp(p.reference_decision,'new_task_plan'),d.plans),1)};
taskData=load(fullfile(source,'tasks.mat'),'tasks');task=taskData.tasks{find(cellfun(@(t)strcmp(t.id,'train_012'),taskData.tasks),1)};
r=ch5ReducedPrepareV48(4,'ideal-force');r.c.episode_seconds=15;
s=struct('plan',first.plan,'planTime',first.time,'goal',z.goal,'arrivalTime',6,'attemptTime',time-.2,'backup_pose',task.q0);
s.plan.reference_mode='task';timer=tic;
[after,replanned,entry]=ch5BaselineReplanV83(s,z.q,z.qd,z.goal,z.observed,r,time);
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'source_task',task.id,'time',time, ...
    'seconds',toc(timer),'replanned',replanned,'reference_decision',entry.reference_decision,'accepted',after.plan.accepted, ...
    'entry',entry,'plan',after.plan,'arrival_time',after.arrivalTime);
save(fullfile(directory,'replan_probe.mat'),'report','r','s','z','after','-v7');writeJSON(fullfile(directory,'replan_probe.json'),report);
fprintf('V83 earlier replan accepted%d decision%s seconds%.3f\n',report.accepted,report.reference_decision,report.seconds);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
