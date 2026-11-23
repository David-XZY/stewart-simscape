function report=ch5SpeedTeacherExactProbeV84()
% Same saved state, observed motion, duration, four starts and hard constraints.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'exact_probe_v84.json')),'ch5:ExistingAttempt','Preserve solver comparison.');
d=load(fullfile(directory,'replan_probe.mat'),'report','z');r=ch5ReducedPrepareV48(4,'ideal-force');
duration=d.report.arrival_time-d.report.time;
timer=tic;plan=ch5SpeedIhsidV84(d.z.q,d.z.qd,d.z.goal,d.z.observed,r,duration,d.report.time);
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'time',d.report.time,'duration',duration, ...
    'seconds',toc(timer),'accepted',plan.accepted,'limited_memory_task_solve_seconds',d.report.entry.plan.seconds, ...
    'limited_memory_task_solve_accepted',d.report.entry.plan.accepted,'limited_memory_backup_accepted',d.report.accepted, ...
    'plan',plan,'solver_change','exact sparse Hessian only');
save(fullfile(directory,'exact_probe_v84.mat'),'report','r','-v7');writeJSON(fullfile(directory,'exact_probe_v84.json'),report);
fprintf('V84 exact Hessian accepted%d seconds%.3f\n',report.accepted,report.seconds);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
