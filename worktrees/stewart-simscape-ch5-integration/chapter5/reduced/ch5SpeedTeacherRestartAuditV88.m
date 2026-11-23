function report=ch5SpeedTeacherRestartAuditV88()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'restart_audit_v88.json')),'ch5:ExistingAttempt','Preserve restart checks.');
d=load(fullfile(fileparts(directory),'teacher_pilot_v82','train_016_nonideal_force.mat'),'records');
[~,index]=min(abs(cellfun(@(z)z.time,d.records)-13.78));z=d.records{index};r=ch5ReducedPrepareV48(4,'nonideal-force');
[drift,G]=r.ad.dynamics(z.q,z.qd);seed=full(G)\(-full(drift));
timer=tic;[u,repair]=ch5TeacherFeasibleProjectV88(z.x,z.previous_u,z.nominal,z.observed,r,z.diagnostic.clf,z.diagnostic.cbf,seed);seconds=toc(timer);
[check,~]=ch5ReducedHoldAuditV22(z.x,z.previous_u,u,z.observed,r);
hard=max([z.diagnostic.cbf.A*u-z.diagnostic.cbf.b;0]);
assert(repair.feasible&&check.accepted&&hard<=10*r.strict.solver.constraintTolerance,'ch5:RestartGate','Restart must pass every original physical and CBF constraint.');
accepted=repair.attempts{end}.solve;
assert(accepted.decisionVariableCount==7&&~accepted.hasCbfSlack,'ch5:RestartGate','Do not relax the original CBF rows.');
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'source_task','train_016','mode','nonideal-force', ...
    'time',z.time,'seconds',seconds,'command',u,'seed',seed,'repair',repair,'check',check,'hard_cbf_violation',hard, ...
    'original_failed',~z.diagnostic.feasible,'all_original_hard_constraints_preserved',true);
writeJSON(fullfile(directory,'restart_audit_v88.json'),report);
fprintf('V88 restart physical%d CBFviol%.3g trials%d seconds%.3f\n',check.accepted,hard,numel(repair.attempts),seconds);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
