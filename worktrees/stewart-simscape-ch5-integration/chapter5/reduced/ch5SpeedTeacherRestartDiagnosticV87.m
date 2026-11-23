function report=ch5SpeedTeacherRestartDiagnosticV87()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'restart_diagnostic_v87.json')),'ch5:ExistingAttempt','Preserve failed restart diagnosis.');
d=load(fullfile(fileparts(directory),'teacher_pilot_v82','train_016_nonideal_force.mat'),'records');
[~,index]=min(abs(cellfun(@(z)z.time,d.records)-13.78));z=d.records{index};r=ch5ReducedPrepareV48(4,'nonideal-force');
[drift,G]=r.ad.dynamics(z.q,z.qd);seed=full(G)\(-full(drift));
[u,repair]=ch5TeacherFeasibleProjectV86(z.x,z.previous_u,z.nominal,z.observed,r,z.diagnostic.clf,z.diagnostic.cbf,seed);
[check,~]=ch5ReducedHoldAuditV22(z.x,z.previous_u,u,z.observed,r);
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'seed',seed,'u',u,'check',check,'repair',repair);
for i=1:numel(repair.attempts)
    a=repair.attempts{i};fprintf('V87 restart radius%.1f qp%d exit%d hard%.3g hold%d mech%.3g\n',a.radius_N,a.solve.feasible,a.solve.exitflag,a.solve.hardConstraintViolation,a.check.accepted,a.check.internal_check.mechanical_violation);
end
writeJSON(fullfile(directory,'restart_diagnostic_v87.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
