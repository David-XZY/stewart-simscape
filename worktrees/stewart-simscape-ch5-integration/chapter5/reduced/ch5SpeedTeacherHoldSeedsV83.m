function report=ch5SpeedTeacherHoldSeedsV83()
% Test genuinely different actuator commands at the preserved failure state.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'hold_seeds.json')),'ch5:ExistingAttempt','Preserve seed feasibility evidence.');
d=load(fullfile(fileparts(directory),'teacher_pilot_v82','train_016_nonideal_force.mat'),'records');
[~,index]=min(abs(cellfun(@(z)z.time,d.records)-13.78));z=d.records{index};r=ch5ReducedPrepareV48(4,'nonideal-force');
[drift,G]=r.ad.dynamics(z.q,z.qd);equilibrium=full(G)\(-full(drift));
cbf=z.diagnostic.cbf;clf=z.diagnostic.clf;commands=[z.previous_u,z.u,equilibrium,z.nominal];
labels={'previous','failed_output','mechanical_equilibrium','nominal'};
for alpha=[.25,.5,.75]
    commands(:,end+1)=z.previous_u+alpha*(equilibrium-z.previous_u);labels{end+1}=sprintf('previous_to_equilibrium_%.2f',alpha); %#ok<AGROW>
end
report=struct('completed',false,'formal',false,'pid',feature('getpid'),'time',z.time,'rows',{{}},'nominal_change',norm(z.nominal-z.previous_u,inf));
for i=1:size(commands,2)
    [check,~]=ch5ReducedHoldAuditV22(z.x,z.previous_u,commands(:,i),z.observed,r);
    hard=max([cbf.A*commands(:,i)-cbf.b;0]);
    [proposal,solve]=ch5TeacherIntervalProjectV62(z.x,z.previous_u,z.nominal,z.observed,r,clf,cbf,commands(:,i));
    row=struct('seed',labels{i},'seed_check',check,'seed_hard_cbf_violation',hard,'projected_feasible',solve.feasible, ...
        'projection',solve,'command',proposal,'seed_command',commands(:,i));
    report.rows{end+1}=row;
    fprintf('V83 hold seed %s physical%d CBFviol%.3g projected%d\n',labels{i},check.accepted,hard,solve.feasible);
end
report.completed=true;writeJSON(fullfile(directory,'hold_seeds.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
