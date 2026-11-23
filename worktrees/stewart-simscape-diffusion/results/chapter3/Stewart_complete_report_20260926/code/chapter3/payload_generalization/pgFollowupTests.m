function report=pgFollowupTests()
c=pgConfig('payload_followup_audit_v2'); root=fileparts(c.output); checks={};
old=jsondecode(fileread(fullfile(root,'payload_integration_v1','penalty_oracle.json')));
[v,g,~,first]=pgPenalty(old.free,old.task,c);
ve=abs(v-old.value); ge=max(abs(g-old.gradient(:)));
assert(ve<1e-9&&ge<1e-7,'PG:PenaltyParity','Parameterized graph changed the reference penalty.'); checks{end+1}='parameterized_penalty_matches_prior_oracle';
changed=old.task; changed.T=changed.T*.8; changed.qGoal(1)=changed.qGoal(1)+.01;
changed.scene.hood.obstacles(1).center_S(2)=changed.scene.hood.obstacles(1).center_S(2)+.01;
changed=pgFinalizeTask(changed,pgMakePayload('box',[1.1,.26,.22],400),c);
[v2,g2,groups,second]=pgPenalty(old.free,changed,c); assert(second.hit&&strcmp(first.key,second.key));
assert(abs(v2-v)>1e-8,'PG:Cache','Changed task was not applied.'); checks{end+1}='task_parameters_change_values_reuse_graph';
dpWriteJson(fullfile(c.output,'penalty_oracle.json'),struct('free',old.free,'task',changed,'envelope',c.envelope,'value',v2,'gradient',g2,'groups',groups));
f=dir(fullfile(root,'payload_pilot_v1','attempt_*.mat')); found=false;
for k=1:numel(f)
    x=load(fullfile(f(k).folder,f(k).name)); if x.row.passed, found=true; break; end
end
assert(found); solver=x.sol;
[ok,r]=pgAccept(solver.bundle,x.task,'physical',tic,inf,c,solver); assert(ok,r.reason); checks{end+1}='current_solver_evidence_preserved';
bad=solver; bad.solverSuccess=false; [ok,r]=pgAccept(bad.bundle,x.task,'physical',tic,inf,c,bad); assert(~ok&&strcmp(r.reason,'solver_gate')); checks{end+1}='failed_solver_state_rejected';
bad=solver; bad.maxBounds=2e-6; [ok,r]=pgAccept(bad.bundle,x.task,'physical',tic,inf,c,bad); assert(~ok&&strcmp(r.reason,'solver_gate')); checks{end+1}='variable_bound_residual_rejected';
[ok,r]=pgAccept(solver.bundle,x.task,'physical',tic,0,c,solver); assert(~ok); checks{end+1}='late_solver_acceptance_rejected';
report=struct('passed',true,'checks',{checks},'valueError',ve,'gradientError',ge,'codeHash',c.codeHash);
dpWriteJson(fullfile(c.output,'tests.json'),report); fprintf('PG_FOLLOWUP_TESTS_PASSED %d\n',numel(checks));
end
