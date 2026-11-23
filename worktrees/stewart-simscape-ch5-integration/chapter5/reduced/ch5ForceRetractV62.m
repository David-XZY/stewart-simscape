function [U,check,traj,cost,info]=ch5ForceRetractV62(x,previous,U,goal,o,r,k,qs)
% A nonlinear retraction of the input sequence onto the feasible steady manifold.
% This is an optimization initializer, not a pose reference or a safety certificate.
[check,traj,cost]=ch5ForceAuditV62(x,previous,U,goal,o,r,k,qs);info=struct('attempted',false,'repair',[],'maximum_force_change_N',0);
if check.finite&&check.inequality_violation<=1e-6&&~check.accepted
    raw=U;[trial,repair]=ch5ReducedTerminalV23(x,U,r,k,traj.X);info.attempted=true;info.repair=repair;
    [candidateCheck,candidateTraj,candidateCost]=ch5ForceAuditV62(x,previous,trial,goal,o,r,k,[]);
    info.after= candidateCheck;info.maximum_force_change_N=max(abs(trial-raw),[],'all');
    if candidateCheck.accepted,U=trial;check=candidateCheck;traj=candidateTraj;cost=candidateCost;end
end
end
