function [check,trajectory,cost]=ch5ReducedAuditFast(x,previous,U,goal,o,r,k,prior)
[check,trajectory,cost]=ch5ReducedAudit(x,previous,U,goal,o,r,k,prior);
if check.accepted
    extra=ch5ReducedTransient(trajectory.X,U,o,r);check.transient=extra;
    check.accepted=extra.accepted;check.inequality_violation=max(check.inequality_violation,extra.maximum_violation);
end
% Input-rate tolerance is expressed in N/s, matching actual-run evaluation.
rateViolation=max([reshape(abs(trajectory.delta)/r.c.Ts-r.strict.forceRateLimit,[],1);0]);
check.command_rate_violation=rateViolation;check.accepted=check.accepted&&rateViolation<=1e-6;
end
