function [u,d]=ch5TeacherSafetyV88(x,qr,vr,ar,nominal,previous,o,r,time)
% Preserve the original filter; restart only a failed local hold projection.
timer=tic;[u,d]=ch5TeacherSafetyV64(x,qr,vr,ar,nominal,previous,o,r,time);
d.feasible_seed_restart=false;d.restart_attempts={};
if d.feasible,return;end
[drift,G]=r.ad.dynamics(x(1:6),x(7:12));equilibrium=full(G)\(-full(drift));
lower=max(r.strict.forceMin,previous-r.strict.forceRateLimit*r.c.Ts);
upper=min(r.strict.forceMax,previous+r.strict.forceRateLimit*r.c.Ts);
best=inf;bestRepair=[];original=u;
for alpha=[1,.5,.25]
    seed=min(max(previous+alpha*(equilibrium-previous),lower),upper);
    [check,~]=ch5ReducedHoldAuditV22(x,previous,seed,o,r);
    hard=max([d.cbf.A*seed-d.cbf.b;0]);
    if ~check.accepted||hard>10*r.strict.solver.constraintTolerance
        d.restart_attempts{end+1}=struct('alpha',alpha,'seed_accepted',false,'check',check,'hard_cbf_violation',hard);continue;
    end
    [proposal,repair]=ch5TeacherFeasibleProjectV88(x,previous,nominal,o,r,d.clf,d.cbf,seed);
    d.restart_attempts{end+1}=struct('alpha',alpha,'seed_accepted',true,'projection',repair);
    if repair.feasible
        slack=max(d.clf.A*proposal-d.clf.b,0);
        cost=.5*sum(r.strict.weights.force.*(proposal-nominal).^2)+ ...
            .5*sum(r.strict.weights.forceRate.*(proposal-previous).^2)+.5*r.strict.weights.clfSlack*slack^2;
        if cost<best,best=cost;u=proposal;bestRepair=repair;end
    end
end
if ~isempty(bestRepair)
    d.original_failed_command=original;d.feasible=true;d.usedFallback=false;d.exitflag=1;
    d.feasible_seed_restart=true;d.interval_projection=bestRepair;d.status='feasible_seed_qp_hold_checked';
    d.cbfResidual=d.cbf.b-d.cbf.A*u;d.minimumCbfResidual=min(d.cbfResidual);
    d.clfSlack=max(d.clf.A*u-d.clf.b,0);d.clfResidual=d.clf.b+d.clfSlack-d.clf.A*u;
    d.hardConstraintViolation=max([-d.cbfResidual;lower-u;u-upper;0]);
end
d.solveTime=toc(timer);d.fallbackSafetyCertified=false;
end
