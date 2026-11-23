function [u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor)
poseError=[anchor(1:3)-x(1:3);atan2(sin(anchor(4:6)-x(4:6)),cos(anchor(4:6)-x(4:6)))];desired=16*poseError-8*x(7:12);
[drift,G,~,~,~,Jq,~,~,~,geometric]=r.ad.dynamics(x(1:6),x(7:12));G=full(G);drift=full(drift);Jq=full(Jq);geometric=full(geometric);
limit=.90*r.physical.actuator.lddotMax;legDesired=min(max(Jq*desired+geometric,-limit),limit);
nominal=G\(Jq\(legDesired-geometric)-drift);
[qp,base]=ch5SafetyQp(x(1:6),x(7:12),anchor,zeros(6,1),zeros(6,1),nominal,previous,o,r,time);
best=qp;bestViolation=inf;bestValid=false;attempts={};
for fraction=[1,.5,.25,0]
    candidate=previous+fraction*(qp-previous);[~,trace]=r.plant.fd(x,candidate,zeros(6,1));trace=full(trace);
    values=zeros(25,size(trace,2));gap=inf;
    for j=1:size(trace,2)
        values(:,j)=full(r.plant.mechanicalValues(trace(:,j),candidate));
        ot=o;ot(1:3,:)=ot(1:3,:)+r.plant.traceTime(j)*o(16:18,:);
        for oi=find(o(22,:)>.5),gg=full(k.collision(trace(1:6,j),ot(:,oi)));gap=min(gap,gg);end
    end
    % Same physical hard limits. Reserve 0.001 m/s^2 in the recovery hold;
    % the normal horizon acceptance continues to reserve 1 percent.
    holdAcceleration=r.physical.actuator.lddotMax-.001;
    lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-holdAcceleration;r.strict.sigmaSafe^2;r.strict.forceMin];
    hi=[r.physical.lmax;r.physical.actuator.ldotMax;holdAcceleration;inf;r.strict.forceMax];
    violation=max([lo-values;values-hi;zeros(1,size(values,2))],[],'all');
    violation=max(violation,r.c.safe_distance+r.c.sample_hold_margin-gap);
    extra=ch5ReducedTransient(x,candidate,o,r,holdAcceleration);violation=max(violation,extra.maximum_violation);
    cbfViolation=max([base.cbf.A*candidate-base.cbf.b;0]);
    valid=base.feasible&&cbfViolation<=10*r.strict.solver.constraintTolerance&&all(isfinite(values),'all')&&violation<=1e-6;
    attempts{end+1}=struct('fraction',fraction,'hold_violation',violation,'cbf_violation',cbfViolation,'valid',valid,'transient',extra);
    if violation<bestViolation,best=candidate;bestViolation=violation;bestValid=valid;end
    if valid,best=candidate;bestViolation=violation;bestValid=true;break;end
end
u=best;d=base;d.feasible=bestValid;d.usedFallback=true;d.fallbackSafetyCertified=false;
d.interval_valid=bestValid;d.hold_violation=bestViolation;d.recovery_attempts=attempts;d.recovery_anchor=anchor;
d.cbfResidual=d.cbf.b-d.cbf.A*u;d.minimumCbfResidual=min(d.cbfResidual);
d.clfSlack=max(d.clf.A*u-d.clf.b,0);d.clfResidual=d.clf.b+d.clfSlack-d.clf.A*u;
d.hardConstraintViolation=max([-d.cbfResidual;d.forceLower-u;u-d.forceUpper;0]);
d.activeConstraints=d.cbf.names(d.cbfResidual<=r.strict.solver.activeTolerance).';
d.recovery_qp_status=base.status;
if bestValid,d.status="recovery_interval_checked";else,d.status="recovery_interval_invalid";end
end
