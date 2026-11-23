function audit=ch5CheckRollout(nodes,values,trace,U,o,r)
% Full internal integration mesh check, independent of the shooting grid.
predictedAccelerationMax=ch5PredictionAccelerationLimit(r);
lower=[r.physical.lmin;-r.physical.actuator.ldotMax;-predictedAccelerationMax;r.strict.sigmaSafe^2;r.strict.forceMin];
upper=[r.physical.lmax;r.physical.actuator.ldotMax;predictedAccelerationMax;inf;r.strict.forceMax];
violation=max([lower-values;values-upper;zeros(1,size(values,2))],[],'all');
minimumGap=ch5NativeCollisionAudit(trace,o,r);
collisionViolation=max(0,r.c.safe_distance+r.c.sample_hold_margin-minimumGap);
audit=struct('maximum_violation',max(violation,collisionViolation),'mechanical_violation',violation, ...
    'collision_violation',collisionViolation,'minimum_gap',minimumGap,'internal_samples',size(trace,2), ...
    'finite',all(isfinite(nodes),'all')&&all(isfinite(values),'all')&&all(isfinite(trace),'all'));
audit.accepted=audit.finite&&audit.maximum_violation<=1e-6;
end
