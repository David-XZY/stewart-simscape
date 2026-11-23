function [check,interval]=ch5TeacherHoldAuditV91(x,previous,u,o,r)
% Known delayed states obey the physical bound; future states keep the reserve.
% Only the 1% acceleration design reserve is inapplicable before actuation.
kernel=ch5ReducedHoldKernel(r);[next,trace,values]=kernel(x,u);
next=full(next);trace=full(trace);values=full(values);
finite=all(isfinite([x;previous;u;next;trace(:);values(:)]));
lower=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
upper=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
lo=repmat(lower,1,size(values,2));hi=repmat(upper,1,size(values,2));
known=false(size(r.plant.traceTime));
if r.mode=="nonideal-force",known=r.plant.traceTime<=r.plant.profile.delay+1e-14;end
lo(13:18,known)=repmat(-r.physical.actuator.lddotMax,1,sum(known));
hi(13:18,known)=repmat(r.physical.actuator.lddotMax,1,sum(known));
mechanical=max([lo-values;values-hi;zeros(1,size(values,2))],[],'all');
holdRuntime=r;holdRuntime.c.H=1;gap=ch5ReducedCollisionAuditV22(trace,o,holdRuntime);
collision=max(0,r.c.safe_distance+r.c.sample_hold_margin-gap);
extra=ch5ReducedTransient(x,u,o,r);
forceViolation=max([r.strict.forceMin-u;u-r.strict.forceMax;0]);
rateViolation=max([abs(u-previous)/r.c.Ts-r.strict.forceRateLimit;0]);
violation=max([mechanical,collision,extra.maximum_violation,forceViolation]);
accepted=finite&&extra.accepted&&violation<=1e-6&&rateViolation<=1e-6;
internal=struct('accepted',finite&&max(mechanical,collision)<=1e-6,'finite',finite, ...
    'mechanical_violation',mechanical,'collision_violation',collision,'minimum_gap',gap, ...
    'maximum_violation',max(mechanical,collision),'internal_samples',size(trace,2), ...
    'known_physical_acceleration_limit',r.physical.actuator.lddotMax, ...
    'future_reserved_acceleration_limit',ch5PredictionAccelerationLimit(r),'known_samples',sum(known));
check=struct('accepted',accepted,'finite',finite,'equality_tolerance_ratio',0,'terminal_checked',false, ...
    'inequality_violation',violation,'command_rate_violation',rateViolation,'internal_check',internal, ...
    'transient',extra,'scope','executed_interval_causal_reserve','duration_seconds',r.c.Ts);
interval=struct('X',[x,next],'U',u,'delta',u-previous,'scope','executed_interval');
end
