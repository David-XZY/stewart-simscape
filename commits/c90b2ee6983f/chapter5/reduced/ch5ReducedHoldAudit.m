function [check,interval]=ch5ReducedHoldAudit(x,previous,u,o,r)
% Certifies only the sampled executed interval, never the unexecuted horizon.
kernel=ch5ReducedHoldKernel(r);[next,trace,values]=kernel(x,u);
next=full(next);trace=full(trace);values=full(values);
finite=all(isfinite([x;previous;u;next;trace(:);values(:)]));
lower=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
upper=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
mechanical=max([lower-values;values-upper;zeros(1,size(values,2))],[],'all');
holdRuntime=r;holdRuntime.c.H=1;
gap=ch5NativeCollisionAudit(trace,o,holdRuntime);
collision=max(0,r.c.safe_distance+r.c.sample_hold_margin-gap);
extra=ch5ReducedTransient(x,u,o,r);
forceViolation=max([r.strict.forceMin-u;u-r.strict.forceMax;0]);
rateViolation=max([abs(u-previous)/r.c.Ts-r.strict.forceRateLimit;0]);
violation=max([mechanical,collision,extra.maximum_violation,forceViolation]);
accepted=finite&&extra.accepted&&violation<=1e-6&&rateViolation<=1e-6;
internal=struct('accepted',finite&&max(mechanical,collision)<=1e-6,'finite',finite, ...
    'mechanical_violation',mechanical,'collision_violation',collision,'minimum_gap',gap, ...
    'maximum_violation',max(mechanical,collision),'internal_samples',size(trace,2));
check=struct('accepted',accepted,'finite',finite,'equality_tolerance_ratio',0, ...
    'terminal_checked',false,'inequality_violation',violation,'command_rate_violation',rateViolation, ...
    'internal_check',internal,'transient',extra,'scope','executed_interval','duration_seconds',r.c.Ts);
interval=struct('X',[x,next],'U',u,'delta',u-previous,'scope','executed_interval');
end
