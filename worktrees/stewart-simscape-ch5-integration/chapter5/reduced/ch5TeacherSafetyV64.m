function [u,d]=ch5TeacherSafetyV64(x,qr,vr,ar,nominal,previous,o,r,time)
% Same IHSID/LQI/CLF-CBF-QP chain, with actuator-aware sampled hold rows.
timer=tic;[guess,base]=ch5SafetyQp(x(1:6),x(7:12),qr,vr,ar,nominal,previous,o,r,time);
[u,repair]=ch5TeacherIntervalProjectV62(x,previous,nominal,o,r,base.clf,base.cbf,guess);
d=base;d.feasible=repair.feasible;d.usedFallback=~repair.feasible;d.interval_projection=repair;d.original_qp_feasible=base.feasible;
d.fallbackSafetyCertified=false;d.solveTime=toc(timer);d.cbfResidual=base.cbf.b-base.cbf.A*u;d.minimumCbfResidual=min(d.cbfResidual);
d.clfSlack=max(base.clf.A*u-base.clf.b,0);d.clfResidual=base.clf.b+d.clfSlack-base.clf.A*u;
d.hardConstraintViolation=max([-d.cbfResidual;d.forceLower-u;u-d.forceUpper;0]);
if d.feasible,d.status='sampled_actuator_hold_checked';d.exitflag=1;else,d.status='sampled_actuator_hold_failed';d.exitflag=-1;end
end
