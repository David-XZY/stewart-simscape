function [F,next,info]=stepAbfdController(time,x,state,runtime)
refs=runtime.bundle.refs; c=runtime.config; p=runtime.provider;
time=min(max(time,refs.t(1)),refs.t(end));
ref=struct('q',interp1(refs.t,refs.q.',time).','qd',interp1(refs.t,refs.qd.',time).', ...
    'qdd',interp1(refs.t,refs.qdd.',time).');
idx=find(refs.t<=time+1e-12,1,'last'); next=state;
if ~isempty(runtime.design)
    idx=find(runtime.design.schedule.t<=time+1e-12,1,'last');
end
if c.governorEnabled
    previous=struct('perturbation',state.governorPosition,'rate',state.governorRate);
    [command,governor]=stepAttitudeCommandGovernor(c.attitudePerturbation,previous,runtime.governor);
    ref.q(4:6)=ref.q(4:6)+command.perturbation; ref.qd(4:6)=ref.qd(4:6)+command.rate;
    ref.qdd(4:6)=ref.qdd(4:6)+command.acceleration;
    next.governorPosition=governor.perturbation; next.governorRate=governor.rate;
else
    ref.q(4:6)=ref.q(4:6)+c.attitudePerturbation;
end
nominal=p.inverse(ref.q,ref.qd,ref.qdd);
if any(strcmp(c.controllerMode,{'baseline-lqi','scheduled-lqi'}))
    gain=runtime.lqi.gains(:,:,min(idx,size(runtime.lqi.gains,3)));
    correction=-gain*[x-[ref.q;ref.qd];state.integral];
    next.integral=state.integral+c.sampleTime*(x(1:6)-ref.q-runtime.lqi.leak*state.integral);
    if c.antiWindupEnabled
        Ki=gain(:,13:18); assert(rcond(Ki)>1e-10,'ABFD:IntegralGain','Singular anti-windup map.');
        mismatch=min(max(state.appliedForce-state.previousForce,-30),30);
        next.integral=next.integral-.1*c.sampleTime/.02*(Ki\mismatch);
    end
    F=nominal+correction; K=gain(:,1:12);
elseif strcmp(c.controllerMode,'fullstate-tvlqr')
    K=runtime.design.schedule.K(:,:,min(idx,size(runtime.design.schedule.K,3)));
    F=nominal-K*(x-[ref.q;ref.qd]);
else
    K=runtime.design.schedule.K(:,:,min(idx,size(runtime.design.schedule.K,3)));
    F=computeGeometricInverseDynamicsForce(x,ref,K,p,c.dVariant);
end
o=p.evaluate(x(1:6),x(7:12),ref.qdd);
compensation=zeros(6,1); wrenchEstimate=zeros(6,1);
if c.dobEnabled
    measuredAcceleration=(x(7:12)-state.previousVelocity)/c.sampleTime;
    measured=p.evaluate(x(1:6),x(7:12),measuredAcceleration);
    residual=measured.W-measured.Jv.'*state.appliedForce;
    alpha=exp(-2*pi*5*c.sampleTime); wrenchEstimate=alpha*state.wrenchEstimate+(1-alpha)*residual;
    desired=-(measured.Jv.'\wrenchEstimate); desired=min(max(desired,-120),120);
    compensation=state.compensation+min(max(desired-state.compensation,-2000*c.sampleTime),2000*c.sampleTime);
    next.wrenchEstimate=wrenchEstimate; next.compensation=compensation;
end
if c.qpEnabled
    cfg=runtime.strict; cfg.time=time;
    [F,diagnostic]=stepStrictClfCbfQp(x(1:6),x(7:12),ref.q,ref.qd,ref.qdd, ...
        F,state.previousForce,p.model,runtime.bundle.scene,cfg,compensation,wrenchEstimate);
    assert(diagnostic.feasible,'ABFD:QpInfeasible','Strict QP did not produce a feasible command.');
else
    F=F+compensation;
end
next.previousForce=F; next.previousVelocity=x(7:12);
info=struct('feedforward',nominal,'feedback',F-nominal,'Jv',o.Jv,'Gamma',o.Gamma,'gain',K, ...
    'margin',min(F-p.model.actuator.forceMin,p.model.actuator.forceMax-F));
% Unconstrained command is logged. Actual physical actuator limits remain in the plant.
assert(all(isfinite(F)),'ABFD:ControllerNonfinite','Invalid control command.');
end
