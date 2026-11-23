function [next,force,rampTime]=advanceAbfdActuator(state,target,h,profile,model)
% Exact constant-target evolution of the existing slew-before-lag actuator.
% Transport delay/dead zone belong to the caller; saturation is physical only.
rate=profile.forceRate; tau=profile.forceLag;
validateattributes(h,{'double'},{'scalar','nonnegative','finite'});
validateattributes(rate,{'double'},{'scalar','positive','finite'});
validateattributes(tau,{'double'},{'scalar','positive','finite'});
difference=target-state.slew; slope=sign(difference)*rate;
rampTime=abs(difference)/rate; active=min(h,rampTime);
rampForce=state.slew+slope.*(active-tau)+ ...
    (state.lag-state.slew+slope*tau).*exp(-active/tau);
next=struct('slew',state.slew+slope.*active, ...
    'lag',target+(rampForce-target).*exp(-(h-active)/tau));
limit=max(abs([model.actuator.forceMin(:),model.actuator.forceMax(:)]),[],2);
force=min(max(next.lag,-limit),limit);
end
