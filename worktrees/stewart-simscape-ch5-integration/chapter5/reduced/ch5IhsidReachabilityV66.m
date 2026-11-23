function result=ch5IhsidReachabilityV66(q0,v0,goal,T,r)
% Necessary per-leg reachability bounds for final leg velocity zero.
% For bounded acceleration and speed, maximize displacement by accelerate,
% optionally coast at vmax, then decelerate to zero. No obstacle assumption.
[~,~,~,~,~,~,~,L0,velocity]=r.ad.dynamics(q0,v0);
[~,~,~,~,~,~,~,L1]=r.ad.dynamics(goal,zeros(6,1));
L0=full(L0);L1=full(L1);velocity=full(velocity);a=r.physical.actuator.lddotMax;vmax=r.physical.actuator.ldotMax;
upper=maximumDistance(velocity,T,a,vmax);lower=-maximumDistance(-velocity,T,a,vmax);
delta=L1-L0;positionWitness=max([lower-delta;delta-upper;0]);
velocityWitness=max([abs(velocity)-a*T;abs(velocity)-vmax;0]);
initialLengthWitness=max([r.physical.lmin-L0;L0-r.physical.lmax;0]);
result=struct('infeasible',positionWitness>.001||velocityWitness>.001||initialLengthWitness>.001, ...
    'duration',T,'delta_length',delta,'initial_leg_velocity',velocity,'minimum_displacement',lower,'maximum_displacement',upper, ...
    'position_witness_m',positionWitness,'velocity_witness_mps',velocityWitness,'initial_length_witness_m',initialLengthWitness, ...
    'witness_tolerance',.001,'scope','necessary_continuous_leg_speed_acceleration_bound');
end
function value=maximumDistance(v,T,a,vmax)
value=a*T^2/4+v*T/2-v.^2./(4*a);peak=(a*T+v)/2;
limited=peak>vmax;
plateau=vmax*T-(vmax-v).^2./(2*a)-vmax.^2./(2*a);
value(limited)=plateau(limited);
end
