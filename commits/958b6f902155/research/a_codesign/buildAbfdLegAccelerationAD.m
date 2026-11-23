function f=buildAbfdLegAccelerationAD(ad)
% Physical leg acceleration with independent state and held-force inputs.
import casadi.*
x=SX.sym('leg_state',12); force=SX.sym('leg_force',6);
[H,~,~,Jq]=ad.terms(x(1:6),x(7:12));
velocity=Jq*x(7:12); velocityJ=jacobian(velocity,x);
acceleration=velocityJ*ad.f(x,force);
f=Function('abfd_leg_acceleration',{x,force}, ...
    {velocity,velocityJ,acceleration,jacobian(acceleration,x), ...
    jacobian(acceleration,force),-Jq*solve(H,SX.eye(6))});
end
