function design=ch5DiscreteLqi(physical,ad,c)
% Same LQI quadratic weights and integral leak, designed on the actual
% 50 Hz grid. The common interface already measures pose AND velocity.
import casadi.*
x=SX.sym('lqi_state',12);u=SX.sym('lqi_force',6);
[d,G]=ad.dynamics(x(1:6),x(7:12));rhs=[x(7:12);d+G*u];
linear=Function('ch5_lqi_linear',{x,u},{jacobian(rhs,x),jacobian(rhs,u)});
home=physical.qHome;[d0,G0]=ad.dynamics(home,zeros(6,1));equilibrium=full(G0)\(-full(d0));
[A,B]=linear([home;zeros(6,1)],equilibrium);A=full(A);B=full(B);
mapping=sgpJacobian(home,physical).Jv'\eye(6);B=B*mapping;C=[eye(6),zeros(6)];
pd=c2d(ss(A,B,-C,zeros(6)),c.Ts,'zoh');
poseScale=[.03*ones(3,1);deg2rad(1)*ones(3,1)];intScale=[.2*ones(3,1);.4*ones(3,1)];
Q=blkdiag(C'*diag(1./poseScale.^2)*C+1e-9*eye(12),diag(1./intScale.^2));R=eye(6)/c.lqi_control_scale^2;
gain=lqi(pd,Q,R);Kx=gain(:,1:12);Ki=gain(:,13:18);leak=exp(-2*pi*2*c.Ts);
closed=[pd.A-pd.B*Kx,-pd.B*Ki;c.Ts*C,leak*eye(6)];poles=eig(closed);
assert(max(abs(poles))<1,'ch5:LqiDesign','Discrete LQI equilibrium is unstable.');
design=struct('Kx',Kx,'Ki',Ki,'mapping',mapping,'leak',leak,'antiwindup',-pinv(Ki)/.02, ...
    'closed_loop_poles',poles,'equilibrium',equilibrium,'control_scale',c.lqi_control_scale, ...
    'definition','LQI on measured [pose; generalized velocity], designed on the registered grid');
end
