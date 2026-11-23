function [physical,scene,ad,strict] = ch5Physics(c)
physical=buildOptModelCustom();scene=buildCylinderBoxTransferScene(physical);
% Simscape M lies on the top surface of the 20 mm thick platform solid.
mp=physical.dynamics.platformMass;ml=physical.dynamics.objectMass;
cp=[0;0;-.01];cl=scene.objectCylinder.center_P;com=(mp*cp+ml*cl)/(mp+ml);
Ip=diag([.25,.25,.5])*mp*physical.rB^2;
rp=scene.objectCylinder.radius;ll=scene.objectCylinder.length;
Il=diag([.5*ml*rp^2,ml/12*(3*rp^2+ll^2),ml/12*(3*rp^2+ll^2)]);
dp=cp-com;dl=cl-com;
physical.dynamics.comP=com;
physical.dynamics.inertiaAtCOM_P=Ip+Il+mp*((dp'*dp)*eye(3)-dp*dp')+ml*((dl'*dl)*eye(3)-dl*dl');
strict=makeStrictClfCbfQpConfig(physical,struct('dt',c.Ts));
ad=buildStrictClfCbfAdModel(physical,scene,strict);
% Expose small primal maps to callers. Differentiating a map that also
% returns barrier Hessians needlessly generates third/fourth derivatives.
import casadi.*
q=SX.sym('q',6);v=SX.sym('v',6);a=SX.sym('a',6);u=SX.sym('u',6);w=SX.sym('w',6);
[~,~,H,b,Jv,Jq,Jbar,L,Ld,geometric]=ad.dynamics(q,v);
kinematics=Function('ch5_jbar',{q},{Jbar});
qm=MX.sym('sigma_q',6);jb=kinematics(qm);gram=jb*jb';
ad.sigmaValue=Function('ch5_sigma_primal',{qm},{1/trace(solve(gram,MX.eye(6)))});
% Solve Newton-Euler about the COM, then transform acceleration to M.
% This is algebraically H\wrench, without expanding a symbolic 6x6 inverse.
R=ch5Rotation(q(4:6));pitch=q(5);yaw=q(6);cp=cos(pitch);sp=sin(pitch);cy=cos(yaw);sy=sin(yaw);
E=[cy*cp,-sy,0;sy*cp,cy,0;-sp,0,1];
Edot=[-cy*sp*v(5)-sy*cp*v(6),-cy*v(6),0; ...
    -sy*sp*v(5)+cy*cp*v(6),-sy*v(6),0;-cp*v(5),0,0];
Ei=[cy/cp,sy/cp,0;-sy,cy,0;cy*sp/cp,sy*sp/cp,1];
center=R*physical.dynamics.comP;omega=E*v(4:6);Iw=R*physical.dynamics.inertiaAtCOM_P*R';
wrench=Jv'*u+w;force=wrench(1:3);torque=wrench(4:6);
alpha=R*(physical.dynamics.inertiaAtCOM_P\eye(3))*R'*(torque-cross(center,force)-cross(omega,Iw*omega));
acceleration=[force/physical.dynamics.totalMass+physical.g-cross(alpha,center)-cross(omega,cross(omega,center));Ei*(alpha-Edot*v(4:6))];
ad.motion=Function('ch5_motion_primal',{q,v,u,w},{[v;acceleration]});
implicitCore=Function('ch5_implicit_core',{q,v,a,u},{Jv'*u-H*a-b,[L;Ld;geometric+Jq*a;u]});
forceAcceleration=substitute(acceleration,w,SX.zeros(6,1));
mechanicalCore=Function('ch5_mechanical_core',{q,v,u},{[L;Ld;geometric+Jq*forceAcceleration;u]});
qm=MX.sym('q',6);vm=MX.sym('v',6);am=MX.sym('a',6);um=MX.sym('u',6);
[balance,mech]=implicitCore(qm,vm,am,um);sigma=ad.sigmaValue(qm);
ad.implicit=Function('ch5_implicit_primal',{qm,vm,am,um},{balance,[mech(1:18);sigma;mech(19:24)]});
mech=mechanicalCore(qm,vm,um);
ad.mechanical=Function('ch5_mechanical_primal',{qm,vm,um},{[mech(1:18);sigma;mech(19:24)]});
ad.feedforward=Function('ch5_force_feedforward',{q,v,a},{solve(Jv',H*a+b)});
end
