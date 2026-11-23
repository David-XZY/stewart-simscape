function p=dpPhysics(m)
% Independent symbolic analytic rigid-body model; forces are NEVER clipped.
import casadi.*
q=SX.sym('q',6); v=SX.sym('v',6); a=SX.sym('a',6);
r=q(4); b=q(5); y=q(6);
R=[cos(y)*cos(b),cos(y)*sin(b)*sin(r)-sin(y)*cos(r),cos(y)*sin(b)*cos(r)+sin(y)*sin(r); ...
sin(y)*cos(b),sin(y)*sin(b)*sin(r)+cos(y)*cos(r),sin(y)*sin(b)*cos(r)-cos(y)*sin(r); ...
-sin(b),cos(b)*sin(r),cos(b)*cos(r)];
E=[cos(y)*cos(b),-sin(y),0;sin(y)*cos(b),cos(y),0;-sin(b),0,1];
omega=E*v(4:6); alpha=jacobian(omega,q)*v+E*a(4:6);
rc=R*m.dynamics.comP; I=R*m.dynamics.inertiaAtCOM_P*R.';
force=m.dynamics.totalMass*(a(1:3)+cross(alpha,rc)+cross(omega,cross(omega,rc))-m.g);
W=[force;I*alpha+cross(omega,I*omega)+cross(rc,force)];
L=SX.zeros(6,1); Jv=SX.zeros(6,6);
for k=1:6
    rb=R*m.B(:,m.legMap(k)); d=q(1:3)+rb-m.A(:,k); L(k)=sqrt(d.'*d); unit=d/L(k);
    Jv(k,:)=[unit.',cross(rb,unit).'];
end
Jq=jacobian(L,q); Ld=Jq*v; Ldd=Jq*a+jacobian(Ld,q)*v;
F=solve(Jv.',W); Jbar=Jv*diag([1,1,1,2,2,2]); G=Jbar*Jbar.';
phi=.01*trace(solve(G+2.5e-5*SX.eye(6),SX.eye(6)));
p.point=Function('dp_point',{q,v,a},{L,Ld,Ldd,F,Jv,W,phi});
p.pose=Function('dp_pose',{q},{R});
p.model=m;
end
