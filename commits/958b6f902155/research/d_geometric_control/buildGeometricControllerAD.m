function ctl=buildGeometricControllerAD(provider,variant)
% AD of the full unsaturated policy, including current-state inverse dynamics.
import casadi.*
ad=buildCasadiCompositeDynamics(provider); x=SX.sym('x',12); r=SX.sym('r',18); K=SX.sym('K',6,12);
q=x(1:6); v=x(7:12); qr=r(1:6); vr=r(7:12); ar=r(13:18);
[R,E,Ed]=abfdRotationExpressions(q(4:6),v(4:6));
[Rr,Er,Erd]=abfdRotationExpressions(qr(4:6),vr(4:6));
if strcmp(variant,'d0')
    eta=x-r(1:12); a=ar-K*eta;
else
    S=R*Rr.'; c=(trace(S)-1)/2;
    safe=fmin(fmax(c,-1+1e-10),1-1e-10);
    theta=acos(safe);
    scale=if_else(c>1-1e-6,0.5+(1-c)/6,theta/(2*sin(theta)));
    e=scale*[S(3,2)-S(2,3);S(1,3)-S(3,1);S(2,1)-S(1,2)];
    eta=[q(1:3)-qr(1:3);e;v(1:3)-vr(1:3);E*v(4:6)-Er*vr(4:6)];
    u=[ar(1:3);Er*ar(4:6)+Erd*vr(4:6)]-K*eta;
    a=[u(1:3);solve(E,u(4:6)-Ed*v(4:6))];
end
F=ad.inverse(q,v,a);
ctl.force=Function('abfd_policy',{x,r,K},{F});
ctl.jacobian=Function('abfd_policy_jacobian',{x,r,K},{jacobian(F,x)});
ctl.errorJacobian=Function('abfd_error_jacobian',{x,r},{jacobian(eta,x)});
ctl.variant=variant;
% Numeric runtime uses the robust SO(3) implementation; AD is rejected near pi.
end
