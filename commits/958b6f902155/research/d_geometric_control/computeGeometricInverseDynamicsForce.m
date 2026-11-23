function [F,info]=computeGeometricInverseDynamicsForce(x,ref,K,provider,variant)
if nargin<5, variant='tvlqr'; end
q=x(1:6); v=x(7:12);
[R,E]=abfdRotationExpressions(q(4:6),v(4:6));
[Rr,Er,Erd]=abfdRotationExpressions(ref.q(4:6),ref.qd(4:6));
if strcmp(variant,'d0')
    eta=[q-ref.q;v-ref.qd]; accel=ref.qdd-K*eta;
else
    eta=[q(1:3)-ref.q(1:3);rotationErrorSO3(R,Rr); ...
        v(1:3)-ref.qd(1:3);E*v(4:6)-Er*ref.qd(4:6)];
    desired=[ref.qdd(1:3);Er*ref.qdd(4:6)+Erd*ref.qd(4:6)]-K*eta;
    accel=[desired(1:3);spatialAccelToRpyAccel(q(4:6),v(4:6),desired(4:6))];
end
F=provider.inverse(q,v,accel);
o=provider.evaluate(q,v,accel);
nominal=provider.inverse(ref.q,ref.qd,ref.qdd);
info=struct('eta',eta,'commandAcceleration',accel,'Jv',o.Jv,'Gamma',o.Gamma, ...
    'feedbackForce',F-nominal,'forceMargin',min(F-provider.model.actuator.forceMin,provider.model.actuator.forceMax-F));
end
