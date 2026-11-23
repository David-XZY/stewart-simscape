function fun=buildAbfdUncertainKinematicsAD(provider,standardizedMean)
% Physical L/Ldot/Lddot/F and their full state/colored-wrench derivatives.
import casadi.*
ad=buildCasadiCompositeDynamics(provider); z=SX.sym('physical_state',24); r=SX.sym('colored_wrench',6);
x=z(1:12); limit=max(abs([provider.model.actuator.forceMin(:),provider.model.actuator.forceMax(:)]),[],2);
force=fmin(fmax(z(19:24),-limit),limit);
[H,~,~,Jq,L,Sigma]=ad.terms(x(1:6),x(7:12)); velocity=Jq*x(7:12);
xd=ad.f(x,force)-[SX.zeros(6,1);solve(H,diag(sqrt(diag(Sigma)))*(r+standardizedMean))];
acceleration=jacobian(velocity,x)*xd; values=[L;velocity;acceleration;force];
fun=Function('abfd_uncertain_kinematics',{z,r},{values,jacobian(values,z),jacobian(values,r)});
end
