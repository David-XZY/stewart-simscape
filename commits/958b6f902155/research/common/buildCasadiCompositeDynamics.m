function ad=buildCasadiCompositeDynamics(provider)
% Analytic wrench expression; no finite differences or SX-to-double conversion.
import casadi.*
assert(isempty(provider.network) || ~isfield(provider.network,'meanAnchoring') || ...
    (isfield(provider,'parameterCalibration') && isfield(provider,'biasCalibration')), ...
    'ABFD:StartupBalanceRequired','Startup inertia and static balance are required before automatic differentiation.');
m=provider.model; q=SX.sym('q',6); v=SX.sym('v',6); a=SX.sym('a',6); F=SX.sym('F',6);
[R,E,Ed]=abfdRotationExpressions(q(4:6),v(4:6));
w=E*v(4:6); alpha=E*a(4:6)+Ed*v(4:6);
r=R*m.dynamics.comP; I=R*m.dynamics.inertiaAtCOM_P*R.';
gravity=zeros(3,1); if m.dynamics.includeGravity, gravity=m.g; end
force=m.dynamics.totalMass*(a(1:3)+cross(alpha,r)+cross(w,cross(w,r))-gravity);
W=[force;I*alpha+cross(w,I*w)+cross(r,force)];
Sigma=SX.zeros(6,6);
if ~isempty(provider.network)
    [mu,Sigma]=residualMlpCasadiExpr([q;v],provider.network); W=W+mu;
end
if isfield(provider,'biasCalibration'), W=W+provider.biasCalibration.wrenchOffset; end
Jv=SX.zeros(6,6); L=SX.zeros(6,1);
for k=1:6
    rb=R*m.B(:,m.legMap(k)); s=q(1:3)+rb-m.A(:,k);
    L(k)=sqrt(s.'*s); u=s/L(k); Jv(k,:)=[u.',cross(rb,u).'];
end
Jq=Jv*diagcat(SX.eye(3),E);
if isfield(provider,'elasticity')
    elastic=provider.elasticity;
    W=W+Jv.'*(elastic.stiffness*(L-elastic.referenceLength)+elastic.damping*(Jq*v));
end
H=jacobian(W,a); bias=substitute(W,a,SX.zeros(6,1));
x=[q;v]; xd=[v;solve(H,Jv.'*F-bias)];
ad=struct();
ad.wrench=Function('abfd_wrench',{q,v,a},{W,Sigma});
ad.terms=Function('abfd_terms',{q,v},{H,bias,Jv,Jq,L,Sigma});
ad.f=Function('abfd_f',{x,F},{xd});
ad.linearize=Function('abfd_linearize',{x,F},{jacobian(xd,x),jacobian(xd,F)});
ad.inverse=Function('abfd_inverse',{q,v,a},{solve(Jv.',W)});
ad.geometry=Function('abfd_geometry',{q},{R,E});
ad.hash=provider.hash;
end
