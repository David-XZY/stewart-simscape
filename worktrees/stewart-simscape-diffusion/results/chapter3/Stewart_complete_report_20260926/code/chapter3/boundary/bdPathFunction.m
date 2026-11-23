function f=bdPathFunction(path)
% Exact symbolic derivatives of stored geometry, including SO(3) adaptation.
import casadi.*
persistent cacheKeys cacheValues;
if isempty(cacheKeys), cacheKeys={}; cacheValues={}; end
key=dpHash(path); hit=find(strcmp(cacheKeys,key),1);
if ~isempty(hit), f=cacheValues{hit}; return; end
s=SX.sym('s'); q=expression(path,s);
f=Function('bd_path',{s},{q,jacobian(q,s),jacobian(jacobian(q,s),s)});
cacheKeys=[cacheKeys,{key}]; cacheValues=[cacheValues,{f}];
if numel(cacheKeys)>32, cacheKeys=cacheKeys(end-31:end); cacheValues=cacheValues(end-31:end); end
end
function q=expression(path,s)
import casadi.*
if strcmp(path.kind,'adapted_path')
    qj=expression(path.source,s); p0=path.sourceQ0; pg=path.sourceGoal;
    Rj=rotation(qj(4:6)); R0=rpy2rotmZYX(p0(4:6)); Rg=rpy2rotmZYX(pg(4:6));
    D=rpy2rotmZYX(path.q0(4:6))*R0.'; target=rpy2rotmZYX(path.qGoal(4:6));
    E=target*(D*Rg).'; angle=acos(max(-1,min(1,(trace(E)-1)/2)));
    assert(angle<pi-1e-5,'BD:RotationBranch','Adapter crosses ambiguous pi logarithm.');
    if angle<1e-9, K=zeros(3); else, K=(E-E.')/(2*sin(angle)); end
    h=10*s^3-15*s^4+6*s^5;
    if strcmp(path.adapterId,'A7'), h=35*s^4-84*s^5+70*s^6-20*s^7; end
    R=(eye(3)+sin(h*angle)*K+(1-cos(h*angle))*(K*K))*D*Rj;
    pos=path.q0(1:3)+D*(qj(1:3)-p0(1:3))+h*(path.qGoal(1:3)-path.q0(1:3)-D*(pg(1:3)-p0(1:3)));
    q=[pos;atan2(R(3,2),R(3,3));atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));atan2(R(2,1),R(1,1))];
elseif strcmp(path.kind,'spline')
    n=size(path.cp,2); knots=[zeros(1,6),(1:n-6)/(n-5),ones(1,6)];
    b=SX.zeros(numel(knots)-1,1);
    for i=1:numel(knots)-1
        b(i)=if_else(s>=knots(i)&s<knots(i+1),1,0);
        if knots(i)<1 && knots(i+1)==1, b(i)=if_else(s==1,1,b(i)); end
    end
    for d=1:5
        v=SX.zeros(size(b,1)-1,1);
        for i=1:size(v,1)
            l=knots(i+d)-knots(i); r=knots(i+d+1)-knots(i+1);
            if l>0, v(i)=v(i)+(s-knots(i))/l*b(i); end
            if r>0, v(i)=v(i)+(knots(i+d+1)-s)/r*b(i+1); end
        end
        b=v;
    end
    q=path.cp*b;
elseif strcmp(path.kind,'ihsid')
    N=path.intervals; q=SX.zeros(6,1); h=path.h;
    for k=N:-1:1
        u=s*N-(k-1); q0=path.X(1:6,k); q1=path.X(1:6,k+1);
        v0=path.X(7:12,k); v1=path.X(7:12,k+1); a0=path.A(:,k); a1=path.A(:,k+1);
        r0=q1-q0-h*v0-.5*h^2*a0; r1=h*(v1-v0)-h^2*a0; r2=h^2*(a1-a0);
        val=q0+h*v0*u+.5*h^2*a0*u^2+(10*r0-4*r1+.5*r2)*u^3+(-15*r0+7*r1-r2)*u^4+(6*r0-3*r1+.5*r2)*u^5;
        if k==N, q=val; else, q=if_else(s<k/N,val,q); end
    end
else, error('BD:PathKind','Unsupported geometry kind: %s',path.kind); end
end
function R=rotation(q)
r=q(1); b=q(2); y=q(3);
R=[cos(y)*cos(b),cos(y)*sin(b)*sin(r)-sin(y)*cos(r),cos(y)*sin(b)*cos(r)+sin(y)*sin(r);sin(y)*cos(b),sin(y)*sin(b)*sin(r)+cos(y)*cos(r),sin(y)*sin(b)*cos(r)-cos(y)*sin(r);-sin(b),cos(b)*sin(r),cos(b)*cos(r)];
end
