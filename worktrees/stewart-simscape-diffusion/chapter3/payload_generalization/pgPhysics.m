function p=pgPhysics(m)
% One symbolic graph per fixed robot; payload dynamics are NUMERICAL parameters.
import casadi.*
persistent key kernel;
fixed=m; fixed=rmfield(fixed,'dynamics'); k=dpHash(fixed);
if isempty(key)||~strcmp(key,k)
    pars=SX.sym('rigid_parameters',13); symbolic=m;
    symbolic.dynamics.totalMass=pars(1); symbolic.dynamics.comP=pars(2:4);
    symbolic.dynamics.inertiaAtCOM_P=reshape(pars(5:13),3,3);
    q=SX.sym('q',6); v=SX.sym('v',6); a=SX.sym('a',6);
    % Build the exact existing equations with explicit extra inputs.
    r=q(4); b=q(5); y=q(6);
    R=[cos(y)*cos(b),cos(y)*sin(b)*sin(r)-sin(y)*cos(r),cos(y)*sin(b)*cos(r)+sin(y)*sin(r);sin(y)*cos(b),sin(y)*sin(b)*sin(r)+cos(y)*cos(r),sin(y)*sin(b)*cos(r)-cos(y)*sin(r);-sin(b),cos(b)*sin(r),cos(b)*cos(r)];
    E=[cos(y)*cos(b),-sin(y),0;sin(y)*cos(b),cos(y),0;-sin(b),0,1];
    w=E*v(4:6); alpha=jacobian(w,q)*v+E*a(4:6);
    rc=R*pars(2:4); I=R*reshape(pars(5:13),3,3)*R.';
    f=pars(1)*(a(1:3)+cross(alpha,rc)+cross(w,cross(w,rc))-m.g);
    W=[f;I*alpha+cross(w,I*w)+cross(rc,f)]; L=SX.zeros(6,1); J=SX.zeros(6,6);
    for i=1:6
        rb=R*m.B(:,m.legMap(i)); d=q(1:3)+rb-m.A(:,i); L(i)=sqrt(d.'*d); u=d/L(i); J(i,:)=[u.',cross(rb,u).'];
    end
    Ld=jacobian(L,q)*v; Ldd=jacobian(L,q)*a+jacobian(Ld,q)*v;
    F=solve(J.',W); Jn=J*diag([1,1,1,2,2,2]); phi=.01*trace(solve(Jn*Jn.'+2.5e-5*SX.eye(6),SX.eye(6)));
    kernel=struct('parameterPoint',Function('pg_parameter_point',{q,v,a,pars},{L,Ld,Ldd,F,J,W,phi}), ...
        'pose',Function('pg_pose',{q},{R}),'rotation',Function('pg_rotation',{q,v,a},{R,w,alpha,E})); key=k;
end
p=kernel; p.parameters=[m.dynamics.totalMass;m.dynamics.comP;m.dynamics.inertiaAtCOM_P(:)];
p.point=@point; p.model=m;
    function varargout=point(q,v,a)
        [varargout{1:nargout}]=kernel.parameterPoint(q,v,a,p.parameters);
    end
end
