function [cp,info]=dpOptimizeSpline(task,initial,c)
% Constrained fitting: preserve the IHSID-derived shape while repairing fit loss.
import casadi.*
timer=tic; p=dpPhysics(task.model); m=task.model;
t=linspace(0,task.T,121); [B,D,DD]=dpBasis(t,task.T); count=numel(t); nob=numel(task.scene.hood.obstacles);
z=MX.sym('controls',60); scale=repmat([.1;.1;.1;.2;.2;.2],10,1);
cpExpr=[initial(:,1:3),initial(:,4:13)+reshape(z.*scale,6,10),initial(:,14:16)];
Q=cpExpr*B; V=cpExpr*D; A=cpExpr*DD; S=MX.sym('separators',8,count*nob);
[L,Ld,Ldd,F,~,~,phi]=p.point(Q,V,A);
g={}; lo={}; hi={}; add(L,repmat(m.lmin,1,count),repmat(m.lmax,1,count));
add(Ld,-.6,.6); add(Ldd,-1.2,1.2); add(F,-2000,2000);
S0=zeros(8,count*nob); qi=initial*B;
for k=1:count
    [~,normals]=dpClearance(qi(:,k),task,true);
    R=p.pose(Q(:,k)); axis=R*task.scene.objectCylinder.axis_P;
    center=Q(1:3,k)+R*task.scene.objectCylinder.center_P;
    for j=1:nob
        ix=(k-1)*nob+j; o=task.scene.hood.obstacles(j); n=S(1:3,ix); eta=S(4:6,ix); zeta=S(7,ix); rho=S(8,ix);
        add(n.'*n,1,1); add(eta-o.R_S.'*n,0,inf); add(eta+o.R_S.'*n,0,inf);
        add(zeta-axis.'*n,0,inf); add(zeta+axis.'*n,0,inf);
        radial=n-axis*(axis.'*n); add(rho-sqrt(radial.'*radial+1e-12),0,inf);
        gap=n.'*(o.center_S-center)-o.halfSize.'*eta-.5*task.scene.objectCylinder.length*zeta-task.scene.objectCylinder.radius*rho;
        add(gap,.01+.0005*sin(pi*t(k)/task.T)^2,inf);
        nn=normals(:,j); rr=rpy2rotmZYX(qi(4:6,k)); aa=rr*task.scene.objectCylinder.axis_P;
        S0(:,ix)=[nn;abs(o.R_S.'*nn);abs(aa.'*nn);sqrt(sum((nn-aa*(aa.'*nn)).^2)+1e-12)];
    end
end
objective=sumsqr(z)+.001*sum2(phi)+.0001*sumsqr(Ldd);
g=vertcat(g{:}); decision=[z;S(:)]; solver=nlpsol('dp_spline_fit','ipopt',struct('x',decision,'g',g,'f',objective),dpOptions(c));
sol=solver('x0',[zeros(60,1);S0(:)],'lbg',vertcat(lo{:}),'ubg',vertcat(hi{:})); stats=solver.stats(); x=full(sol.x);
cp=initial; cp(:,4:13)=initial(:,4:13)+reshape(x(1:60).*scale,6,10);
info=struct('success',stats.success,'status',char(stats.return_status),'seconds',toc(timer),'iterations',stats.iter_count);
    function add(value,l,u)
        g{end+1}=value(:); lo{end+1}=repmat(l(:),numel(value)/numel(l),1); hi{end+1}=repmat(u(:),numel(value)/numel(u),1);
    end
end
