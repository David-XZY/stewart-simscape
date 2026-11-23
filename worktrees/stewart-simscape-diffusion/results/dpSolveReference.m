function result=dpSolveReference(task,cp,c)
% Full-horizon implicit Hermite-Simpson inverse-dynamics optimization.
import casadi.*
timer=tic; assert(task.endpointPassed,'DP:Endpoint','%s',task.endpointReason);
assert(all(isfinite(cp(:))),'DP:Nonfinite','Nonfinite seed.');
p=dpPhysics(task.model); m=task.model; N=c.intervals; h=task.T/N;
tn=linspace(0,task.T,N+1); tm=tn(1:end-1)+h/2; tt=[tn,tm]; M=numel(tt);
seed=dpSpline(cp,tt,task.T); nob=numel(task.scene.hood.obstacles);
X=MX.sym('X',12,M); A=MX.sym('A',6,M); U=MX.sym('U',6,M); S=MX.sym('S',8,nob*M);
[L,Ld,Ldd,Fneed,Jv,W,phi]=p.point(X(1:6,:),X(7:12,:),A);
g={}; lo={}; hi={}; J=MX(0);
add(X(:,1)-[task.q0;zeros(6,1)],0,0); add(X(:,N+1)-[task.qGoal;zeros(6,1)],0,0);
add(A(:,[1,N+1]),0,0);
[~,~,~,F0]=p.point(seed.q,seed.v,seed.a); F0=full(F0);
S0=zeros(8,nob*M);
for i=1:M
    ji=Jv(:,(i-1)*6+(1:6)); add(ji.'*(1000*U(:,i))-W(:,i),0,0);
    add(L(:,i),m.lmin,m.lmax); add(Ld(:,i),-m.actuator.ldotMax,m.actuator.ldotMax);
    add(Ldd(:,i),-m.actuator.lddotMax,m.actuator.lddotMax);
    [~,normals]=dpClearance(seed.q(:,i),task,false);
    for j=1:nob
        si=(i-1)*nob+j; n=S(1:3,si); eta=S(4:6,si); zeta=S(7,si); rho=S(8,si);
        o=task.scene.hood.obstacles(j); R=p.pose(X(1:6,i)); axis=R*task.scene.objectCylinder.axis_P;
        center=X(1:3,i)+R*task.scene.objectCylinder.center_P; radial=n-axis*(axis.'*n);
        add(n.'*n,1,1); add(eta-o.R_S.'*n,0,inf); add(eta+o.R_S.'*n,0,inf);
        add(zeta-axis.'*n,0,inf); add(zeta+axis.'*n,0,inf); add(rho-sqrt(radial.'*radial+1e-12),0,inf);
        gap=n.'*(o.center_S-center)-o.halfSize.'*eta-.5*task.scene.objectCylinder.length*zeta-task.scene.objectCylinder.radius*rho;
        % Interior optimization cushion protects inter-sample interpolation;
        % the user-facing dense threshold stays exactly 10 mm, including goal.
        add(gap,.01+.001*sin(pi*tt(i)/task.T)^2,inf);
        nn=normals(:,j); rr=rpy2rotmZYX(seed.q(4:6,i)); aa=rr*task.scene.objectCylinder.axis_P;
        S0(:,si)=[nn;abs(o.R_S.'*nn);abs(aa.'*nn);sqrt(sum((nn-aa*(aa.'*nn)).^2)+1e-12)];
    end
    deviation=sum1(((X(1:6,i)-seed.q(:,i))./[.1;.1;.1;deg2rad(10)*ones(3,1)]).^2);
    weight=h/3; if i>N+1, weight=2*h/3; elseif i==1 || i==N+1, weight=h/6; end
    J=J+weight*(.1*deviation+.3*sum1((Ldd(:,i)/1.2).^2)+.4*phi(i));
end
for k=1:N
    mid=N+1+k; f0=[X(7:12,k);A(:,k)]; f1=[X(7:12,k+1);A(:,k+1)]; fc=[X(7:12,mid);A(:,mid)];
    add(X(:,mid)-(X(:,k)+X(:,k+1))/2-h/8*(f0-f1),0,0);
    add(X(:,k+1)-X(:,k)-h/6*(f0+4*fc+f1),0,0);
    J=J+.08*h/2*(sum1(((U(:,mid)-U(:,k))/(h/2)).^2)+sum1(((U(:,k+1)-U(:,mid))/(h/2)).^2));
end
z=[X(:);A(:);U(:);S(:)]; z0=[reshape([seed.q;seed.v],[],1);seed.a(:);F0(:)/1000;S0(:)];
lower=-inf(size(z0)); upper=inf(size(z0)); first=18*M+1; last=24*M;
lower(first:last)=-2; upper(first:last)=2;
g=vertcat(g{:}); lowerG=vertcat(lo{:}); upperG=vertcat(hi{:});
solver=nlpsol('dp_ihsid','ipopt',struct('x',z,'f',J,'g',g),dpOptions(c));
buildSeconds=toc(timer); solveTimer=tic;
fprintf('IHSID %s built in %.1fs; %d variables, %d constraints\n',task.taskId,buildSeconds,numel(z0),numel(lowerG));
sol=solver('x0',z0,'lbx',lower,'ubx',upper,'lbg',lowerG,'ubg',upperG);
seconds=toc(solveTimer); stats=solver.stats(); value=full(sol.x); gv=full(sol.g);
Xv=reshape(value(1:12*M),12,M); Av=reshape(value(12*M+(1:6*M)),6,M);
Fv=1000*reshape(value(18*M+(1:6*M)),6,M);
eq=isfinite(lowerG)&lowerG==upperG;
res=max(abs(gv(eq)-lowerG(eq))); violation=max([0;lowerG(~eq)-gv(~eq);gv(~eq)-upperG(~eq)]);
bundle=struct('kind','ihsid','T',task.T,'h',h,'tNode',tn,'X',Xv,'A',Av,'F',Fv,'intervals',N, ...
    'separators',reshape(value(24*M+1:end),8,nob*M));
report=dpValidate(bundle,task,c);
result=struct('schemaVersion',1,'taskId',task.taskId,'taskHash',task.hash,'sourceSeed',cp, ...
    'bundle',bundle,'report',report,'solverStatus',char(stats.return_status),'solverSuccess',stats.success, ...
    'iterations',stats.iter_count,'solveSeconds',seconds,'buildSeconds',buildSeconds, ...
    'maxEquality',res,'maxInequality',violation,'objective',full(sol.f), ...
    'passed',stats.success && res<=1e-6 && violation<=1e-6 && report.passed);
result.totalSeconds=toc(timer);
fprintf('%s: %s, eq %.2g, ineq %.2g, clearance %.6f, dense %d, %.1fs\n',task.taskId,result.solverStatus,res,violation,report.minClearance,result.passed,result.totalSeconds);
    function add(v,l,u)
        g{end+1}=v(:); lo{end+1}=repmat(l(:),numel(v)/numel(l),1); hi{end+1}=repmat(u(:),numel(v)/numel(u),1);
    end
end
