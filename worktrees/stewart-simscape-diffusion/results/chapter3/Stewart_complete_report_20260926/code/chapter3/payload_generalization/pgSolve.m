function result=pgSolve(task,cp,c,warm)
if nargin<4, warm=[]; end
% Full-horizon implicit Hermite-Simpson inverse-dynamics optimization.
import casadi.*
timer=tic; assert(task.endpointPassed,'DP:Endpoint','%s',task.endpointReason);
assert(all(isfinite(cp(:))),'DP:Nonfinite','Nonfinite seed.');
p=pgPhysics(task.model); m=task.model; N=c.intervals; h=task.T/N;
tn=linspace(0,task.T,N+1); tm=tn(1:end-1)+h/2; tt=[tn,tm]; M=numel(tt);
seed=bdReconstruct(struct('kind','spline','cp',cp,'T',task.T),tt); reference=sgSpline(dpBaseline(task),tt,task.T); nob=numel(task.scene.hood.obstacles);
persistent problems;
if isempty(problems), problems=containers.Map('KeyType','char','ValueType','any'); end
options=dpOptions(c); options.ipopt.warm_start_init_point='yes'; key=pgProblemKey(task,c);
cacheHit=isKey(problems,key); if cacheHit, cachedProblem=problems(key); end
[~,~,~,F0]=p.point(seed.q,seed.v,seed.a); F0=full(F0); S0=zeros(9,nob*M);
for i=1:M
    [~,normals]=pgClearance(seed.q(:,i),task,false);
    rr=rpy2rotmZYX(seed.q(4:6,i)); aa=rr(:,1);
    for j=1:nob
        si=(i-1)*nob+j; o=task.scene.hood.obstacles(j); nn=normals(:,j);
        if strcmp(task.payload.shape,'cylinder'), obj=[abs(aa.'*nn);sqrt(sum((nn-aa*(aa.'*nn)).^2)+1e-12);0]; else, obj=abs(rr.'*nn); end
        S0(:,si)=[nn;abs(o.R_S.'*nn);obj];
    end
end
z0=[reshape([seed.q;seed.v],[],1);seed.a(:);F0(:)/1000;S0(:)];
initializationSeconds=toc(timer); graphClock=tic;
g={}; lo={}; hi={};
if ~cacheHit
X=MX.sym('X',12,M); A=MX.sym('A',6,M); U=MX.sym('U',6,M); S=MX.sym('S',9,nob*M);
nominal=MX.sym('nominal',6,M); pars=MX.sym('task_parameters',29+16*nob); rigid=pars(14+16*nob:26+16*nob); dims=pars(27+16*nob:29+16*nob);
h=pars(13)/N;
[L,Ld,Ldd,Fneed,Jv,W,phi]=p.parameterPoint(X(1:6,:),X(7:12,:),A,rigid);
J=MX(0);
add(X(:,1)-[pars(1:6);zeros(6,1)],0,0); add(X(:,N+1)-[pars(7:12);zeros(6,1)],0,0);
add(A(:,[1,N+1]),0,0);
for i=1:M
    ji=Jv(:,(i-1)*6+(1:6)); add(ji.'*(1000*U(:,i))-W(:,i),0,0);
    add(L(:,i),m.lmin,m.lmax); add(Ld(:,i),-m.actuator.ldotMax,m.actuator.ldotMax);
    add(Ldd(:,i),-m.actuator.lddotMax,m.actuator.lddotMax);
    for j=1:nob
        si=(i-1)*nob+j; n=S(1:3,si); eta=S(4:6,si); zeta=S(7,si); rho=S(8,si);
        off=13+16*(j-1); o=struct('center_S',pars(off+(1:3)),'halfSize',pars(off+(4:6)),'R_S',reshape(pars(off+(7:15)),3,3)); mask=pars(off+16); R=p.pose(X(1:6,i)); axis=R(:,1);
        center=X(1:3,i)+R*[0;0;.15]; radial=n-axis*(axis.'*n);
        add(n.'*n,1,1); add(eta-o.R_S.'*n,0,inf); add(eta+o.R_S.'*n,0,inf);
        if strcmp(task.payload.shape,'cylinder')
            add(zeta-axis.'*n,0,inf); add(zeta+axis.'*n,0,inf); add(rho-sqrt(radial.'*radial+1e-12),0,inf); add(S(9,si),0,0);
            support=.5*dims(1)*zeta+.5*dims(2)*rho;
        else
            obj=S(7:9,si); add(obj-R.'*n,0,inf); add(obj+R.'*n,0,inf); support=.5*dims.'*obj;
        end
        gap=n.'*(o.center_S-center)-o.halfSize.'*eta-support;
        % Interior optimization cushion protects inter-sample interpolation;
        % the user-facing dense threshold stays exactly 10 mm, including goal.
        add(mask*(gap-(.01+.001*sin(pi*tt(i)/task.T)^2)),0,inf);
        robot=bdEnvelopeSymbolic(X(1:6,i),o,m,c.envelope,p); add(mask*(robot-.01),0,inf);
    end
    deviation=sum1(((X(1:6,i)-nominal(:,i))./[.1;.1;.1;deg2rad(10)*ones(3,1)]).^2);
    weight=h/3; if i>N+1, weight=2*h/3; elseif i==1 || i==N+1, weight=h/6; end
    J=J+weight*(.1*deviation+.3*sum1((Ldd(:,i)/1.2).^2)+.4*phi(i));
end
for k=1:N
    mid=N+1+k; f0=[X(7:12,k);A(:,k)]; f1=[X(7:12,k+1);A(:,k+1)]; fc=[X(7:12,mid);A(:,mid)];
    add(X(:,mid)-(X(:,k)+X(:,k+1))/2-h/8*(f0-f1),0,0);
    add(X(:,k+1)-X(:,k)-h/6*(f0+4*fc+f1),0,0);
    J=J+.08*h/2*(sum1(((U(:,mid)-U(:,k))/(h/2)).^2)+sum1(((U(:,k+1)-U(:,mid))/(h/2)).^2));
end
z=[X(:);A(:);U(:);S(:)];
lower=-inf(size(z0)); upper=inf(size(z0)); first=18*M+1; last=24*M;
lower(first:last)=repmat(m.actuator.forceMin/1000,M,1); upper(first:last)=repmat(m.actuator.forceMax/1000,M,1);
g=vertcat(g{:}); lowerG=vertcat(lo{:}); upperG=vertcat(hi{:});
callback=bdDeadline(numel(z0),numel(lowerG),numel(nominal)+numel(pars)); options.iteration_callback=callback;
solver=nlpsol('dp_ihsid','ipopt',struct('x',z,'p',[nominal(:);pars],'f',J,'g',g),options);
cachedProblem=struct('callback',callback,'solver',solver,'lower',lower,'upper',upper,'lowerG',lowerG,'upperG',upperG); problems(key)=cachedProblem;
else
    solver=cachedProblem.solver; lower=cachedProblem.lower; upper=cachedProblem.upper;
    lowerG=cachedProblem.lowerG; upperG=cachedProblem.upperG;
end
buildSeconds=toc(graphClock);
if isfield(c,'buildOnly')&&c.buildOnly, result=struct('buildSeconds',buildSeconds,'cacheHit',cacheHit,'trajectorySolveCalls',0); return; end
cachedProblem.callback.clockHandle=c.clockHandle; cachedProblem.callback.budget=c.wallBudget;
warmClock=tic;
warmInfo=struct('state',false,'velocity',false,'acceleration',false,'force',false,'dual',false,'endpointAdapted',false);
parameterValues=[task.q0;task.qGoal;task.T];
for jj=1:nob
    oo=task.scene.hood.obstacles(jj); parameterValues=[parameterValues;oo.center_S;oo.halfSize;oo.R_S(:);1];
end
parameterValues=[parameterValues;p.parameters;task.payload.boundingDimensions];
h=task.T/N;
args={'x0',z0,'p',[reference.q(:);parameterValues],'lbx',lower,'ubx',upper,'lbg',lowerG,'ubg',upperG};
if ~isempty(warm) && isfield(warm,'bundle')
    old=warm.bundle; tr=c3TransportWarm(old,task,tt);
    z0(1:12*M)=reshape([tr.q;tr.v],[],1);
    z0(12*M+(1:6*M))=tr.a(:);
    warmInfo.state=true; warmInfo.velocity=true; warmInfo.acceleration=true; warmInfo.endpointAdapted=true;
    [~,~,~,newForce]=p.point(tr.q,tr.v,tr.a); z0(18*M+(1:6*M))=reshape(full(newForce),[],1)/1000; warmInfo.force=true;
    % Rebuild separators for changed obstacles; duals only for identical structure.
    args{2}=z0;
    if isfield(warm,'structureHash') && strcmp(warm.structureHash,key) && ...
            numel(warm.lamX)==numel(z0) && numel(warm.lamG)==numel(lowerG)
        args=[args,{'lam_x0',warm.lamX,'lam_g0',warm.lamG}]; warmInfo.dual=true;
    end
end
initializationSeconds=initializationSeconds+toc(warmClock);
fprintf('IHSID %s built in %.1fs; %d variables, %d constraints\n',task.taskId,buildSeconds,numel(z0),numel(lowerG));
solveTimer=tic;
assert(toc(c.clockHandle)<c.wallBudget,'BD:Deadline','Expired before solver call.');
if isfield(c,'debugInitial') && c.debugInitial
    gf=solver.get_function('nlp_g'); initialG=full(gf(z0,[reference.q(:);parameterValues]));
    initialEq=isfinite(lowerG)&lowerG==upperG;
    fprintf('BD_INITIAL eq %.5g ineq %.5g\n',max(abs(initialG(initialEq)-lowerG(initialEq))),max([0;lowerG(~initialEq)-initialG(~initialEq);initialG(~initialEq)-upperG(~initialEq)]));
end
sol=solver(args{:});
seconds=toc(solveTimer); stats=solver.stats(); value=full(sol.x); gv=full(sol.g);
Xv=reshape(value(1:12*M),12,M); Av=reshape(value(12*M+(1:6*M)),6,M);
Fv=1000*reshape(value(18*M+(1:6*M)),6,M);
eq=isfinite(lowerG)&lowerG==upperG;
res=max(abs(gv(eq)-lowerG(eq))); violation=max([0;lowerG(~eq)-gv(~eq);gv(~eq)-upperG(~eq)]);
boundViolation=max([0;lower-value;value-upper]);
finite=all(isfinite([value;gv;full(sol.f)]));
bundle=struct('kind','ihsid','T',task.T,'h',h,'tNode',tn,'X',Xv,'A',Av,'F',Fv,'intervals',N, ...
    'separators',reshape(value(24*M+1:end),9,nob*M));
validationClock=tic;
try
    report=pgValidate(bundle,task,c,c.clockHandle,c.wallBudget);
catch e
    report=struct('passed',false,'reason',['validation_exception:',e.identifier],'minClearance',NaN);
end
validationSeconds=toc(validationClock);
result=struct('schemaVersion',1,'taskId',task.taskId,'taskHash',task.hash,'sourceSeed',cp, ...
    'bundle',bundle,'report',report,'solverStatus',char(stats.return_status),'solverSuccess',stats.success, ...
    'iterations',stats.iter_count,'solveSeconds',seconds,'initializationSeconds',initializationSeconds,'buildSeconds',buildSeconds,'problemCacheHit',cacheHit, ...
    'maxEquality',res,'maxInequality',violation,'objective',full(sol.f), ...
    'maxBounds',boundViolation,'finite',finite,'validationSeconds',validationSeconds,'warmInfo',warmInfo, ...
    'passed',c3SolverGate(stats.success,char(stats.return_status),res,violation,boundViolation,finite,toc(c.clockHandle)<=c.wallBudget,report.passed));
result.warm=struct('bundle',bundle,'lamX',full(sol.lam_x),'lamG',full(sol.lam_g), ...
    'structureHash',key);
result.totalSeconds=toc(timer); result.commonReference=true;
if toc(c.clockHandle)>c.wallBudget, result.passed=false; result.solverStatus=[result.solverStatus,'; task_wall_budget']; end
fprintf('%s: %s, eq %.2g, ineq %.2g, clearance %.6f, dense %d, %.1fs\n',task.taskId,result.solverStatus,res,violation,report.minClearance,result.passed,result.totalSeconds);
    function add(v,l,u)
        g{end+1}=v(:); lo{end+1}=repmat(l(:),numel(v)/numel(l),1); hi{end+1}=repmat(u(:),numel(v)/numel(u),1);
    end
end
