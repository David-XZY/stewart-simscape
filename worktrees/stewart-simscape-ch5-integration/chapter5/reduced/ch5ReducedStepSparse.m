function [proposed,info]=ch5ReducedStepSparse(x,previous,U,goal,o,prior,r,k)
% A blocked force-INCREMENT correction on a full, possibly unblocked warm U.
% Zero correction preserves that warm sequence. Dynamics are not relaxed.
timer=tic;H=r.c.H;nb=H/5;nv=6*nb;n=r.plant.n;scale=160;
nodes=full(k.nodes(x,U));[~,aa,bb]=k.stages(nodes(:,1:H),U);aa=full(aa);bb=full(bb);
M=zeros(6*H,nv);sens=zeros(n,nv,H+1);K=ch5ReducedFeedbackGain(x,r,k);
for j=1:H
    block=ceil(j/5);rows=6*(j-1)+(1:6);
    for b=1:block,M(rows,6*(b-1)+(1:6))=scale*eye(6);end
    A=aa(:,n*(j-1)+(1:n));B=bb(:,6*(j-1)+(1:6));
    M(rows,:)=M(rows,:)-K*sens(:,:,j);
    sens(:,:,j+1)=A*sens(:,:,j)+B*M(rows,:);
end
ps=[.1;.1;.1;deg2rad(10)*ones(3,1)];vs=[.1;.1;.1;.2;.2;.2];
Q=1e-6*eye(nv);f=zeros(nv,1);St=sens(1:6,:,end);qa=nodes(1:6,end);
for j=1:H
    S=sens(:,:,j+1);rows=6*(j-1)+(1:6);Mu=M(rows,:);
    [Q,f]=term(Q,f,(nodes(1:6,j+1)-qa)./ps,(S(1:6,:)-St)./ps,r.c.Ts);
    [Q,f]=term(Q,f,nodes(7:12,j+1)./vs,S(7:12,:)./vs,.1*r.c.Ts);
    [Q,f]=term(Q,f,(nodes(1:6,j+1)-goal)./ps,S(1:6,:)./ps,.05*r.c.Ts);
    if j==1,Du=Mu;D=U(:,1)-previous;else,Du=Mu-M(rows-6,:);D=U(:,j)-U(:,j-1);end
    [Q,f]=term(Q,f,D/160,Du/160,.01*r.c.Ts);
    [Q,f]=term(Q,f,(D-prior(:,j))/160,Du/160,.05);
    [Q,f]=term(Q,f,U(:,j)/2000,Mu/2000,.001*r.c.Ts);
end
[Q,f]=term(Q,f,(qa-goal)./ps,St./ps,100);
[ua,eqJac]=k.equilibrium(qa);ua=full(ua);eqJac=full(eqJac);
E=[sens(7:12,:,end)./vs;(M(end-5:end,:)-eqJac*St)/1000];
e=[nodes(7:12,end)./vs;(U(:,end)-ua)/1000];
% Endpoint constraints guide the step; full inner mesh is mandatory afterward.
states=reshape([nodes(:,1:H);nodes(:,2:end)],n,2*H);commands=repelem(U,1,2);
[values,vx,vu]=k.mechanics(states,commands);values=full(values);vx=full(vx);vu=full(vu);
lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];
hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
gs=[ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];
Aineq=zeros(2*H*38+H*24+numel(find(o(22,:)>.5))*2*H,nv);bineq=zeros(size(Aineq,1),1);support=zeros(size(Aineq,1),1);count=0;
for j=1:H
    before=count;rows=6*(j-1)+(1:6);Mu=M(rows,:);
    if j==1,Du=Mu;D=U(:,j)-previous;else,Du=Mu-M(rows-6,:);D=U(:,j)-U(:,j-1);end
    [Aineq,bineq,count]=bound(Aineq,bineq,count,U(:,j),Mu,r.strict.forceMin,r.strict.forceMax,2000);
    [Aineq,bineq,count]=bound(Aineq,bineq,count,D,Du,-r.strict.forceRateLimit*r.c.Ts,r.strict.forceRateLimit*r.c.Ts,160);
    for t=1:2
        vj=2*(j-1)+t;S=sens(:,:,j+t-1);
        C=vx(:,n*(vj-1)+(1:n))*S+vu(:,6*(vj-1)+(1:6))*Mu;
        [Aineq,bineq,count]=bound(Aineq,bineq,count,values(:,vj),C,lo,hi,gs);
        ot=o;ot(1:3,:)=o(1:3,:)+((j+t-2)*r.c.Ts)*o(16:18,:);
        for obstacle=find(o(22,:)>.5)
            [gap,grad]=k.collision(states(1:6,vj),ot(:,obstacle));
            count=count+1;Aineq(count,:)=-full(grad)*S(1:6,:)/.1;
            bineq(count)=(full(gap)-r.c.safe_distance-r.c.sample_hold_margin)/.1;
        end
    end
    support(before+1:count)=6*ceil(j/5);
end
Aineq=Aineq(1:count,:);bineq=bineq(1:count);support=support(1:count);linearizationSeconds=toc(timer);timer=tic;
import casadi.*
persistent solver solverCount
if isempty(solver)||isempty(solverCount)||solverCount~=count
    options=struct('error_on_fail',false,'print_time',false,'osqp',struct('verbose',false,'polish',false,'max_iter',100,'eps_abs',1e-4,'eps_rel',1e-4,'adaptive_rho_interval',25));
    pattern=DM(sparse([((1:nv)<=support);ones(12,nv)]));
    solver=conic('ch5_reduced_sparse_osqp','osqp',struct('h',Sparsity.dense(nv,nv),'a',pattern.sparsity()),options);solverCount=count;
end
raw=solver('h',(Q+Q')/2,'g',f,'a',sparse([Aineq;E]),'lba',[-1e30*ones(count,1);-e],'uba',[bineq;-e],'lbx',-ones(nv,1),'ubx',ones(nv,1),'x0',zeros(nv,1));
delta=full(raw.x);stats=solver.stats();flag=double(stats.success);output.iterations=-1;
if isfield(stats,'iter_count'),output.iterations=stats.iter_count;end
if any(contains(lower(string(stats.return_status)),["infeasible","non convex","unsolved"])),delta=[];end
info=struct('linearization_seconds',linearizationSeconds,'solver_seconds',toc(timer),'exitflag',flag, ...
    'iterations',output.iterations,'solver_status',stats.return_status,'variables',nv,'inequalities',count,'finite',~isempty(delta)&&all(isfinite(delta)));
proposed=U;
if info.finite
    rollout=ch5ReducedFeedbackRollout(r);proposed=full(rollout(x,nodes,U,K,reshape(delta,6,nb)));
end
end
function [Q,f]=term(Q,f,residual,J,weight)
Q=Q+2*weight*(J'*J);f=f+2*weight*J'*residual;
end
function [A,b,count]=bound(A,b,count,v,J,lo,hi,scale)
upper=isfinite(hi);lower=isfinite(lo);nu=nnz(upper);nl=nnz(lower);
if isscalar(scale),scale=repmat(scale,numel(v),1);end
A(count+(1:nu),:)=J(upper,:)./scale(upper);b(count+(1:nu))=(hi(upper)-v(upper))./scale(upper);count=count+nu;
A(count+(1:nl),:)=-J(lower,:)./scale(lower);b(count+(1:nl))=(v(lower)-lo(lower))./scale(lower);count=count+nl;
end
