function [cp,info]=c3Repair(task,initial,c,standard,taskClock)
% Low-dimensional AD repair. Coarse surrogate never certifies an output.
timer=tic; import casadi.*
t=linspace(0,task.T,61); h=t(2)-t(1); [B,D,DD]=sgBasis(t,task.T);
scale=repmat([.1;.1;.1;.2;.2;.2],10,1); z=SX.sym('z',60);
control=SX(initial); control(:,4:13)=control(:,4:13)+reshape(z.*scale,6,10);
Q=control*B; V=control*D; A=control*DD; M=numel(t);
n=SX.sym('n',3,6*M); centers=SX.sym('centers',3,6*M); supports=SX.sym('supports',1,6*M); mask=SX.sym('mask',1,6*M);
physical=sgRepairPoint(task); [violations,~]=physical(Q,V,A,n,centers,supports,mask);
p=sgPhysics(task.model); [~,~,Ldd,F]=p.point(Q,V,A);
b=Q(5,:); y=Q(6,:); rd=V(4,:); bd=V(5,:); yd=V(6,:);
omega=[cos(y).*cos(b).*rd-sin(y).*bd;sin(y).*cos(b).*rd+cos(y).*bd;-sin(b).*rd+yd];
alpha=[cos(y).*cos(b).*A(4,:)-sin(y).*A(5,:)+(-sin(y).*yd.*cos(b)-cos(y).*sin(b).*bd).*rd-cos(y).*yd.*bd; ...
    sin(y).*cos(b).*A(4,:)+cos(y).*A(5,:)+(cos(y).*yd.*cos(b)-sin(y).*sin(b).*bd).*rd-sin(y).*yd.*bd; ...
    -sin(b).*A(4,:)+A(6,:)-cos(b).*bd.*rd];
normcols=@(x)sqrt(sum1(x.^2)+1e-18);
rmsnorm=@(x)sqrt(sum2(sum1(x.^2))/size(x,2)+1e-18);
deriv=@(x)(x(:,3:end)-x(:,1:end-2))/(2*h);
metrics=[h*sum2(normcols(V(1:3,:)));h*sum2(normcols(omega));rmsnorm(A(1:3,:));rmsnorm(alpha); ...
    rmsnorm(Ldd)/sqrt(6);rmsnorm(deriv(A(1:3,:)));rmsnorm(deriv(alpha));rmsnorm(deriv(Ldd))/sqrt(6); ...
    rmsnorm(F)/sqrt(6);rmsnorm(deriv(F))/sqrt(6)];
normed=metrics./standard.scales(:); groups=SX.zeros(4,1);
for g=1:4, groups(g)=sum1(normed(standard.groups{g}))/numel(standard.groups{g}); end
quality=standard.weights(standard.primary,:)*groups;
penalty=sum2(violations)/M; objective=100*penalty+quality;
fun=Function('c3_repair',{z,n,centers,supports,mask},{objective,gradient(objective,z),penalty,quality});
info=struct('seconds',0,'buildSeconds',toc(timer),'iterations',0,'steps',0,'reason','iteration_limit','before',NaN,'after',NaN);
value=zeros(60,1); cp=initial;
for iteration=1:c.repairIterations
    info.iterations=iteration;
    if toc(timer)>=c.repairBudget || toc(taskClock)>=c.wallBudget, info.reason='budget'; break; end
    [~,normals]=sgClearanceBatch(cp*B,task); count=numel(task.scene.hood.obstacles);
    ns=zeros(3,6,M); ns(:,1:count,:)=normals; cs=zeros(3,6); hs=zeros(1,6,M); ms=zeros(1,6); ms(1:count)=1;
    for j=1:count
        o=task.scene.hood.obstacles(j); cs(:,j)=o.center_S; nn=reshape(ns(:,j,:),3,M);
        hs(1,j,:)=reshape(o.halfSize.'*abs(o.R_S.'*nn),1,1,M);
    end
    args={reshape(ns,3,6*M),repmat(cs,1,M),reshape(hs,1,6*M),repmat(ms,1,M)};
    [cost,grad]=fun(value,args{:}); cost=full(cost); grad=full(grad);
    if iteration==1, info.before=cost; end
    if ~isfinite(cost) || any(~isfinite(grad)), info.reason='nonfinite'; break; end
    direction=grad/max(1e-12,sqrt(mean(grad.^2))); accepted=false;
    for backtrack=0:6
        if toc(timer)>=c.repairBudget || toc(taskClock)>=c.wallBudget, break; end
        trial=value-.02/(2^backtrack)*direction; if sqrt(mean(trial.^2))>.25, continue; end
        trialCost=full(fun(trial,args{:}));
        if isfinite(trialCost) && trialCost<cost
            value=trial; cp=initial; cp(:,4:13)=cp(:,4:13)+reshape(scale.*value,6,10);
            info.steps=info.steps+1; info.after=trialCost; accepted=true; break;
        end
    end
    if ~accepted, info.reason='line_search_or_budget'; break; end
end
info.normalizedChangeRms=sqrt(mean(value.^2)); info.seconds=toc(timer);
end
