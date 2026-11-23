function [cp,info]=sgRepair(task,initial,c,taskClock)
timer=tic; build=tic; fun=sgRepairPoint(task); buildSeconds=toc(build);
t=linspace(0,task.T,61); [B,D,DD]=sgBasis(t,task.T); scales=repmat([.1;.1;.1;.2;.2;.2],10,1);
cp=initial; z=zeros(60,1); steps=0; iteration=0; before=NaN; after=NaN; reason='iteration_limit';
for iteration=1:c.repairIterations
    if toc(timer)>c.repairBudget || toc(taskClock)>c.fastBudget, reason='budget'; break; end
    Q=cp*B; V=cp*D; A=cp*DD;
    [~,normals]=sgClearanceBatch(Q,task); M=size(Q,2); count=numel(task.scene.hood.obstacles);
    ns=zeros(3,6,M); ns(:,1:count,:)=normals; centers=zeros(3,6); supports=zeros(1,6,M); mask=zeros(1,6); mask(1:count)=1;
    for j=1:count
        o=task.scene.hood.obstacles(j); centers(:,j)=o.center_S;
        nn=reshape(ns(:,j,:),3,M); supports(1,j,:)=reshape(o.halfSize.'*abs(o.R_S.'*nn),1,1,M);
    end
    narg=reshape(ns,3,6*M); carg=repmat(centers,1,M); harg=reshape(supports,1,6*M); marg=repmat(mask,1,M);
    [value,g]=fun(Q,V,A,narg,carg,harg,marg); cost=mean(full(value)); g=full(g);
    derivative=(g(1:6,:)*B.'+g(7:12,:)*D.'+g(13:18,:)*DD.')/M;
    grad=reshape(derivative(:,4:13),[],1).*scales;
    if iteration==1, before=cost; end
    if ~isfinite(cost) || any(~isfinite(grad)), reason='nonfinite_gradient'; break; end
    if cost<1e-12, reason='coarse_cost_small'; after=cost; break; end
    direction=grad/max(1e-12,sqrt(mean(grad.^2))); accepted=false;
    for bt=0:4
        if toc(timer)>c.repairBudget || toc(taskClock)>c.fastBudget, break; end
        trial=z-.02/(2^bt)*direction; if sqrt(mean(trial.^2))>.25, continue; end
        proposal=initial; proposal(:,4:13)=initial(:,4:13)+reshape(scales.*trial,6,10);
        val=mean(full(fun(proposal*B,proposal*D,proposal*DD,narg,carg,harg,marg)));
        if isfinite(val) && val<cost
            z=trial; cp=proposal; steps=steps+1; after=val; accepted=true; break;
        end
    end
    if ~accepted, reason='line_search_or_budget'; break; end
end
info=struct('seconds',toc(timer),'buildSeconds',buildSeconds,'iterations',iteration, ...
    'acceptedSteps',steps,'initialSurrogateCost',before,'lastSurrogateCost',after, ...
    'normalizedChangeRms',sqrt(mean(z.^2)),'reason',reason, ...
    'scope','Fixed-support-plane AD per line search; final independent acceptance required.');
end
