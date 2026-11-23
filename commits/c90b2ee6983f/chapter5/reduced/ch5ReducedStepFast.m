function [proposed,info]=ch5ReducedStepFast(x,previous,U,goal,o,prior,r,k,debugOnly)
% Batch the exact same stage objective / constraints; no approximate Jacobian.
timer=tic;phase=tic;H=r.c.H;nb=H/5;nv=6*nb;n=r.plant.n;
nodes=full(k.nodes(x,U));nodeSeconds=toc(phase);phase=tic;
[~,aa,bb]=k.stages(nodes(:,1:H),U);aa=full(aa);bb=full(bb);stageSeconds=toc(phase);phase=tic;
K=ch5ReducedFeedbackGain(x,r,k);M=zeros(6*H,nv);S=zeros(n,nv,H+1);
for j=1:H
    rows=6*(j-1)+(1:6);block=ceil(j/5);
    for b=1:block,M(rows,6*(b-1)+(1:6))=160*eye(6);end
    M(rows,:)=M(rows,:)-K*S(:,:,j);
    S(:,:,j+1)=aa(:,n*(j-1)+(1:n))*S(:,:,j)+bb(:,6*(j-1)+(1:6))*M(rows,:);
end
condensingSeconds=toc(phase);phase=tic;
ps=[.1;.1;.1;deg2rad(10)*ones(3,1)];vs=[.1;.1;.1;.2;.2;.2];qa=nodes(1:6,end);St=S(1:6,:,end);
qSensitivity=S(1:6,:,2:end);vSensitivity=S(7:12,:,2:end);
Jq=flatten(qSensitivity./ps);Jp=flatten((qSensitivity-St)./ps);Jv=flatten(vSensitivity./vs);
Md=M-[zeros(6,nv);M(1:end-6,:)];D=diff([previous,U],1,2);
J=[sqrt(r.c.Ts)*Jp;sqrt(.1*r.c.Ts)*Jv;sqrt(.05*r.c.Ts)*Jq; ...
    sqrt(.01*r.c.Ts)*Md/160;sqrt(.05)*Md/160;sqrt(.001*r.c.Ts)*M/2000;10*St./ps];
residual=[sqrt(r.c.Ts)*reshape((nodes(1:6,2:end)-qa)./ps,[],1); ...
    sqrt(.1*r.c.Ts)*reshape(nodes(7:12,2:end)./vs,[],1); ...
    sqrt(.05*r.c.Ts)*reshape((nodes(1:6,2:end)-goal)./ps,[],1); ...
    sqrt(.01*r.c.Ts)*D(:)/160;sqrt(.05)*(D(:)-prior(:))/160;sqrt(.001*r.c.Ts)*U(:)/2000;10*(qa-goal)./ps];
Q=2*(J'*J)+1e-6*eye(nv);f=2*J'*residual;
[ua,eqJac]=k.equilibrium(qa);ua=full(ua);eqJac=full(eqJac);
E=[S(7:12,:,end)./vs;(M(end-5:end,:)-eqJac*St)/1000];e=[nodes(7:12,end)./vs;(U(:,end)-ua)/1000];
states=reshape([nodes(:,1:H);nodes(:,2:end)],n,2*H);commands=repelem(U,1,2);
[values,vx,vu]=k.mechanics(states,commands);values=full(values);vx=full(vx);vu=full(vu);
Sj=repelem(S,1,1,2);Sj=Sj(:,:,2:end-1);Mj=repelem(reshape(M',nv,6,H),1,1,2);Mj=permute(Mj,[2,1,3]);
C=pagemtimes(reshape(vx,19,n,2*H),Sj)+pagemtimes(reshape(vu,19,6,2*H),Mj);
lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];
hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
gs=[ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];finiteHi=isfinite(hi);
mechanicalA=[C(finiteHi,:,:)./gs(finiteHi);-C./gs];
mechanicalB=[(hi(finiteHi)-values(finiteHi,:))./gs(finiteHi);(values-lo)./gs];
active=find(o(22,:)>.5);
if ~isempty(active)
    mapped=k.collision.map(2*H);times=reshape([(0:H-1);(1:H)],1,[])*r.c.Ts;
    for oi=active
        ot=repmat(o(:,oi),1,2*H);ot(1:3,:)=ot(1:3,:)+o(16:18,oi)*times;
        [gap,grad]=mapped(states(1:6,:),ot);gradient=reshape(full(grad),1,6,2*H);
        a=-pagemtimes(gradient,Sj(1:6,:,:))/.1;
        b=(full(gap)-r.c.safe_distance-r.c.sample_hold_margin)/.1;
        mechanicalA=[mechanicalA;a];mechanicalB=[mechanicalB;b]; %#ok<AGROW>
    end
end
nu=size(mechanicalA,1);mechanicalA=reshape(permute(reshape(mechanicalA,nu,nv,2,H),[1,3,2,4]),2*nu,nv,H);
mechanicalB=reshape(mechanicalB,2*nu,H);
Mu=permute(reshape(M',nv,6,H),[2,1,3]);Du=permute(reshape(Md',nv,6,H),[2,1,3]);
stageA=[Mu/2000;-Mu/2000;Du/160;-Du/160;mechanicalA];
stageB=[(r.strict.forceMax-U)/2000;(U-r.strict.forceMin)/2000; ...
    (r.strict.forceRateLimit*r.c.Ts-D)/160;(D+r.strict.forceRateLimit*r.c.Ts)/160;mechanicalB];
A=flatten(stageA);b=stageB(:);originalCount=numel(b);
if nargin>8&&debugOnly
    proposed=U;info=struct('Q',Q,'f',f,'A',A,'b',b,'E',E,'e',e,'M',M,'nodes',nodes);return;
end
keep=b<sum(abs(A),2)+1e-10;A=A(keep,:);b=b(keep);linearizationSeconds=toc(timer);objectiveSeconds=toc(phase);timer=tic;
persistent options
if isempty(options),options=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',100,'ConstraintTolerance',1e-7,'OptimalityTolerance',1e-6);end
[delta,~,flag,output]=quadprog((Q+Q')/2,f,A,b,E,-e,-ones(nv,1),ones(nv,1),zeros(nv,1),options);
info=struct('linearization_seconds',linearizationSeconds,'solver_seconds',toc(timer),'exitflag',flag, ...
    'iterations',output.iterations,'status',sprintf('quadprog_flag_%d',flag),'solver_success',flag>0, ...
    'variables',nv,'inequalities',numel(b),'original_inequalities',originalCount,'finite',~isempty(delta)&&all(isfinite(delta)), ...
    'phases',struct('nodes',nodeSeconds,'stage_derivatives',stageSeconds,'condensing',condensingSeconds,'objective_constraints',objectiveSeconds));
proposed=U;
if info.finite,rollout=ch5ReducedFeedbackRollout(r);proposed=full(rollout(x,nodes,U,K,reshape(delta,6,nb)));end
end
function out=flatten(pages)
out=reshape(permute(pages,[1,3,2]),[],size(pages,2));
end
