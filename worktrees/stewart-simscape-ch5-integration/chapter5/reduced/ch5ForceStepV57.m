function [proposed,qsNew,info]=ch5ForceStepV57(x,previous,U,goal,o,prior,r,k,time,qs)
% 48 feedback correction variables + six feasible steady-pose variables.
% Existing endpoint physics sensitivities are reused, then new exact audits decide admission.
candidateCount=size(prior,3);firstPrior=prior(:,:,1);
[~,linear]=ch5ReducedStepV50(x,previous,U,goal,o,zeros(size(U)),r,k,time,true);
nodes=linear.nodes;M=linear.M;n=r.plant.n;nv=54;H=r.c.H;starts=[1,6,16,26,36,46,51,56];
[~,aa,bb]=k.stages(nodes(:,1:H),U);aa=full(aa);bb=full(bb);S=zeros(n,48,H+1);
for j=1:H,S(:,:,j+1)=aa(:,n*(j-1)+(1:n))*S(:,:,j)+bb(:,6*(j-1)+(1:6))*M(6*(j-1)+(1:6),:);end
ps=[.1*ones(3,1);deg2rad(10)*ones(3,1)];vs=[.1*ones(3,1);.2*ones(3,1)];Pq=diag(ps);
term=ch5ForceTerminalV57(qs,r,k);e=([nodes(:,end);U(:,end)]-term.xi)./term.scale;
Je=[[S(:,:,end);M(end-5:end,:)],-term.steady_jacobian*Pq]./term.scale;
L=chol(term.P,'lower');Md=M-[zeros(6,48);M(1:end-6,:)];D=diff([previous,U],1,2);
Jq=zeros(6*H,nv);Jg=Jq;Jv=Jq;
for j=1:H
    rows=6*(j-1)+(1:6);Jq(rows,:)=[S(1:6,:,j+1)./ps,-eye(6)];
    Jg(rows,:)=[S(1:6,:,j+1)./ps,zeros(6)];Jv(rows,:)=[S(7:12,:,j+1)./vs,zeros(6)];
end
J=[sqrt(r.c.Ts)*Jq;sqrt(r.c.Ts)*Jv;sqrt(.05*r.c.Ts)*Jg; ...
    sqrt(.01*r.c.Ts)*[Md/160,zeros(360,6)];sqrt(.001*r.c.Ts)*[M/2000,zeros(360,6)]; ...
    sqrt(.05*r.c.Ts)*[M/20,zeros(360,6)];[zeros(6,48),10*eye(6)];L'*Je];
res=[sqrt(r.c.Ts)*reshape((nodes(1:6,2:end)-qs)./ps,[],1);sqrt(r.c.Ts)*reshape(nodes(7:12,2:end)./vs,[],1); ...
    sqrt(.05*r.c.Ts)*reshape((nodes(1:6,2:end)-goal)./ps,[],1);sqrt(.01*r.c.Ts)*D(:)/160; ...
    sqrt(.001*r.c.Ts)*U(:)/2000;sqrt(.05*r.c.Ts)*(U(:)-firstPrior(:))/20;10*(qs-goal)./ps;L'*e];
% Terminal sublevel constraint, rather than the old 12 terminal equalities.
At=2*e'*term.P*Je;bt=term.rho-e'*term.P*e;
linear.Q=2*(J'*J)+1e-6*eye(nv);linear.f=2*J'*res;
linear.A=[[linear.A,zeros(size(linear.A,1),6)];At];linear.b=[linear.b;bt];linear.E=zeros(0,nv);linear.e=zeros(0,1);
baseF=linear.f;proposed=repmat(U,1,1,candidateCount);qsNew=repmat(qs,1,candidateCount);infos=cell(1,candidateCount);
feedback=ch5ReducedFeedbackRolloutV50(r);K=ch5ReducedFeedbackGain(x,r,k);
for candidate=1:candidateCount
    difference=prior(:,:,candidate)-firstPrior;
    linear.f=baseF-2*.05*r.c.Ts*[M/400,zeros(360,6)]'*difference(:);
    [delta,solve]=ch5ReducedSolveV24(linear,r);
    solve.terminal_region_certified=false;solve.terminal_region_rho=term.rho;solve.variables=nv;solve.starts=starts;
    solve.shared_linearization=candidateCount>1;infos{candidate}=solve;
    if solve.finite
        proposed(:,:,candidate)=full(feedback(x,nodes,U,K,reshape(delta(1:48),6,8)));qsNew(:,candidate)=qs+Pq*delta(49:54);
    end
end
if candidateCount==1,info=infos{1};else,info=infos;end
end
