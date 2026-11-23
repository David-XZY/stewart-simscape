function e=p474Metrics(b,task,c)
t=linspace(0,task.T,ceil(task.T/.005)+1);tr=bdReconstruct(b,t);p=pgPhysics(task.model);m=task.model;
[L,V,A,F,J]=p.point(tr.q,tr.v,tr.a);L=full(L);V=full(V);A=full(A);F=full(F);J=full(J);
g=pgClearanceBatch(tr.q,task);coverage=bdRobotEnvelope(task,tr.q,c);g=[g;coverage.gaps];
if isempty(g),gap=[];gapTime=[];else,[gap,index]=min(g(:));[~,col]=ind2sub(size(g),index);gapTime=t(col);end
sigma=inf;condition=0;
for k=1:numel(t),s=svd(J(:,(k-1)*6+(1:6))*diag([1,1,1,2,2,2]));sigma=min(sigma,min(s));condition=max(condition,max(s)/min(s));end
boundary=max(abs([tr.q(:,1)-task.q0;tr.q(:,end)-task.qGoal;reshape(tr.v(:,[1,end]),[],1);reshape(tr.a(:,[1,end]),[],1)]));
e=struct('minClearanceLowerBound',gap,'clearanceTime',gapTime,'minSigma',sigma,'maxCondition',condition,'boundaryError',boundary, ...
    'lengthViolation',max([0;reshape(m.lmin-L,[],1);reshape(L-m.lmax,[],1)]), ...
    'speedViolation',max([0;reshape(abs(V)-m.actuator.ldotMax(:),[],1)]),'accelerationViolation',max([0;reshape(abs(A)-m.actuator.lddotMax(:),[],1)]), ...
    'forceViolation',max([0;reshape(m.actuator.forceMin(:)-F,[],1);reshape(F-m.actuator.forceMax(:),[],1)]),'sampleCount',numel(t));
end
