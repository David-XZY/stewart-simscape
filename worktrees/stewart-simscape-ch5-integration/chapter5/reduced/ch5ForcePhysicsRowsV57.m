function [J,g]=ch5ForcePhysicsRowsV57(x,U,traj,r,k)
% Exact local derivatives of 11 active mechanical rows plus the terminal sublevel.
% The training penalty uses these only at low noise; nonlinear audits remain decisive.
H=r.c.H;n=r.plant.n;nodes=traj.X;
[~,aa,bb]=k.stages(nodes(:,1:H),U);aa=full(aa);bb=full(bb);S=zeros(n,360,H+1);
for j=1:H
    S(:,:,j+1)=aa(:,n*(j-1)+(1:n))*S(:,:,j);
    S(:,6*(j-1)+(1:6),j+1)=S(:,6*(j-1)+(1:6),j+1)+bb(:,6*(j-1)+(1:6));
end
states=reshape([nodes(:,1:H);nodes(:,2:end)],n,2*H);commands=repelem(U,1,2);
[v,vx,vu]=k.mechanics(states,commands);v=full(v);vx=full(vx);vu=full(vu);
lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];
hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
sc=[ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];hiIdx=find(isfinite(hi));
rows=[(v(hiIdx,:)-hi(hiIdx))./sc(hiIdx);(lo-v)./sc];
[~,order]=sort(rows(:),'descend');order=order(1:11);J=zeros(12,360);g=zeros(12,1);nr=size(rows,1);
for i=1:11
    [ci,stage]=ind2sub([nr,2*H],order(i));j=ceil(stage/2);node=j+mod(stage+1,2);sgn=1;
    if ci>numel(hiIdx),mi=ci-numel(hiIdx);sgn=-1;else,mi=hiIdx(ci);end
    dx=vx(:,n*(stage-1)+(1:n));du=vu(:,6*(stage-1)+(1:6));
    J(i,:)=sgn*dx(mi,:)*S(:,:,node)/sc(mi);
    J(i,6*(j-1)+(1:6))=J(i,6*(j-1)+(1:6))+sgn*du(mi,:)/sc(mi);g(i)=rows(ci,stage);
end
t=traj.terminal;e=([nodes(:,end);U(:,end)]-t.xi)./t.scale;
Je=[S(:,:,end);zeros(6,354),eye(6)]./t.scale;
J(12,:)=2*e'*t.P*Je/max(t.rho,realmin);g(12)=e'*t.P*e/max(t.rho,realmin)-1;
end
