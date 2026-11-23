function feature=pgModeFeatures(bundle,task)
% Align geometry, not time. Cylinder axis spin does not create a new mode.
tr=bdReconstruct(bundle,linspace(0,task.T,513)); Q=tr.q; points=size(Q,2); center=zeros(3,points); orient=zeros(9,points);
for k=1:points
    R=rpy2rotmZYX(Q(4:6,k)); center(:,k)=Q(1:3,k)+R*[0;0;.15];
    if strcmp(task.payload.shape,'cylinder'), orient(1:3,k)=R(:,1); else, orient(:,k)=R(:)/sqrt(2); end
end
p=pgPhysics(task.model); L=full(p.point(Q,zeros(size(Q)),zeros(size(Q))));
signals=[center/.1;orient/(5*pi/180);L/.05]; ds=sqrt(sum(diff(signals,1,2).^2,1)); progress=[0,cumsum(ds)];
if progress(end)<1e-12, aligned=repmat(signals(:,1),1,33);
else
    [u,index]=unique(progress/progress(end),'stable'); aligned=interp1(u,signals(:,index).',linspace(0,1,33),'linear').';
end
feature=aligned(:); assert(all(isfinite(feature)),'PG:Mode','Nonfinite mode features.');
end
