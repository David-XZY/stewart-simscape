function d=pgDistance(task,a)
assert(size(a.trainX,2)==99,'PG:Schema','Retrieval requires full 99-dimensional load conditions.');
x=task.context(:).'; z=(a.trainX-x)./a.inputScale;
pos=mean(z(:,[1:3,7:9]).^2,2); rot=zeros(size(pos));
for k=1:numel(a.library)
    old=a.library{k}.task;
    for pair={[task.q0,old.q0],[task.qGoal,old.qGoal]}
        q=pair{1}; R=rpy2rotmZYX(q(4:6,1)).'*rpy2rotmZYX(q(4:6,2)); rot(k)=rot(k)+acos(max(-1,min(1,(trace(R)-1)/2)))^2;
    end
end
rot=rot/max(2*mean(a.inputScale([4:6,10:12]).^2),1e-6);
obstacle=mean(z(:,16:93).^2,2); duration=log(task.T./a.trainX(:,13)).^2/max(var(log(a.trainX(:,13)),1),1e-6);
shape=any(a.trainX(:,94:95)~=x(94:95),2); dims=mean(z(:,96:98).^2,2);
mass=log(task.payload.mass./a.trainX(:,99)).^2/max(var(log(a.trainX(:,99)),1),1e-6);
w=ones(7,1);
switch a.distanceVariant
    case 'equal_blocks'
    case 'shape_x2', w(5)=2;
    case 'dimensions_x2', w(6)=2;
    case 'mass_x2', w(7)=2;
    case 'obstacle_x2', w(3)=2;
    case 'time_x2', w(4)=2;
    otherwise, error('PG:Distance','Unregistered distance configuration.');
end
d=[pos,rot,obstacle,duration,double(shape),dims,mass]*w;
if isfield(a,'sameShapeOnly')&&a.sameShapeOnly, d(shape)=inf; end
if isfield(a,'signatureDistance')&&a.signatureDistance
    assert(isfield(a,'trainSignature')&&isfield(a,'signatureScale'),'PG:Signature','Train-only signature scales required.');
    if isfield(a,'callSignature'), s=a.callSignature(:).'; else, s=pgPhysicsSignature(task).'; end
    d=d+mean(((a.trainSignature-s)./a.signatureScale).^2,2);
end
end
