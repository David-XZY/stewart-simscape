function refs=reconstructIhsidContinuousReference(traj,provider,dt)
% C2 quintic execution reference through HS nodes AND midpoints, not HS cubic.
required={'t','tc','Xnode','Xmid','Anode','Amid'};
for k=1:numel(required)
    assert(isfield(traj,required{k}),'ABFD:MissingHsData','Full HS data required: %s',required{k});
end
knots=[traj.t(:);traj.tc(:)];
[knots,order]=sort(knots);
assert(all(diff(knots)>0),'ABFD:ReferenceTime','HS times must be unique.');
x=[traj.Xnode,traj.Xmid]; x=x(:,order);
a=[traj.Anode,traj.Amid]; a=a(:,order);
assert(size(x,1)==12 && size(a,1)==6 && size(a,2)==numel(knots) && ...
    all(isfinite([x(:);a(:)])),'ABFD:ReferenceShape','Invalid HS arrays.');
t=unique([knots; (knots(1):dt:knots(end)).']);
Q=zeros(6,numel(t)); V=Q; A=Q;
for k=1:numel(knots)-1
    h=knots(k+1)-knots(k); ids=find(t>=knots(k) & t<=knots(k+1));
    s=(t(ids)-knots(k)).'/h;
    c0=x(1:6,k); c1=h*x(7:12,k); c2=h^2*a(:,k)/2;
    rhs=[x(1:6,k+1)-c0-c1-c2, h*x(7:12,k+1)-c1-2*c2, h^2*a(:,k+1)-2*c2];
    tail=rhs/[1,3,6;1,4,12;1,5,20];
    c=[c0,c1,c2,tail];
    Q(:,ids)=c*[ones(size(s));s;s.^2;s.^3;s.^4;s.^5];
    V(:,ids)=c(:,2:6)*[ones(size(s));2*s;3*s.^2;4*s.^3;5*s.^4]/h;
    A(:,ids)=c(:,3:6)*[2*ones(size(s));6*s;12*s.^2;20*s.^3]/h^2;
end
F=zeros(size(Q)); L=F; Ld=F; Ldd=F; residual=F;
for k=1:numel(t)
    o=provider.evaluate(Q(:,k),V(:,k),A(:,k)); F(:,k)=o.Jv.'\o.W;
    kin=computeLegKinematics(Q(:,k),V(:,k),A(:,k),provider.model);
    L(:,k)=kin.L; Ld(:,k)=kin.Ld; Ldd(:,k)=kin.Ldd;
    residual(:,k)=o.Jv.'*F(:,k)-o.W;
end
refs=struct('schemaVersion',1,'t',t.','q',Q,'qd',V,'qdd',A, ...
    'Fleg',F,'Fcomputed',F,'L',L,'Ld',Ld,'Ldd',Ldd,'q0',Q(:,1), ...
    'modelHash',provider.hash,'referenceInterpolation','hs-data-piecewise-quintic-C2', ...
    'maxDynamicsResidual',max(abs(residual),[],'all'),'hs',traj);
end
