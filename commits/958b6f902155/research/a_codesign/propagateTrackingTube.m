function tube=propagateTrackingTube(Acl,G,Sigma,P0,Qd)
% Sigma is per-step world-wrench covariance; Qd is discrete state covariance.
N=size(Acl,3); P=zeros(12,12,N+1); P(:,:,1)=P0;
for k=1:N
    P(:,:,k+1)=Acl(:,:,k)*P(:,:,k)*Acl(:,:,k).'+ ...
        G(:,:,k)*Sigma(:,:,min(k,size(Sigma,3)))*G(:,:,k).'+Qd(:,:,min(k,size(Qd,3)));
    P(:,:,k+1)=(P(:,:,k+1)+P(:,:,k+1).')/2;
    assert(all(isfinite(P(:,:,k+1)),'all') && min(eig(P(:,:,k+1)))>=-1e-8, ...
        'ABFD:TubeCovariance','Invalid propagated covariance.');
end
tube=struct('P',P,'sigma',sqrt(max(0,reshape(diagPages(P),12,N+1))));
end
function d=diagPages(P)
d=zeros(12,size(P,3)); for k=1:size(P,3), d(:,k)=diag(P(:,:,k)); end
end
