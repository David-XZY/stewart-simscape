function schedule=buildTransmissionWeightedTvlqr(refs,provider,config)
N=numel(refs.t)-1; A=zeros(12,12,N); B=zeros(12,6,N); R=zeros(6,6,N);
for k=1:N
    dt=refs.t(k+1)-refs.t(k);
    omega=rpyRateToSpatialVelocity(refs.q(4:6,k),refs.qd(4:6,k));
    Ac=[zeros(6),eye(6);zeros(6,12)];
    Ac(4:6,4:6)=[0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
    Z=expm([Ac,[zeros(6);eye(6)];zeros(6,18)]*dt);
    A(:,:,k)=Z(1:12,1:12); B(:,:,k)=Z(1:12,13:18);
    o=provider.evaluate(refs.q(:,k),refs.qd(:,k),refs.qdd(:,k));
    [~,E]=abfdRotationExpressions(refs.q(4:6,k),refs.qd(4:6,k));
    spatialMap=blkdiag(eye(3),E\eye(3));
    gamma=o.Gamma*spatialMap;
    R(:,:,k)=(gamma.'*config.RF*gamma)*dt+1e-9*eye(6);
end
Q=repmat(config.Q,1,1,N);
for k=1:N, Q(:,:,k)=Q(:,:,k)*(refs.t(k+1)-refs.t(k)); end
schedule=buildFiniteHorizonFullStateLqr(A,B,Q,R,config.Q);
schedule.t=refs.t; schedule.variant='tvlqr'; schedule.providerHash=provider.hash;
end
