function [corrected,next,Pnext]=stepAbfdExternalObserver(prior,P,pose,F,provider,dt)
% A single external EKF; pose correction then actual-force prediction.
C=[eye(6),zeros(6)]; measurementCov=diag([1e-8*ones(1,3),1e-8*ones(1,3)]);
innovation=pose-prior(1:6);
R=rpy2rotmZYX(prior(4:6)); Rm=rpy2rotmZYX(pose(4:6));
E=rpyRateMapZYX(prior(4:6)); assert(rcond(E)>1e-5,'ABFD:ObserverSingular','Singular observer coordinates.');
innovation(4:6)=E\rotationErrorSO3(Rm,R);
gain=P*C.'/(C*P*C.'+measurementCov); corrected=prior+gain*innovation;
Pc=(eye(12)-gain*C)*P*(eye(12)-gain*C).'+gain*measurementCov*gain.';
if dt==0, next=corrected; Pnext=Pc; return; end
persistent cachedHash cachedAd
if isempty(cachedHash) || ~strcmp(cachedHash,provider.hash)
    cachedAd=buildCasadiCompositeDynamics(provider); cachedHash=provider.hash;
end
[A,~]=cachedAd.linearize(corrected,F);
transition=eye(12)+dt*full(A);
next=corrected+dt*provider.forward(corrected,F);
Pnext=transition*Pc*transition.'+dt*1e-6*eye(12); Pnext=(Pnext+Pnext.')/2;
end
