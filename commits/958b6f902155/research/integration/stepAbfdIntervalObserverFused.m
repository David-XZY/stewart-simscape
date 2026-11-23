function [corrected,Pnext,predicted]=stepAbfdIntervalObserverFused(posterior,P,pose,forceImpulse,provider,dt)
% Candidate execution kernel: same prediction/correction and all stage guards.
% The deployed stepAbfdIntervalObserver remains the comparison implementation.
persistent cachedHash cachedStep
validateattributes(posterior,{'double'},{'size',[12,1],'real','finite'});
validateattributes(P,{'double'},{'size',[12,12],'real','finite'});
validateattributes(pose,{'double'},{'size',[6,1],'real','finite'});
validateattributes(forceImpulse,{'double'},{'size',[6,1],'real','finite'});
validateattributes(dt,{'double'},{'scalar','real','finite','nonnegative'});
assert(norm(P-P.','fro')<1e-10 && min(eig((P+P.')/2))>=-1e-12, ...
    'ABFD:ObserverCovariance','Invalid observer posterior covariance.');
predicted=posterior; Ppred=P;
if dt==0
    assert(norm(forceImpulse,inf)<1e-12,'ABFD:ObserverImpulseClock','A zero interval cannot contain force impulse.');
else
    if isempty(cachedHash) || ~strcmp(cachedHash,provider.hash)
        cachedStep=buildAbfdIntervalObserverKernel(provider); cachedHash=provider.hash;
    end
    [value,transition,stages,mass,transmission,rate]=cachedStep(posterior,forceImpulse/dt,dt);
    predicted=full(value); transition=full(transition); stages=full(stages);
    mass=full(mass); transmission=full(transmission); rate=full(rate);
    assert(all(isfinite([predicted;transition(:);stages(:)])),'ABFD:ObserverNonfinite','Nonfinite observer prediction or derivative.');
    for k=1:size(stages,2)
        E=rate(:,3*(k-1)+(1:3)); H=mass(:,6*(k-1)+(1:6)); Jv=transmission(:,6*(k-1)+(1:6));
        assert(rcond(E)>1e-5 && rcond(H)>provider.model.num.rcondMin && ...
            rcond(Jv)>provider.model.num.rcondMin, ...
            'ABFD:ObserverSingular','Ill-conditioned observer integration stage.');
    end
    Ppred=transition*P*transition.'+dt*1e-6*eye(12); Ppred=(Ppred+Ppred.')/2;
end
[corrected,~,Pnext]=stepAbfdExternalObserver(predicted,Ppred,pose,zeros(6,1),provider,0);
end
