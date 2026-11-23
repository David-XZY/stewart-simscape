function report=testTemporalInitialObservation()
% Closed-form Gaussian conditioning oracle with a correlated latent history.
I=eye(6); P=[2*I,.4*I;.4*I,3*I]; mu=(1:6).'/100;
t=struct('initialCovariance',P,'standardizedMean',mu);
prior=initializeAbfdTemporalState(t);
assert(isequal(prior.covariance,P) && ~any(prior.centeredMean), ...
    'ABFD:TemporalPrior','Legacy stationary initialization changed.');
t.conditioning=struct('kind','independent-initial-static-balance-v1');
s=initializeAbfdTemporalState(t);
expectedP=blkdiag(zeros(6),2.92*I); expectedMean=[-mu;-.2*mu];
assert(norm(s.covariance-expectedP,'fro')<1e-12 && norm(s.centeredMean-expectedMean)<1e-12, ...
    'ABFD:TemporalConditionalIdentity','Conditioning did not preserve the unobserved history uncertainty.');
assert(norm(s.covariance+P(:,1:6)/P(1:6,1:6)*P(1:6,:)-P,'fro')<1e-12, ...
    'ABFD:TemporalTotalCovariance','Conditional and explained covariance do not reconstruct the prior.');
report=struct('passed',true,'meanError',norm(s.centeredMean-expectedMean), ...
    'covarianceError',norm(s.covariance-expectedP,'fro'),'unobservedHistoryVariance',2.92, ...
    'scope','Gaussian conditioning identity; no experimental coverage claim');
end
