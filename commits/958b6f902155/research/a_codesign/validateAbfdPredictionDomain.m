function report=validateAbfdPredictionDomain(meanState,provider)
% Numeric postcheck of the predicted center; never repair a singular solve.
validateattributes(meanState,{'double'},{'2d','nrows',12,'real','finite','nonempty'});
assert(all(abs(cos(meanState(5,:)))>1e-5),'ABFD:PredictionInvalid','Predicted mean reaches an attitude singularity.');
minH=inf; minJ=inf; minSigma=inf;
for k=1:size(meanState,2)
    q=meanState(1:6,k); v=meanState(7:12,k);
    physical=provider.evaluate(q,v,zeros(6,1)); J=sgpJacobian(q,provider.model);
    minH=min(minH,rcond(physical.H)); minJ=min(minJ,rcond(physical.Jv)); minSigma=min(minSigma,J.sigmaMin);
end
assert(minH>provider.model.num.rcondMin && minJ>provider.model.num.rcondMin, ...
    'ABFD:PredictionSingular','Predicted mean is dynamically or transmission singular.');
violation=max(0,provider.model.singularity.sigmaMinSafe-minSigma);
report=struct('passed',violation==0,'maxIneq',violation,'minRcondH',minH,'minRcondJ',minJ, ...
    'minSigma',minSigma,'samples',size(meanState,2));
end
