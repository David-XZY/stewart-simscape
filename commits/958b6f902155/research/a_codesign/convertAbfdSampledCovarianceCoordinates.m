function [converted,report]=convertAbfdSampledCovarianceCoordinates(lifting,physicalCount,z,target)
% Convert every full normalized covariance block; never repair indefiniteness.
target=validatestring(target,{'cholesky','symmetric'});
source='cholesky';
if isfield(lifting,'kind')
    assert(strcmp(lifting.kind,'sampled-symmetric-shooting-v1'), ...
        'ABFD:CovarianceCoordinateKind','Only the complete sampled 12-state covariance is supported.');
    source='symmetric';
end
assert(lifting.factorSize==78 && numel(lifting.lowerIndices)==78 && ...
    numel(z)==physicalCount+78*numel(lifting.cutTimes) && all(isfinite(z)), ...
    'ABFD:CovarianceCoordinateLayout','Invalid sampled covariance coordinate layout.');
converted=z(:); minimumDiagonal=inf; maximumRoundTrip=0;
for k=1:numel(lifting.cutTimes)
    ids=physicalCount+(k-1)*78+(1:78); lower=zeros(12); lower(lifting.lowerIndices)=z(ids);
    if strcmp(source,'symmetric')
        covariance=lower+lower.'-diag(diag(lower));
        [factor,flag]=chol(covariance,'lower');
        assert(flag==0,'ABFD:CovarianceCoordinatePositiveDefinite', ...
            'The complete covariance has no positive definite factor; it is not projected or inflated.');
    else
        factor=lower;
        assert(all(diag(factor)>0),'ABFD:CovarianceCoordinatePositiveDefinite', ...
            'Strictly positive covariance factor diagonals are required for invertible coordinates.');
        covariance=factor*factor.';
    end
    minimumDiagonal=min(minimumDiagonal,min(diag(factor)));
    maximumRoundTrip=max(maximumRoundTrip,norm(covariance-factor*factor.',inf));
    if strcmp(target,'symmetric'), converted(ids)=covariance(lifting.lowerIndices);
    else, converted(ids)=factor(lifting.lowerIndices); end
end
report=struct('passed',true,'source',source,'target',target,'cuts',numel(lifting.cutTimes), ...
    'minimumFactorDiagonal',minimumDiagonal,'maximumCovarianceRoundTrip',maximumRoundTrip, ...
    'physicalDecisionDifference',norm(converted(1:physicalCount)-z(1:physicalCount),inf), ...
    'scope','Complete covariance coordinate conversion and positive-definiteness check; no optimization or trajectory acceptance');
end
