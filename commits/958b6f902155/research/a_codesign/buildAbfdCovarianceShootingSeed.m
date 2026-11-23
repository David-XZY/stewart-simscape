function [z,report]=buildAbfdCovarianceShootingSeed(nlp,physical)
% Solve each covariance continuity equation in chronological order. Never
% regularize or reset a predicted covariance to make its factorization pass.
import casadi.*
assert(isfield(nlp.tube,'lifting') && numel(physical)==nlp.physicalNumZ,'ABFD:CovarianceSeedLayout');
lifting=nlp.tube.lifting; z=[physical(:);zeros(numel(lifting.variables),1)]; timer=tic;
if isfield(lifting,'kind') && strcmp(lifting.kind,'actuator-moment-shooting-v1')
    for k=1:numel(lifting.cutTimes)
        evaluate=Function('abfd_actuator_moment_seed_segment',{nlp.z},lifting.predictions(k));
        predicted=full(evaluate(z));
        assert(numel(predicted)==lifting.factorSize && all(isfinite(predicted)), ...
            'ABFD:MomentSeedFinite','The full propagated moment seed is invalid.');
        ids=nlp.physicalNumZ+(k-1)*lifting.factorSize+(1:lifting.factorSize);
        z(ids)=predicted;
    end
    check=Function('abfd_actuator_moment_continuity_seed',{nlp.z},{lifting.equalities});
    residual=norm(full(check(z)),inf);
    assert(residual<1e-9,'ABFD:CovarianceSeedContinuity','Full moment continuity seed failed.');
    report=struct('kind',lifting.kind,'cuts',numel(lifting.cutTimes), ...
        'auxiliaryVariables',numel(lifting.variables),'seconds',toc(timer), ...
        'maxNormalizedContinuityResidual',residual, ...
        'scope','Full actuator/VAR moment seed from chronological propagation; no covariance inflation, projection or reset');
    return;
end
minDiagonal=inf; maxSymmetry=0;
for k=1:numel(lifting.cutTimes)
    evaluate=Function('abfd_covariance_seed_segment',{nlp.z},lifting.predictions(k));
    P=full(evaluate(z)); normalized=lifting.scale\P/lifting.scale;
    symmetry=norm(normalized-normalized.','fro')/max(1,norm(normalized,'fro'));
    assert(symmetry<1e-10,'ABFD:CovarianceSeedSymmetry');
    [L,flag]=chol((normalized+normalized.')/2,'lower');
    assert(flag==0,'ABFD:CovarianceSeedFactorization','Predicted covariance has no positive definite seed factor.');
    ids=nlp.physicalNumZ+(k-1)*lifting.factorSize+(1:lifting.factorSize);
    if isfield(lifting,'kind') && strcmp(lifting.kind,'sampled-symmetric-shooting-v1')
        z(ids)=normalized(lifting.lowerIndices);
    else
        z(ids)=L(lifting.lowerIndices);
    end
    minDiagonal=min(minDiagonal,min(diag(L))); maxSymmetry=max(maxSymmetry,symmetry);
end
check=Function('abfd_covariance_continuity_seed',{nlp.z},{lifting.equalities}); residual=max(abs(full(check(z))));
assert(residual<1e-9,'ABFD:CovarianceSeedContinuity');
report=struct('cuts',numel(lifting.cutTimes),'auxiliaryVariables',numel(lifting.variables), ...
    'seconds',toc(timer),'maxNormalizedContinuityResidual',residual,'minimumFactorDiagonal',minDiagonal, ...
    'maxSymmetryRelativeError',maxSymmetry,'scope','Exact covariance continuity seed with no covariance inflation or reset');
end
