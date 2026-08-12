function evaluation = evaluateStrictClfCbfAdModel(q, qd, model, scene, config)
% evaluateStrictClfCbfAdModel - Numerically evaluate the cached exact maps.
arguments
    q double
    qd double
    model struct
    scene struct
    config struct
end
q = q(:);
qd = qd(:);
validateattributes(q, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(qd, {'double'}, {'real', 'finite', 'size', [6, 1]});

ad = getStrictClfCbfAdModel(model, scene, config);
[drift, inputMap, H, rigidBias, Jv, Jq, Jbar, L, legSpeed, ...
    geometricLegBias, legAccelerationDrift, legAccelerationMap] = ...
    ad.dynamics(q, qd);
[collisionGap, collisionGradient, collisionHessianStack] = ad.collision(q);
[sigmaLowerSquared, singularityBarrier, singularityGradient, ...
    singularityHessian, singularityJbar] = ad.singularity(q);

evaluation = struct();
evaluation.drift = full(drift);
evaluation.inputMap = full(inputMap);
evaluation.H = full(H);
evaluation.rigidBias = full(rigidBias);
evaluation.Jv = full(Jv);
evaluation.Jq = full(Jq);
evaluation.Jbar = full(Jbar);
evaluation.legLength = full(L);
evaluation.legSpeed = full(legSpeed);
evaluation.geometricLegAccelerationBias = full(geometricLegBias);
evaluation.legAccelerationDrift = full(legAccelerationDrift);
evaluation.legAccelerationMap = full(legAccelerationMap);
evaluation.collisionGap = full(collisionGap);
evaluation.collisionGradient = full(collisionGradient);
hessianStack = full(collisionHessianStack);
evaluation.collisionHessian = zeros(6, 6, 3);
for index = 1:3
    evaluation.collisionHessian(:, :, index) = ...
        hessianStack((index-1)*6+(1:6), :);
end
evaluation.sigmaLowerSquared = full(sigmaLowerSquared);
evaluation.singularityBarrier = full(singularityBarrier);
evaluation.singularityGradient = full(singularityGradient);
evaluation.singularityHessian = full(singularityHessian);
evaluation.singularityJbar = full(singularityJbar);
evaluation.derivativeBackend = ad.backend;
evaluation.usesFiniteDifferences = ad.usesFiniteDifferences;
evaluation.collisionNormalBanks = ad.collisionNormalBanks;
end
