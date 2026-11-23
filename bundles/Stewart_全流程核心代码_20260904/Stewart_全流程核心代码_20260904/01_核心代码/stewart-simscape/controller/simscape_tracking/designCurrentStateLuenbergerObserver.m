function observer = designCurrentStateLuenbergerObserver( ...
        initialPose, initialVelocity, initialForce, model, poseConfig, sampleTime)
% designCurrentStateLuenbergerObserver - Physical [q;qd] observer design.
arguments
    initialPose double
    initialVelocity double
    initialForce double
    model struct
    poseConfig struct
    sampleTime (1, 1) double {mustBePositive, mustBeFinite}
end

initialPose = reshape(initialPose, 6, 1);
initialVelocity = reshape(initialVelocity, 6, 1);
initialForce = reshape(initialForce, 6, 1);
linearization = linearizeCompositeRigidBodyLqi( ...
    initialPose, initialVelocity, initialForce, model);
% Use the same dual-LQR construction as the existing LQI output observer,
% while weighting the unmeasured velocity states strongly enough to meet
% the independent observer's 0.5 s convergence contract. The existing LQI
% controller and its gain are not changed.
velocityStateWeight = 1e4;
observerStateWeight = diag([ones(6, 1); ...
    velocityStateWeight*ones(6, 1)]);
observerOutputWeight = diag((poseConfig.lqiVelocityScale*ones(6, 1)).^2);
internalGain = lqr(linearization.A.', (-linearization.C).', ...
    observerStateWeight, observerOutputWeight).';
gain = -internalGain;
errorDynamics = linearization.A-gain*linearization.C;
errorPoles = eig(errorDynamics);
observabilityRank = rank(obsv(linearization.A, linearization.C));
if observabilityRank ~= 12
    error('designCurrentStateLuenbergerObserver:Unobservable', ...
        'The physical [q;qd] pair is not observable (rank=%d).', ...
        observabilityRank);
end
if any(real(errorPoles) >= -1e-7)
    error('designCurrentStateLuenbergerObserver:Unstable', ...
        'Observer error dynamics are not Hurwitz.');
end

observer = struct();
observer.type = "physical-nonlinear-luenberger";
observer.sampleTime = sampleTime;
observer.gain = gain;
observer.designMethod = "dual LQR at initial physical reference state";
observer.velocityStateWeight = velocityStateWeight;
observer.initialState = [initialPose; initialVelocity];
observer.initialForce = initialForce;
observer.linearization = linearization;
observer.errorPoles = errorPoles;
observer.observabilityRank = observabilityRank;
observer.usesVelocityMeasurement = false;
observer.usesAppliedForceTruth = false;
observer.forceInputDefinition = "previous QP force command";
end
