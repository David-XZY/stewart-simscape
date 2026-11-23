function test_80_current_state_observer_contract
% Observer rank, stability, wrapped innovation, and convergence contract.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));

model = buildOptModelCustom();
config = makeSimscapePoseForceConfig(model);
q = model.qHome(:);
qd = zeros(6, 1);
force = inverseDynamicsCompositeRigidBody(q, qd, zeros(6, 1), model);
observer = designCurrentStateLuenbergerObserver(q, qd, force, ...
    model, config, 0.01);
assert(observer.observabilityRank == 12);
assert(all(real(observer.errorPoles) < -1e-7));
assert(~observer.usesVelocityMeasurement);
assert(~observer.usesAppliedForceTruth);

measured = q;
estimated = q;
measured(6) = pi-1e-6;
estimated(6) = -pi+1e-6;
innovation = computePoseInnovation(measured, estimated);
assert(abs(innovation(6)+2e-6) < 1e-10, ...
    'RPY innovation is discontinuous across +/-pi.');

initialError = [0.5e-3; -0.4e-3; 0.3e-3; ...
    deg2rad([0.03; -0.02; 0.025]); 0.002*ones(6, 1)];
estimate = [q; qd]+initialError;
for index = 1:50
    estimate = stepCurrentStateLuenbergerObserver( ...
        estimate, q, force, model, observer, 0.01);
end
finalError = estimate-[q; qd];
assert(norm(finalError) <= 0.1*norm(initialError), ...
    'Observer state error did not decay by 90 percent within 0.5 s.');
assert(sqrt(mean(finalError(1:3).^2)) <= 0.05e-3);
assert(sqrt(mean(finalError(4:6).^2)) <= deg2rad(0.01));

didThrow = false;
try
    stepCurrentStateLuenbergerObserver(estimate, ...
        [q(1:5); NaN], force, model, observer, 0.01);
catch
    didThrow = true;
end
assert(didThrow, 'A nonfinite observer measurement must terminate the step.');
end
