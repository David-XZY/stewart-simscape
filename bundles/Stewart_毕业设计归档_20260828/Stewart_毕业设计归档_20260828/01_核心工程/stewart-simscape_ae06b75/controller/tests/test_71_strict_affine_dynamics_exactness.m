function test_71_strict_affine_dynamics_exactness
% Exact affine map must reproduce the composite-rigid-body dynamics.
[model, scene, controllerRoot] = testEnvironment();
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));
config = makeStrictClfCbfQpConfig(model);

states = [model.qHome, ...
    model.qHome+[0.015; -0.010; 0.020; 0.02; -0.015; 0.01], ...
    scene.qWaypoint];
rates = [zeros(6, 1), ...
    [0.04; -0.03; 0.02; 0.015; -0.012; 0.01], ...
    [0.02; 0.01; -0.015; -0.01; 0.008; 0.012]];
forces = [250*ones(6, 1), [180; 240; 300; 220; 260; 200], ...
    [320; 260; 210; 280; 230; 300]];

for index = 1:size(states, 2)
    q = states(:, index);
    qd = rates(:, index);
    force = forces(:, index);
    exact = evaluateStrictClfCbfAdModel(q, qd, model, scene, config);
    [xdot, auxiliary] = stateDynamicsCompositeRigidBody([q; qd], force, model);
    qddAffine = exact.drift + exact.inputMap*force;
    assert(max(abs(qddAffine-xdot(7:12))) < 2e-7);
    assert(max(abs(exact.H-auxiliary.H), [], 'all') < 2e-7);
    assert(max(abs(exact.rigidBias-auxiliary.Wbias)) < 2e-7);

    leg = computeLegKinematics(q, qd, qddAffine, model);
    legAffine = exact.legAccelerationDrift + exact.legAccelerationMap*force;
    assert(max(abs(legAffine-leg.Ldd)) < 3e-7);
    assert(exact.derivativeBackend == "CasADi algorithmic differentiation");
    assert(~exact.usesFiniteDifferences);
end
end

function [model, scene, controllerRoot] = testEnvironment()
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
end
