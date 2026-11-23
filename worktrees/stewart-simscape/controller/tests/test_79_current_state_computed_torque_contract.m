function test_79_current_state_computed_torque_contract
% Current-state CT equals legacy on-reference and responds off-reference.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
config = makeSimscapePoseForceConfig(model);
q = model.qHome(:)+[1e-3; -0.5e-3; 0.8e-3; deg2rad([0.1; -0.05; 0.08])];
qd = [1e-3; -2e-3; 1.5e-3; deg2rad([0.2; -0.1; 0.15])];
qdd = [0.01; -0.02; 0.015; deg2rad([0.5; -0.3; 0.4])];

legacyConfig = config;
legacyConfig.computedTorqueStateMode = 'reference';
observedConfig = config;
observedConfig.computedTorqueStateMode = 'observed-current-state';
legacy = computeComputedTorqueForce(q, qd, qdd, q, qd, ...
    model, legacyConfig);
sameState = computeComputedTorqueForce(q, qd, qdd, q, qd, ...
    model, observedConfig);
assert(norm(legacy-sameState, inf) <= 1e-10*max(1, norm(legacy, inf)));

qhat = q+[0.7e-3; -0.4e-3; 0.5e-3; deg2rad([0.04; -0.03; 0.02])];
qdhat = qd+[2e-3; -1e-3; 1.5e-3; deg2rad([0.1; -0.05; 0.08])];
changed = computeComputedTorqueForce(q, qd, qdd, qhat, qdhat, ...
    model, observedConfig);
assert(norm(changed-sameState, inf) > 1e-5, ...
    'Current-state computed torque did not respond to qhat/qdot changes.');
end
