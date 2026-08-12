function test_83_dob_sign_and_limits
% Residual sign, opposing compensation, amplitude, and rate limit.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
model = buildOptModelCustom();
q = model.qHome(:); qd = zeros(6, 1);
force = inverseDynamicsCompositeRigidBody(q, qd, zeros(6, 1), model);
[~, aux] = stateDynamicsCompositeRigidBody([q; qd], force, model);
wrench = [100; -80; 60; 6; -5; 4];
measuredAcceleration = aux.H\(aux.Wact+wrench-aux.Wbias);
dob = makeDobConfig(0.01, 5, 60, 1000);
state = initializeDisturbanceObserver();
[~, state] = stepDisturbanceObserver(q, qd, measuredAcceleration, force, model, dob, state);
[compensation, ~, diagnostic] = stepDisturbanceObserver( ...
    q, qd, measuredAcceleration, force, model, dob, state);
mapped = sgpJacobian(q, model).Jv.'*compensation;
assert(dot(mapped, wrench) < 0, 'DOB compensation must oppose the disturbance.');
assert(max(abs(compensation)) <= 10+1e-12, 'First active step must obey 1000 N/s.');
assert(max(abs(diagnostic.wrenchEstimate)) > 0);
assert(max(abs(compensation)) <= 60+1e-12);
fprintf('test_83_dob_sign_and_limits passed.\n');
end
