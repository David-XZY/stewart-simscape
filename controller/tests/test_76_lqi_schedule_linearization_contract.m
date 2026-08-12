function test_76_lqi_schedule_linearization_contract
% test_76_lqi_schedule_linearization_contract - Exact LQI linearization checks.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));

model = buildOptModelCustom();
q = model.qHome(:);
qd = zeros(6, 1);
force = inverseDynamicsCompositeRigidBody(q, qd, zeros(6, 1), model);
linearization = linearizeCompositeRigidBodyLqi(q, qd, force, model);

assert(isequal(size(linearization.A), [12, 12]));
assert(isequal(size(linearization.B), [12, 6]));
assert(linearization.inputMapRelativeError < 1e-8);
assert(isfinite(linearization.aConvergenceRelative));
assert(linearization.aConvergenceRelative < 1e-3);
assert(rank(linearization.B(7:12, :)) == 6);
end
