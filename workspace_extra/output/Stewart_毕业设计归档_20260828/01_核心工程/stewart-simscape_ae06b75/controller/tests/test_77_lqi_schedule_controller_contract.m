function test_77_lqi_schedule_controller_contract
% test_77_lqi_schedule_controller_contract - Constant reference degeneracy.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));

model = buildOptModelCustom();
config = makeSimscapePoseForceConfig(model, struct('lqiControlScale', 10));
q = model.qHome(:);
force = inverseDynamicsCompositeRigidBody(q, zeros(6, 1), zeros(6, 1), model);
reference = struct('time', [0, 0.01, 0.02], ...
    'q', repmat(q, 1, 3), 'qd', zeros(6, 3), ...
    'computedForce', repmat(force, 1, 3));

fixed = buildReferenceScheduledLqi(reference, model, config, 'mode', 'fixed');
scheduled = buildReferenceScheduledLqi(reference, model, config, 'mode', 'scheduled');
assert(fixed.allStable && scheduled.allStable);
for index = 1:numel(reference.time)
    assert(norm(fixed.samples(index).controllerA-scheduled.samples(index).controllerA, 'fro') < 1e-8);
    assert(norm(fixed.samples(index).controllerB-scheduled.samples(index).controllerB, 'fro') < 1e-8);
    assert(norm(fixed.samples(index).controllerC-scheduled.samples(index).controllerC, 'fro') < 1e-8);
end
end
