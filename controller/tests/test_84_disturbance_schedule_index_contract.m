function test_84_disturbance_schedule_index_contract
% Fixed versus scheduled lookup contract.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'simscape_tracking'));
addpath(fullfile(projectRoot, 'controller', 'lqi_schedule_comparison'));
model = buildOptModelCustom();
config = makeSimscapePoseForceConfig(model, struct('derivativeSampleTime', 0.01));
time = 0:0.01:0.02;
force = inverseDynamicsCompositeRigidBody(model.qHome, zeros(6, 1), zeros(6, 1), model);
reference = struct('time', time, 'q', repmat(model.qHome, 1, 3), ...
    'qd', zeros(6, 3), 'computedForce', repmat(force, 1, 3));
fixed = buildReferenceScheduledLqi(reference, model, config, 'mode', 'fixed');
scheduled = buildReferenceScheduledLqi(reference, model, config, 'mode', 'scheduled');
assert(isequal(fixed.referenceIndex, ones(3, 1)));
assert(isequal(scheduled.referenceIndex, (1:3).'));
assert(fixed.sampleCount == 3 && scheduled.sampleCount == 3);
fprintf('test_84_disturbance_schedule_index_contract passed.\n');
end
