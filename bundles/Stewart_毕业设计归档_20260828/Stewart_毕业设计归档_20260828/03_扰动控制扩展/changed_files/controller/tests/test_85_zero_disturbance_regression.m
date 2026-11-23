function test_85_zero_disturbance_regression
% New exact harness must match the previous fixed-LQI path within 1 percent.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'simscape_tracking'));
addpath(fullfile(projectRoot, 'controller', 'lqi_schedule_comparison'));
addpath(fullfile(projectRoot, 'controller', 'strict_clf_cbf_qp'));
addpath(fullfile(projectRoot, 'controller', 'exact_model_study'));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
poseConfig = makeSimscapePoseForceConfig(model, struct('derivativeSampleTime', 0.01));
time = 0:0.01:0.20; count = numel(time);
force = inverseDynamicsCompositeRigidBody(model.qHome, zeros(6, 1), zeros(6, 1), model);
lengthHome = sgpIK(model.qHome, model).L;
reference = struct('time', time, 'q', repmat(model.qHome, 1, count), ...
    'qd', zeros(6, count), 'qdd', zeros(6, count), ...
    'computedForce', repmat(force, 1, count), ...
    'legLength', repmat(lengthHome, 1, count), 'legSpeed', zeros(6, count), ...
    'targetPerturbation', zeros(6, count), 'feedforwardPolicy', "nominal");
schedule = buildReferenceScheduledLqi(reference, model, poseConfig, 'mode', 'fixed');
scenario = struct('id', "nominal", 'displayName', "nominal", ...
    'plantModel', model, 'actuatorLag', 0);
old = simulateLqiScheduleScenario(reference, schedule, poseConfig, scenario, ...
    'integrationSubsteps', 2);
config = makeDisturbanceComparisonConfig(struct('integrationSubsteps', 2, ...
    'screeningSkipCollision', true));
experimentCase = buildDisturbanceCaseMatrix(config, 'all');
experimentCase = experimentCase(1);
strictConfig = makeStrictClfCbfQpConfig(model, struct('dt', 0.01));
dob = makeDobConfig(0.01, 5, 120, 2000);
controller = struct('id', "fixed_lqi", 'displayName', "fixed", ...
    'schedule', schedule, 'useDob', false, 'useStrictQp', false, ...
    'dobConfig', dob, 'filterScene', scene);
new = simulateDisturbanceControlScenario(reference, model, scene, poseConfig, ...
    strictConfig, experimentCase, controller, config);
relative = norm(new.control.state-old.control.state, 'fro')/ ...
    max(norm(old.control.state, 'fro'), eps);
assert(relative <= 0.01, 'Zero-disturbance regression exceeded 1%%.');
fprintf('test_85_zero_disturbance_regression passed (relative %.3g).\n', relative);
end
