function test_82_smooth_reference_recomputation
% Smooth q/qd/qdd and synchronized leg/feedforward references.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'simscape_tracking'));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
model = buildOptModelCustom();
poseConfig = makeSimscapePoseForceConfig(model, struct('derivativeSampleTime', 0.01));
time = 0:0.01:7.5;
count = numel(time);
q = repmat(model.qHome(:), 1, count);
qd = zeros(6, count); qdd = qd;
force = inverseDynamicsCompositeRigidBody(model.qHome, zeros(6, 1), zeros(6, 1), model);
lengthHome = sgpIK(model.qHome, model).L;
nominal = struct('time', time, 'q', q, 'qd', qd, 'qdd', qdd, ...
    'computedForce', repmat(force, 1, count), ...
    'legLength', repmat(lengthHome, 1, count), 'legSpeed', zeros(6, count), ...
    'targetPerturbation', zeros(6, count), 'feedforwardPolicy', "nominal");
config = makeDisturbanceComparisonConfig();
cases = buildDisturbanceCaseMatrix(config, 'all');
item = cases([cases.disturbanceType] == "smooth_target" & [cases.scale] == 1);
reference = buildDisturbedReference(nominal, item, model, poseConfig, config);
outside = time < 4.5 | time > 5.3;
assert(max(abs(reference.q(4:6, outside)-nominal.q(4:6, outside)), [], 'all') < 1e-14);
assert(max(abs(reference.qd(:, [1, end])), [], 'all') < 1e-14);
assert(max(abs(reference.qdd(:, [1, end])), [], 'all') < 1e-14);
assert(max(abs(reference.legLength(:, 1)-lengthHome)) < 1e-12);
assert(any(abs(reference.computedForce(:)-nominal.computedForce(:)) > 1e-8));
fprintf('test_82_smooth_reference_recomputation passed.\n');
end
