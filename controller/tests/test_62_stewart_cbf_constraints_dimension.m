function test_62_stewart_cbf_constraints_dimension
% test_62_stewart_cbf_constraints_dimension - 验证 CBF/barrier-like 约束维度
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model, struct( ...
    'enableSingularityBarrier', true, ...
    'enableCollisionBarrier', true));
q = model.qHome + [0.001; 0.001; -0.001; 0.0005; 0; -0.0005];
qd = [0.01; 0.02; -0.015; 0.001; 0; -0.001];
Fprev = zeros(6, 1);

cbf = buildStewartCbfConstraints(q, qd, Fprev, model, config);

assert(size(cbf.A, 2) == 6);
assert(size(cbf.A, 1) == numel(cbf.b));
assert(size(cbf.A, 1) >= 42);
assert(numel(cbf.names) == numel(cbf.b));
assert(isfinite(cbf.minMargin));
assert(isfinite(cbf.sigmaMin));
assert(isfinite(cbf.conditionNumber));
assert(isfinite(cbf.minCollisionDistance));
assert(any(cbf.names == "force_upper"));
assert(any(cbf.names == "force_rate_upper"));
assert(any(cbf.names == "length_upper"));
assert(any(cbf.names == "leg_speed_upper"));
assert(any(cbf.names == "leg_accel_upper"));
assert(any(cbf.names == "singularity_barrier"));
assert(any(cbf.names == "collision_barrier"));
end
