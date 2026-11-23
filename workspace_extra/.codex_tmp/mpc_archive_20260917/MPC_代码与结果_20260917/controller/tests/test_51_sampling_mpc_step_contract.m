function test_51_sampling_mpc_step_contract
% test_51_sampling_mpc_step_contract - 验证单步采样 MPC 会在候选序列中选优
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'predictive_control'));

model = buildOptModelCustom();
config = makeSamplingMpcControlConfig(model, struct('horizonSteps', 4));
refs = makeShortReference(model, config);
q = refs.q(:, 1) + [0.004; -0.003; 0.002; 0.001; -0.001; 0.0008];
qd = refs.qd(:, 1);
Fprev = zeros(6, 1);
Fnom = computeComputedTorqueForce(refs.q(:, 1), refs.qd(:, 1), refs.qdd(:, 1), ...
    q, qd, model, config);

[Fcmd, diagnostic] = stepSamplingMpcController(q, qd, refs, 1, Fprev, model, config, Fnom);

assert(isequal(size(Fcmd), [6, 1]));
assert(all(isfinite(Fcmd)));
assert(all(Fcmd <= config.forceMax + 1e-9));
assert(all(Fcmd >= config.forceMin - 1e-9));
assert(all(abs(Fcmd - Fprev) <= config.forceRateLimit * config.sampleTime + 1e-9));
assert(diagnostic.selectedIndex >= 1);
assert(height(diagnostic.candidateTable) == numel(config.candidateScales));
assert(diagnostic.bestCost <= max(diagnostic.candidateTable.totalCost));
assert(diagnostic.horizonSteps == config.horizonSteps);
zeroScaleRow = abs(diagnostic.candidateTable.scale) < 1e-12;
assert(diagnostic.bestCost <= diagnostic.candidateTable.totalCost(zeroScaleRow) + 1e-12);
end

function refs = makeShortReference(model, config)
time = 0:config.sampleTime:0.16;
shape = sin(pi * time / time(end));
refs = struct();
refs.t = time;
refs.q = model.qHome + [0.004; -0.002; 0.003; 0.001; -0.001; 0.0008] .* shape;
refs.qd = differentiateRows(refs.q, time);
refs.qdd = differentiateRows(refs.qd, time);
end

function derivative = differentiateRows(value, time)
derivative = zeros(size(value));
for row = 1:size(value, 1)
    derivative(row, :) = gradient(value(row, :), time);
end
end
