function test_52_sampling_mpc_comparison_export
% test_52_sampling_mpc_comparison_export - 验证 SC-MPC 对比实验和证据导出
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'predictive_control'));

model = buildOptModelCustom();
config = makeSamplingMpcControlConfig(model, struct('sampleTime', 0.02, 'horizonSteps', 4));
refs = makeShortReference(model, config);
comparison = compareSamplingMpcWithBaselineControllers(refs, model, config);

assert(height(comparison.summaryTable) == 4);
assert(isequal(string(comparison.summaryTable.method), [
    "IHSID feedforward baseline"
    "Computed-torque LQI surrogate"
    "Safety QP surrogate"
    "Sampling constrained MPC"
    ]));
assert(comparison.summaryTable.position_rmse(4) < comparison.summaryTable.position_rmse(1));
assert(comparison.summaryTable.position_rmse(4) <= comparison.summaryTable.position_rmse(2));
assert(comparison.summaryTable.min_safety_margin(4) >= comparison.summaryTable.min_safety_margin(2));
assert(comparison.summaryTable.max_force_rate(4) <= config.forceRateLimit(1) + 1e-9);
assert(isfield(comparison.runs(3), 'diagnostics'));
assert(isfield(comparison.runs(4), 'diagnostics'));

outputDir = fullfile(tempdir, 'sampling_mpc_export_contract');
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
cleanup = onCleanup(@() removeDirectory(outputDir));
exported = exportSamplingMpcEvidence(comparison, outputDir, config);

assert(isfile(fullfile(outputDir, 'table_sampling_mpc_comparison.csv')));
assert(isfile(fullfile(outputDir, 'sampling_mpc_comparison_data.mat')));
assert(numel(exported.figureFiles) >= 6);
for index = 1:numel(exported.figureFiles)
    assert(isfile(exported.figureFiles(index)));
end
end

function refs = makeShortReference(model, config)
time = 0:config.sampleTime:0.30;
shape = sin(pi * time / time(end));
refs = struct();
refs.t = time;
refs.q = model.qHome + [0.006; -0.004; 0.003; 0.002; -0.0015; 0.001] .* shape;
refs.qd = differentiateRows(refs.q, time);
refs.qdd = differentiateRows(refs.qd, time);
refs.Fleg = zeros(6, numel(time));
refs.Fcomputed = zeros(6, numel(time));
for index = 1:numel(time)
    refs.Fcomputed(:, index) = computeComputedTorqueForce(refs.q(:, index), ...
        refs.qd(:, index), refs.qdd(:, index), refs.q(:, index), ...
        refs.qd(:, index), model, config);
end
end

function derivative = differentiateRows(value, time)
derivative = zeros(size(value));
for row = 1:size(value, 1)
    derivative(row, :) = gradient(value(row, :), time);
end
end

function removeDirectory(outputDir)
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
end
