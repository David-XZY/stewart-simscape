function result = run_10_sampling_mpc_control(outputDir)
% run_10_sampling_mpc_control - 运行 SC-MPC 与现有控制算法对比实验
arguments
    outputDir {mustBeTextScalar} = ""
end

runRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(runRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'predictive_control'));

model = buildOptModelCustom();
config = makeSamplingMpcControlConfig(model);
refs = makeDemoReference(model, config);
comparison = compareSamplingMpcWithBaselineControllers(refs, model, config);
if strlength(string(outputDir)) == 0
    outputDir = fullfile(projectRoot, 'results', 'reports', 'sampling_mpc_control');
end
exported = exportSamplingMpcEvidence(comparison, outputDir, config);
methodDocument = writeSamplingMpcMethodDocument(projectRoot);

result = struct();
result.config = config;
result.refs = refs;
result.comparison = comparison;
result.exported = exported;
result.methodDocument = methodDocument;
fprintf('SC-MPC 对比证据已导出到：%s\n', exported.outputDir);
end

function refs = makeDemoReference(model, config)
time = 0:config.sampleTime:1.0;
shape = sin(pi * time / time(end));
refs = struct();
refs.t = time;
refs.q = model.qHome + [0.008; -0.005; 0.006; 0.002; -0.0015; 0.0012] .* shape;
refs.qd = differentiateRows(refs.q, time);
refs.qdd = differentiateRows(refs.qd, time);
refs.Fcomputed = zeros(6, numel(time));
refs.Fleg = zeros(6, numel(time));
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
