function result = run_12_safety_qp_tracking_control(trajectoryFile, outputDir)
% run_12_safety_qp_tracking_control - 运行 SC-QP 控制对比、消融和证据导出
arguments
    trajectoryFile {mustBeTextScalar} = ""
    outputDir {mustBeTextScalar} = ""
end

runRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(runRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

if strlength(string(outputDir)) == 0
    outputDir = fullfile(projectRoot, 'results', 'reports', 'safety_qp_control');
end

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model);
refs = loadReferenceTrajectory(trajectoryFile, optRoot, model, config);
comparison = compareSafetyQpWithBaselineControllers(refs, model, config);
exported = exportSafetyQpEvidence(comparison, outputDir, config);
methodDoc = writeSafetyQpMethodDocument(projectRoot);

result = struct();
result.refs = refs;
result.model = model;
result.config = config;
result.comparison = comparison;
result.exported = exported;
result.methodDocument = methodDoc;
fprintf('SC-QP 控制证据已导出到：%s\n', exported.outputDir);
end

function refs = loadReferenceTrajectory(trajectoryFile, optRoot, model, config)
if strlength(string(trajectoryFile)) == 0
    candidate = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
        'simscape_references.mat');
else
    candidate = char(trajectoryFile);
end
if isfile(candidate)
    data = load(candidate, 'refs');
    refs = data.refs;
    if ~isfield(refs, 'qdd')
        refs.qdd = differentiateRows(refs.qd, refs.t);
    end
    if ~isfield(refs, 'Fcomputed')
        refs.Fcomputed = refs.Fleg;
    end
    refs = completeReferenceKinematics(refs, model);
else
    refs = makeDemoReference(model, config);
end
end

function refs = completeReferenceKinematics(refs, model)
sampleCount = numel(refs.t);
if ~isfield(refs, 'L')
    refs.L = zeros(6, sampleCount);
end
if ~isfield(refs, 'Ld')
    refs.Ld = zeros(6, sampleCount);
end
for index = 1:sampleCount
    if ~all(isfinite(refs.L(:, index)))
        refs.L(:, index) = sgpIK(refs.q(:, index), model).L;
    end
    if ~all(isfinite(refs.Ld(:, index)))
        refs.Ld(:, index) = sgpJacobian(refs.q(:, index), model).Jq * refs.qd(:, index);
    end
end
end

function refs = makeDemoReference(model, config)
time = 0:config.dt:1.2;
shape = sin(pi * time / time(end));
refs = struct();
refs.t = time;
refs.q = model.qHome + [0.01; -0.006; 0.008; 0.004; -0.003; 0.002] .* shape;
refs.qd = differentiateRows(refs.q, time);
refs.qdd = differentiateRows(refs.qd, time);
refs.Fleg = zeros(6, numel(time));
refs.Fcomputed = zeros(6, numel(time));
refs.L = zeros(6, numel(time));
refs.Ld = zeros(6, numel(time));
for index = 1:numel(time)
    refs.L(:, index) = sgpIK(refs.q(:, index), model).L;
    refs.Ld(:, index) = sgpJacobian(refs.q(:, index), model).Jq * refs.qd(:, index);
end
end

function derivative = differentiateRows(value, time)
derivative = zeros(size(value));
for row = 1:size(value, 1)
    derivative(row, :) = gradient(value(row, :), time);
end
end
