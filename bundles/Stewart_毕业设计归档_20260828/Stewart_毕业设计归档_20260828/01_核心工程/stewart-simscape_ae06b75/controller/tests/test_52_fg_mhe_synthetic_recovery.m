function test_52_fg_mhe_synthetic_recovery
% test_52_fg_mhe_synthetic_recovery - 验证合成数据下参数可恢复且失败可回退
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'factor_graph_estimation'));

model = buildOptModelCustom();
truth = struct();
truth.bL = [0.8; -0.4; 0.6; -0.5; 0.3; -0.2] * 1e-3;
truth.dm = 0.35;
truth.dc = [0.01; -0.008; 0.006];
config = makeStewartFgMheConfig(model, 0.01, struct( ...
    'windowLength', 8, ...
    'estimate', struct('qdd', false, 'payload', true, 'actuator', false), ...
    'solver', struct('maxIterations', 40, 'display', 'off', 'functionTolerance', 1e-10, 'stepTolerance', 1e-10)));

series = evaluateStewartFgMheSeries(struct( ...
    'model', model, ...
    'config', config, ...
    'truth', truth, ...
    'smoke', true, ...
    'runVariants', "fg_mhe_with_dynamics"));

initialBL = norm(truth.bL);
estimatedBL = norm(series.finalEstimate.bL - truth.bL);
assert(estimatedBL < 0.75 * initialBL);
assert(abs(series.finalEstimate.dm - truth.dm) < 0.75 * abs(truth.dm));

badConfig = makeStewartFgMheConfig(model, 0.01, struct( ...
    'windowLength', 8, ...
    'solver', struct('maxIterations', 0, 'display', 'off', 'functionTolerance', 1e-10, 'stepTolerance', 1e-10)));
window = series.windows{1};
initial = initializeStewartFgMheWindow(window, badConfig);
result = solveStewartFgMheWindow(window, initial, badConfig, struct());
assert(isfield(result, 'fallbackUsed'));
assert(result.fallbackUsed);
assert(isfield(result, 'estimate'));
end
