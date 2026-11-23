function test_50_fg_mhe_config_contract
% test_50_fg_mhe_config_contract - 验证 SC-FG-MHE 配置字段契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'factor_graph_estimation'));

model = buildOptModelCustom();
sampleTime = 0.01;
config = makeStewartFgMheConfig(model, sampleTime);

assert(config.windowLength >= 4);
assert(config.sampleTime == sampleTime);
assert(isfield(config, 'weights'));
assert(isfield(config.weights, 'leg'));
assert(isfield(config.weights, 'attitude'));
assert(isfield(config.weights, 'imu'));
assert(isfield(config.weights, 'dynamics'));
assert(isfield(config.weights, 'parameterPrior'));
assert(isfield(config, 'robust'));
assert(isfield(config.robust, 'enabled'));
assert(isfield(config.robust, 'kernel'));
assert(isfield(config, 'estimate'));
assert(isfield(config.estimate, 'qdd'));
assert(isfield(config.estimate, 'payload'));
assert(isfield(config.estimate, 'actuator'));
assert(isfield(config, 'solver'));
assert(isfield(config.solver, 'maxIterations'));
assert(isfield(config.solver, 'display'));
assert(isfield(config, 'geometry'));
assert(isfield(config.geometry, 'legLengthMin'));
assert(isfield(config.geometry, 'legLengthMax'));
assert(isfield(config.geometry, 'sigmaSafe'));
assert(isfield(config, 'outputs'));
assert(contains(config.outputs.reportSubdir, 'factor_graph_estimation'));

overrideConfig = makeStewartFgMheConfig(model, sampleTime, struct( ...
    'windowLength', 7, ...
    'robust', struct('enabled', true, 'kernel', "huber", 'huberDelta', 1.5)));
assert(overrideConfig.windowLength == 7);
assert(overrideConfig.robust.enabled);
assert(overrideConfig.robust.kernel == "huber");

failed = false;
try
    makeStewartFgMheConfig(model, sampleTime, struct('unknownField', 1));
catch ME
    failed = strcmp(ME.identifier, 'makeStewartFgMheConfig:UnknownOverride');
end
assert(failed);
end
