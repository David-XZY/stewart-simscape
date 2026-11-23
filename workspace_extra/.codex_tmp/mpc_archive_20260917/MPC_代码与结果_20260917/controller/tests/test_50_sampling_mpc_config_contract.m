function test_50_sampling_mpc_config_contract
% test_50_sampling_mpc_config_contract - 验证采样预测控制器配置契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'predictive_control'));

model = buildOptModelCustom();
config = makeSamplingMpcControlConfig(model, struct('horizonSteps', 5, 'sampleTime', 0.02));

assert(config.shortName == "SC-MPC");
assert(config.horizonSteps == 5);
assert(config.sampleTime == 0.02);
assert(config.forceMin(1) == model.actuator.forceMin(1));
assert(config.forceMax(1) == model.actuator.forceMax(1));
assert(all(config.forceRateLimit > 0));
assert(numel(config.candidateScales) >= 7);
assert(config.weights.position > config.weights.force);
assert(config.weights.forceRate > 0);

try
    makeSamplingMpcControlConfig(model, struct('unknownField', 1));
    error('test_50_sampling_mpc_config_contract:UnknownOverrideAccepted', ...
        'SC-MPC 配置不应接受未知字段。');
catch errorInfo
    assert(~strcmp(errorInfo.identifier, ...
        'test_50_sampling_mpc_config_contract:UnknownOverrideAccepted'));
end
end
