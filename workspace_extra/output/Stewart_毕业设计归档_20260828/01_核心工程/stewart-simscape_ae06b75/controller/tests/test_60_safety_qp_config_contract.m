function test_60_safety_qp_config_contract
% test_60_safety_qp_config_contract - 验证 SC-QP 控制器配置字段契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model, struct( ...
    'dt', 0.005, ...
    'nominalMode', "pd", ...
    'enableSingularityBarrier', true));

requiredFields = {
    'methodName'
    'shortName'
    'dt'
    'nominalMode'
    'forceMin'
    'forceMax'
    'forceRateLimit'
    'lengthMin'
    'lengthMax'
    'lengthMargin'
    'legSpeedLimit'
    'legAccelerationLimit'
    'sigmaSafe'
    'collisionSafeDistance'
    'clf'
    'weights'
    'fallback'
    'enableClf'
    'enableCbf'
    'enableSingularityBarrier'
    'enableCollisionBarrier'
    };
for index = 1:numel(requiredFields)
    assert(isfield(config, requiredFields{index}), requiredFields{index});
end

assert(config.shortName == "SC-QP");
assert(config.dt == 0.005);
assert(config.nominalMode == "pd");
assert(config.enableSingularityBarrier);
assert(numel(config.forceMin) == 6);
assert(numel(config.forceMax) == 6);
assert(all(config.forceMin < config.forceMax));
assert(all(config.lengthMin < config.lengthMax));
assert(config.weights.cbfSlack > config.weights.clfSlack);
assert(config.fallback.safeBrakeDamping > 0);
end
