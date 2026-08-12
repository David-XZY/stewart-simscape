function config = makeSafetyQpControlConfig(model, overrides)
% makeSafetyQpControlConfig - 构建 CLF-CBF-QP 安全关键控制器配置
arguments
    model struct
    overrides struct = struct()
end

config = struct();
config.methodName = "Safety-Critical CLF-CBF-QP Tracking Control";
config.shortName = "SC-QP";
config.dt = 0.01;
config.nominalMode = "computed-torque";
config.characteristicLength = model.Lc;
config.forceMin = model.actuator.forceMin(:);
config.forceMax = model.actuator.forceMax(:);
config.forceRateLimit = 4 * max(abs([config.forceMin; config.forceMax])) * ones(6, 1);
config.lengthMin = model.lmin(:);
config.lengthMax = model.lmax(:);
config.lengthMargin = 0.01;
config.legSpeedLimit = model.actuator.ldotMax(:);
config.legAccelerationLimit = model.actuator.lddotMax(:);
config.sigmaSafe = model.singularity.sigmaMinSafe;
config.collisionSafeDistance = 0.005;
config.enableClf = true;
config.enableCbf = true;
config.enableSingularityBarrier = false;
config.enableCollisionBarrier = false;
config.forceInfeasibleForTest = false;

config.clf = struct();
config.clf.positionGain = diag([35, 35, 35, 20, 20, 20]);
config.clf.velocityGain = diag([12, 12, 12, 8, 8, 8]);
config.clf.P = diag([ones(1, 3), config.characteristicLength^2 * ones(1, 3), ...
    0.08 * ones(1, 3), 0.08 * config.characteristicLength^2 * ones(1, 3)]);
config.clf.rate = 4.0;

config.weights = struct();
config.weights.force = ones(6, 1);
config.weights.forceRate = 2e-3;
config.weights.clfSlack = 1e5;
config.weights.cbfSlack = 1e8;

config.fallback = struct();
config.fallback.nominalScale = 0.35;
config.fallback.safeBrakeDamping = 40;
config.fallback.maxProjectionPasses = 3;

overrideFields = fieldnames(overrides);
for fieldIndex = 1:numel(overrideFields)
    fieldName = overrideFields{fieldIndex};
    if ~isfield(config, fieldName)
        error('makeSafetyQpControlConfig:UnknownOverride', ...
            '未知 SC-QP 配置字段：%s。', fieldName);
    end
    config.(fieldName) = overrides.(fieldName);
end

config.nominalMode = string(validatestring(config.nominalMode, ...
    {'ihsid', 'computed-torque', 'pd'}));
config.forceMin = config.forceMin(:);
config.forceMax = config.forceMax(:);
config.forceRateLimit = scalarOrVector6(config.forceRateLimit, 'forceRateLimit');
config.lengthMin = config.lengthMin(:);
config.lengthMax = config.lengthMax(:);
config.legSpeedLimit = scalarOrVector6(config.legSpeedLimit, 'legSpeedLimit');
config.legAccelerationLimit = scalarOrVector6(config.legAccelerationLimit, 'legAccelerationLimit');

validateattributes(config.dt, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.forceMin, {'double'}, {'size', [6, 1], 'finite'});
validateattributes(config.forceMax, {'double'}, {'size', [6, 1], 'finite'});
validateattributes(config.forceRateLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.lengthMin, {'double'}, {'size', [6, 1], 'finite'});
validateattributes(config.lengthMax, {'double'}, {'size', [6, 1], 'finite'});
validateattributes(config.lengthMargin, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(config.sigmaSafe, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.collisionSafeDistance, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(config.clf.positionGain, {'double'}, {'size', [6, 6], 'finite'});
validateattributes(config.clf.velocityGain, {'double'}, {'size', [6, 6], 'finite'});
validateattributes(config.clf.P, {'double'}, {'size', [12, 12], 'finite'});
validateattributes(config.weights.force, {'double'}, {'size', [6, 1], 'positive', 'finite'});
if any(config.forceMin >= config.forceMax)
    error('makeSafetyQpControlConfig:InvalidForceBounds', ...
        'forceMin 必须逐元素小于 forceMax。');
end
if any(config.lengthMin >= config.lengthMax)
    error('makeSafetyQpControlConfig:InvalidLengthBounds', ...
        'lengthMin 必须逐元素小于 lengthMax。');
end
end

function value = scalarOrVector6(value, name)
value = value(:);
if isscalar(value)
    value = repmat(value, 6, 1);
end
if numel(value) ~= 6
    error('makeSafetyQpControlConfig:InvalidVectorSize', ...
        '%s 必须为标量或 6x1 向量。', name);
end
end
