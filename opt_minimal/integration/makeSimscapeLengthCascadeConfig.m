function config = makeSimscapeLengthCascadeConfig(model, overrides)
% makeSimscapeLengthCascadeConfig - 构建纯长度串级控制统一配置
arguments
    model struct
    overrides struct = struct()
end

config = struct();
config.sampleTime = 0.01;
config.innerBandwidthHz = 10;
config.outerBandwidthRatio = 5;
config.positionGainScale = 1;
config.velocityGainScale = 1;
config.velocityPlantGain = ones(6, 1);
config.velocityPlantTimeConstant = 0.03 * ones(6, 1);
config.speedLimit = model.actuator.ldotMax;
config.accelerationLimit = model.actuator.lddotMax;
config.gravityEnabled = false;

overrideFields = fieldnames(overrides);
for fieldIndex = 1:numel(overrideFields)
    fieldName = overrideFields{fieldIndex};
    if ~isfield(config, fieldName)
        error('makeSimscapeLengthCascadeConfig:UnknownOverride', ...
            '未知长度串级配置字段 %s。', fieldName);
    end
    config.(fieldName) = overrides.(fieldName);
end

validateattributes(config.sampleTime, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.innerBandwidthHz, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.outerBandwidthRatio, {'double'}, {'scalar', '>', 1, 'finite'});
validateattributes(config.positionGainScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.velocityGainScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.velocityPlantGain, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.velocityPlantTimeConstant, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.speedLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.accelerationLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.gravityEnabled, {'logical'}, {'scalar'});
end
