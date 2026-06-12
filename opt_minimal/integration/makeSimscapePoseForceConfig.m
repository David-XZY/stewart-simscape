function config = makeSimscapePoseForceConfig(model, overrides)
% makeSimscapePoseForceConfig - 构建力输入位姿轨迹跟踪统一配置
arguments
    model struct
    overrides struct = struct()
end

config = struct();
config.bandwidthHz = 15;
config.gainScale = 0.45;
config.rotationGainScale = 1.4;
config.characteristicLength = model.Lc;
config.gravityEnabled = true;
config.derivativeSampleTime = 0.01;
config.forceLimit = max(abs([model.actuator.forceMin; model.actuator.forceMax]));
config.speedLimit = model.actuator.ldotMax;
config.accelerationLimit = model.actuator.lddotMax;

overrideFields = fieldnames(overrides);
for fieldIndex = 1:numel(overrideFields)
    fieldName = overrideFields{fieldIndex};
    if ~isfield(config, fieldName)
        error('makeSimscapePoseForceConfig:UnknownOverride', ...
            '未知力输入位姿跟踪配置字段 %s。', fieldName);
    end
    config.(fieldName) = overrides.(fieldName);
end

validateattributes(config.bandwidthHz, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.gainScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.rotationGainScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.characteristicLength, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.derivativeSampleTime, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.gravityEnabled, {'logical'}, {'scalar'});
validateattributes(config.forceLimit, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.speedLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.accelerationLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
end
