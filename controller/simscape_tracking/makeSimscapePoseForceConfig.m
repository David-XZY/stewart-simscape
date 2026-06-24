function config = makeSimscapePoseForceConfig(model, overrides)
% makeSimscapePoseForceConfig - 构建力输入位姿轨迹跟踪统一配置
arguments
    model struct
    overrides struct = struct()
end

config = struct();
config.controlLaw = "computed-torque";
config.feedbackLaw = "lqi-output";
config.actuatorMode = "nonideal-force";
config.bandwidthHz = 15;
config.gainScale = 0.45;
config.rotationGainScale = 1.4;
config.characteristicLength = model.Lc;
config.gravityEnabled = true;
config.derivativeSampleTime = 0.01;
config.forceLimit = max(abs([model.actuator.forceMin; model.actuator.forceMax]));
config.forceRateLimit = 4 * config.forceLimit / config.derivativeSampleTime;
config.forceLagTimeConstant = 0.001;
config.forceDeadzone = zeros(6, 1);
config.forceInputDelay = 1e-6;
config.nonidealMaxLengthTrackingPeak = 3e-2;
config.nonidealMaxTranslationPeak = 3e-2;
config.nonidealMaxRotationPeak = deg2rad(1);
config.nonidealAccelerationLimit = 10 * ones(6, 1);
config.speedLimit = model.actuator.ldotMax;
config.accelerationLimit = model.actuator.lddotMax;
config.lqiTranslationScale = 3e-2;
config.lqiRotationScale = deg2rad(1);
config.lqiVelocityScale = 5e-3;
config.lqiIntegralScale = 2e-1;
config.lqiControlScale = 5000;
config.lqiIntegralLeakHz = 2;
config.lqiDerivativeFilterTime = 1 / (2 * pi * config.bandwidthHz);
config.lqiFeedbackForceLimit = 120;
config.lqiAntiWindupTimeConstant = 0.02;

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
validateattributes(config.forceRateLimit, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.forceLagTimeConstant, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.forceDeadzone, {'double'}, {'size', [6, 1], 'nonnegative', 'finite'});
validateattributes(config.forceInputDelay, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(config.nonidealMaxLengthTrackingPeak, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.nonidealMaxTranslationPeak, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.nonidealMaxRotationPeak, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.nonidealAccelerationLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.speedLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.accelerationLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.lqiTranslationScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiRotationScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiVelocityScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiIntegralScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiControlScale, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiIntegralLeakHz, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiDerivativeFilterTime, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiFeedbackForceLimit, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lqiAntiWindupTimeConstant, {'double'}, {'scalar', 'positive', 'finite'});
config.controlLaw = validatestring(config.controlLaw, ...
    {'computed-torque', 'linear-pose-force'});
config.controlLaw = string(config.controlLaw);
config.feedbackLaw = validatestring(config.feedbackLaw, ...
    {'lqi-output'});
config.feedbackLaw = string(config.feedbackLaw);
config.actuatorMode = validatestring(config.actuatorMode, ...
    {'nonideal-force', 'ideal-force'});
config.actuatorMode = string(config.actuatorMode);
end
