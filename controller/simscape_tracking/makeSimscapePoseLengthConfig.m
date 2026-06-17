function config = makeSimscapePoseLengthConfig(model, overrides)
% makeSimscapePoseLengthConfig - 构建位姿外环加长度串级控制配置
arguments
    model struct
    overrides struct = struct()
end

poseFields = {'poseFeedbackGain', 'poseCorrectionLimit', 'poseFeedbackFilterHz'};
lengthOverrides = overrides;
for index = 1:numel(poseFields)
    if isfield(lengthOverrides, poseFields{index})
        lengthOverrides = rmfield(lengthOverrides, poseFields{index});
    end
end
config = makeSimscapeLengthCascadeConfig(model, lengthOverrides);
config.poseFeedbackGain = 1.0;
config.poseCorrectionLimit = 0.002;
config.poseFeedbackFilterHz = 2.0;
for index = 1:numel(poseFields)
    fieldName = poseFields{index};
    if isfield(overrides, fieldName)
        config.(fieldName) = overrides.(fieldName);
    end
end

validateattributes(config.poseFeedbackGain, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.poseCorrectionLimit, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.poseFeedbackFilterHz, {'double'}, {'scalar', 'positive', 'finite'});
if config.poseFeedbackFilterHz >= 0.5 / config.sampleTime
    error('makeSimscapePoseLengthConfig:FilterFrequencyTooHigh', ...
        '位姿反馈低通截止频率必须低于 Nyquist 频率。');
end
end
