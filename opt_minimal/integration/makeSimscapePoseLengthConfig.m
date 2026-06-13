function config = makeSimscapePoseLengthConfig(model, overrides)
% makeSimscapePoseLengthConfig - 构建位姿外环加长度串级控制配置
arguments
    model struct
    overrides struct = struct()
end

poseFields = {'poseFeedbackGain', 'poseCorrectionLimit'};
lengthOverrides = overrides;
for index = 1:numel(poseFields)
    if isfield(lengthOverrides, poseFields{index})
        lengthOverrides = rmfield(lengthOverrides, poseFields{index});
    end
end
config = makeSimscapeLengthCascadeConfig(model, lengthOverrides);
config.poseFeedbackGain = 0.3;
config.poseCorrectionLimit = 0.002;
for index = 1:numel(poseFields)
    fieldName = poseFields{index};
    if isfield(overrides, fieldName)
        config.(fieldName) = overrides.(fieldName);
    end
end

validateattributes(config.poseFeedbackGain, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.poseCorrectionLimit, {'double'}, {'scalar', 'positive', 'finite'});
end
