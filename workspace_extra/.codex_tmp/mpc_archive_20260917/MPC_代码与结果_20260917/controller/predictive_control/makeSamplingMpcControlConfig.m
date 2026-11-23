function config = makeSamplingMpcControlConfig(model, overrides)
% makeSamplingMpcControlConfig - 构建采样式约束预测控制器配置
arguments
    model struct
    overrides struct = struct()
end

config = struct();
config.methodName = "Sampling Constrained Receding-Horizon MPC";
config.shortName = "SC-MPC";
config.sampleTime = 0.01;
config.horizonSteps = 6;
config.characteristicLength = model.Lc;
config.forceMin = model.actuator.forceMin(:);
config.forceMax = model.actuator.forceMax(:);
config.forceRateLimit = 4 * max(abs([config.forceMin; config.forceMax])) * ones(6, 1);
config.candidateScales = [-0.20, 0.00, 0.35, 0.65, 0.90, 1.10, 1.35, 1.70];
config.lengthMin = model.lmin(:);
config.lengthMax = model.lmax(:);
config.lengthMargin = 0.01;
config.sigmaSafe = model.singularity.sigmaMinSafe;

config.feedback = struct();
config.feedback.positionGain = diag([36, 36, 36, 22, 22, 22]);
config.feedback.velocityGain = diag([13, 13, 13, 8, 8, 8]);
config.feedback.feedbackForceLimit = 180;

config.weights = struct();
config.weights.position = 1.0;
config.weights.rotation = config.characteristicLength^2;
config.weights.velocity = 0.08;
config.weights.force = 2e-7;
config.weights.forceRate = 1e-12;
config.weights.safety = 3e3;
config.weights.terminal = 2.5;

overrideFields = fieldnames(overrides);
for fieldIndex = 1:numel(overrideFields)
    fieldName = overrideFields{fieldIndex};
    if ~isfield(config, fieldName)
        error('makeSamplingMpcControlConfig:UnknownOverride', ...
            '未知 SC-MPC 配置字段：%s。', fieldName);
    end
    config.(fieldName) = overrides.(fieldName);
end

config.forceMin = config.forceMin(:);
config.forceMax = config.forceMax(:);
config.forceRateLimit = scalarOrVector6(config.forceRateLimit, 'forceRateLimit');
config.candidateScales = config.candidateScales(:).';
validateattributes(config.sampleTime, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.horizonSteps, {'double'}, {'scalar', 'integer', '>=', 2});
validateattributes(config.forceMin, {'double'}, {'size', [6, 1], 'finite'});
validateattributes(config.forceMax, {'double'}, {'size', [6, 1], 'finite'});
validateattributes(config.forceRateLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(config.candidateScales, {'double'}, {'row', 'numel', numel(config.candidateScales), 'finite'});
validateattributes(config.lengthMargin, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(config.sigmaSafe, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.feedback.positionGain, {'double'}, {'size', [6, 6], 'finite'});
validateattributes(config.feedback.velocityGain, {'double'}, {'size', [6, 6], 'finite'});
validateattributes(config.feedback.feedbackForceLimit, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.weights.position, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.weights.force, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.weights.forceRate, {'double'}, {'scalar', 'positive', 'finite'});
if any(config.forceMin >= config.forceMax)
    error('makeSamplingMpcControlConfig:InvalidForceBounds', ...
        'forceMin 必须逐元素小于 forceMax。');
end
end

function value = scalarOrVector6(value, name)
value = value(:);
if isscalar(value)
    value = repmat(value, 6, 1);
end
if numel(value) ~= 6
    error('makeSamplingMpcControlConfig:InvalidVectorSize', ...
        '%s 必须为标量或 6x1 向量。', name);
end
end
