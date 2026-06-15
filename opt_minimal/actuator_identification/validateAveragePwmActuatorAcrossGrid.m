function report = validateAveragePwmActuatorAcrossGrid(actuator, overrides)
% validateAveragePwmActuatorAcrossGrid - 多轴多工况校验平均 PWM 模型
arguments
    actuator struct
    overrides struct = struct()
end

options = struct('axisIndices', 1:6, 'duties', [0.35, 0.60, 0.85], ...
    'legSpeeds', [0.03, 0.10, 0.20], 'duration', 0.15);
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('validateAveragePwmActuatorAcrossGrid:UnknownOverride', ...
            '未知多工况校验配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end

caseCount = numel(options.axisIndices) * numel(options.duties) * numel(options.legSpeeds);
cases = repmat(struct('axisIndex', 0, 'duty', 0, 'legSpeed', 0, ...
    'meanAverageForce', 0, 'meanSwitchingForce', 0, ...
    'relativeForceError', 0, 'normalizedForceError', 0), caseCount, 1);
caseIndex = 0;
for axisIndex = options.axisIndices
    for duty = options.duties
        for legSpeed = options.legSpeeds
            caseIndex = caseIndex + 1;
            sample = validateAveragePwmActuatorAgainstSwitching(actuator, struct( ...
                'axisIndex', axisIndex, 'duty', duty, ...
                'legSpeed', legSpeed, 'duration', options.duration));
            absoluteError = abs(sample.metrics.meanAverageForce - ...
                sample.metrics.meanSwitchingForce);
            cases(caseIndex) = struct('axisIndex', axisIndex, 'duty', duty, ...
                'legSpeed', legSpeed, ...
                'meanAverageForce', sample.metrics.meanAverageForce, ...
                'meanSwitchingForce', sample.metrics.meanSwitchingForce, ...
                'relativeForceError', sample.metrics.relativeMeanForceError, ...
                'normalizedForceError', absoluteError / actuator.forceLimit(axisIndex));
        end
    end
end

metrics = struct();
metrics.maxRelativeForceError = max([cases.relativeForceError]);
metrics.maxNormalizedForceError = max([cases.normalizedForceError]);
metrics.maxAbsoluteForceError = max(abs([cases.meanAverageForce] - [cases.meanSwitchingForce]));
report = struct('cases', cases, 'metrics', metrics, ...
    'passed', metrics.maxNormalizedForceError <= 0.006 && ...
    metrics.maxRelativeForceError <= 0.12);
end
