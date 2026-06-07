function report = evaluateSimscapeLengthControl(simout, refs, model, design)
% evaluateSimscapeLengthControl - 解析纯长度反馈仿真并执行硬验收

arguments
    simout struct
    refs struct
    model struct
    design struct
end

[time, actualRelativeLength] = readTimeseries(simout.y.dLm);
[poseTime, actualRelativePose] = readTimeseries(simout.x.Xr);
[forceTime, controlForce] = readTimeseries(simout.u);

referenceRelativeLength = interp1(refs.t(:), (refs.L - refs.L(:, 1)).', time, 'linear');
referenceRelativePose = interp1(refs.t(:), (refs.q - refs.q0).', poseTime, 'linear');
optimizedForce = interp1(refs.t(:), refs.Fleg.', forceTime, 'linear');

absoluteLength = actualRelativeLength + refs.L(:, 1).';
lengthError = referenceRelativeLength - actualRelativeLength;
poseError = referenceRelativePose - actualRelativePose;

legSpeed = differentiate(time, absoluteLength);
legAcceleration = differentiate(time, legSpeed);

allValues = [actualRelativeLength(:); actualRelativePose(:); controlForce(:)];
finitePassed = all(isfinite(allValues));
lengthPassed = all(absoluteLength >= model.lmin.' - 1e-8, 'all') && ...
    all(absoluteLength <= model.lmax.' + 1e-8, 'all');
forcePassed = all(controlForce >= model.actuator.forceMin.' - 1e-6, 'all') && ...
    all(controlForce <= model.actuator.forceMax.' + 1e-6, 'all');
linearDesignPassed = design.stable && design.lowFrequencyRank == 6;

report = struct();
report.time = time;
report.poseTime = poseTime;
report.forceTime = forceTime;
report.actualRelativeLength = actualRelativeLength;
report.referenceRelativeLength = referenceRelativeLength;
report.actualAbsoluteLength = absoluteLength;
report.lengthError = lengthError;
report.actualRelativePose = actualRelativePose;
report.referenceRelativePose = referenceRelativePose;
report.poseError = poseError;
report.controlForce = controlForce;
report.optimizedForce = optimizedForce;
report.legSpeed = legSpeed;
report.legAcceleration = legAcceleration;
report.metrics = struct( ...
    'lengthRms', sqrt(mean(lengthError.^2, 1)), ...
    'lengthPeak', max(abs(lengthError), [], 1), ...
    'poseRms', sqrt(mean(poseError.^2, 1)), ...
    'posePeak', max(abs(poseError), [], 1), ...
    'maxAbsControlForce', max(abs(controlForce), [], 'all'), ...
    'maxAbsOptimizedForce', max(abs(optimizedForce), [], 'all'), ...
    'minAbsoluteLength', min(absoluteLength, [], 'all'), ...
    'maxAbsoluteLength', max(absoluteLength, [], 'all'), ...
    'maxAbsLegSpeed', max(abs(legSpeed), [], 'all'), ...
    'maxAbsLegAcceleration', max(abs(legAcceleration), [], 'all'));
report.acceptance = struct( ...
    'simulationCompleted', true, ...
    'finitePassed', finitePassed, ...
    'lengthPassed', lengthPassed, ...
    'forcePassed', forcePassed, ...
    'linearDesignPassed', linearDesignPassed);
report.passed = finitePassed && lengthPassed && forcePassed && linearDesignPassed;
end

function [time, data] = readTimeseries(value)
% readTimeseries - 将模型记录的 timeseries 统一整理为 N×6
if ~isa(value, 'timeseries')
    error('evaluateSimscapeLengthControl:InvalidSignal', '仿真输出必须为 timeseries。');
end
time = value.Time(:);
data = squeeze(value.Data);
if size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
if size(data, 1) ~= numel(time) || size(data, 2) ~= 6
    error('evaluateSimscapeLengthControl:InvalidSignalSize', ...
        '仿真信号必须可整理为 N×6，当前尺寸为 %s。', mat2str(size(data)));
end
[time, uniqueIndex] = unique(time, 'stable');
data = data(uniqueIndex, :);
end

function derivative = differentiate(time, value)
% differentiate - 使用非均匀时间步长计算数值导数
derivative = zeros(size(value));
for columnIndex = 1:size(value, 2)
    derivative(:, columnIndex) = gradient(value(:, columnIndex), time);
end
end
