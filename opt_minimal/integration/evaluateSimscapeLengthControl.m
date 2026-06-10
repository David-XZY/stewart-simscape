function report = evaluateSimscapeLengthControl(simout, refs, model, design)
% evaluateSimscapeLengthControl - 解析力前馈加长度反馈仿真并执行硬验收

arguments
    simout struct
    refs struct
    model struct
    design struct
end

[time, actualRelativeLength] = readTimeseries(simout.y.dLm);
[poseTime, actualRelativePose] = readTimeseries(simout.x.Xr);
[forceTime, controlForce] = readTimeseries(simout.u);
[feedbackForceTime, feedbackForce] = readTimeseries(simout.uFeedback);
[feedforwardForceTime, feedforwardForce] = readTimeseries(simout.uFF);

referenceRelativeLength = interp1(refs.t(:), (refs.L - refs.L(:, 1)).', time, 'linear');
referenceRelativePose = interp1(refs.t(:), (refs.q - refs.q0).', poseTime, 'linear');
optimizedForce = interp1(refs.t(:), refs.Fleg.', forceTime, 'linear');

absoluteLength = actualRelativeLength + refs.L(:, 1).';
lengthError = referenceRelativeLength - actualRelativeLength;
poseError = referenceRelativePose - actualRelativePose;

regularTime = regularDerivativeTime(time);
regularAbsoluteLength = interp1(time, absoluteLength, regularTime, 'linear');
legSpeed = differentiate(regularTime, regularAbsoluteLength);
legAcceleration = differentiate(regularTime, legSpeed);

allValues = [actualRelativeLength(:); actualRelativePose(:); ...
    controlForce(:); feedbackForce(:); feedforwardForce(:)];
finitePassed = all(isfinite(allValues));
lengthPassed = all(absoluteLength >= model.lmin.' - 1e-8, 'all') && ...
    all(absoluteLength <= model.lmax.' + 1e-8, 'all');
forcePassed = all(controlForce >= model.actuator.forceMin.' - 1e-6, 'all') && ...
    all(controlForce <= model.actuator.forceMax.' + 1e-6, 'all');
linearDesignPassed = design.stable && design.lowFrequencyRank == 6;
lengthTrackingPeak = max(abs(lengthError), [], 1);
posePeak = max(abs(poseError), [], 1);
maxLengthTrackingPeak = max(lengthTrackingPeak);
maxTranslationPeak = max(posePeak(1:3));
maxRotationPeak = max(posePeak(4:6));
lengthTrackingPassed = maxLengthTrackingPeak <= 5e-3;
translationTrackingPassed = maxTranslationPeak <= 10e-3;
rotationTrackingPassed = maxRotationPeak <= deg2rad(1);
trackingPassed = lengthTrackingPassed && translationTrackingPassed && rotationTrackingPassed;

report = struct();
report.time = time;
report.poseTime = poseTime;
report.forceTime = forceTime;
report.feedbackForceTime = feedbackForceTime;
report.feedforwardForceTime = feedforwardForceTime;
report.actualRelativeLength = actualRelativeLength;
report.referenceRelativeLength = referenceRelativeLength;
report.actualAbsoluteLength = absoluteLength;
report.lengthError = lengthError;
report.actualRelativePose = actualRelativePose;
report.referenceRelativePose = referenceRelativePose;
report.poseError = poseError;
report.controlForce = controlForce;
report.feedbackForce = feedbackForce;
report.feedforwardForce = feedforwardForce;
report.optimizedForce = optimizedForce;
report.derivativeTime = regularTime;
report.legSpeed = legSpeed;
report.legAcceleration = legAcceleration;
report.metrics = struct( ...
    'lengthRms', sqrt(mean(lengthError.^2, 1)), ...
    'lengthPeak', lengthTrackingPeak, ...
    'poseRms', sqrt(mean(poseError.^2, 1)), ...
    'posePeak', posePeak, ...
    'maxLengthTrackingPeak', maxLengthTrackingPeak, ...
    'maxTranslationPeak', maxTranslationPeak, ...
    'maxRotationPeak', maxRotationPeak, ...
    'maxAbsControlForce', max(abs(controlForce), [], 'all'), ...
    'maxAbsFeedbackForce', max(abs(feedbackForce), [], 'all'), ...
    'maxAbsFeedforwardForce', max(abs(feedforwardForce), [], 'all'), ...
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
    'linearDesignPassed', linearDesignPassed, ...
    'lengthTrackingPassed', lengthTrackingPassed, ...
    'translationTrackingPassed', translationTrackingPassed, ...
    'rotationTrackingPassed', rotationTrackingPassed, ...
    'trackingPassed', trackingPassed);
report.thresholds = struct( ...
    'maxLengthTrackingPeak', 5e-3, ...
    'maxTranslationPeak', 10e-3, ...
    'maxRotationPeak', deg2rad(1));
report.passed = finitePassed && lengthPassed && forcePassed && ...
    linearDesignPassed && trackingPassed;
end

function regularTime = regularDerivativeTime(time)
% regularDerivativeTime - 使用规则时间网格避免求解器微小步长放大数值导数
duration = time(end) - time(1);
sampleCount = max(2, ceil(duration / 1e-3) + 1);
regularTime = linspace(time(1), time(end), sampleCount).';
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
