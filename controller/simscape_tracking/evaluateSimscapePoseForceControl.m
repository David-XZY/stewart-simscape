function report = evaluateSimscapePoseForceControl(simout, refs, model, design, config)
% evaluateSimscapePoseForceControl - 解析并验收力输入位姿轨迹跟踪仿真
arguments
    simout struct
    refs struct
    model struct
    design struct
    config struct
end

[time, actualRelativeLength] = readTimeseries(simout.y.dLm);
[poseTime, actualRelativePose] = readTimeseries(simout.x.Xr);
[forceTime, controlForce] = readTimeseries(simout.u);
[feedbackForceTime, feedbackForce] = readTimeseries(simout.uFeedback);
[feedforwardForceTime, feedforwardForce] = readTimeseries(simout.uFF);

referenceRelativeLength = interp1(refs.t(:), (refs.L - refs.L(:, 1)).', time, 'linear');
referenceRelativePose = interp1(refs.t(:), (refs.q - refs.q0).', poseTime, 'linear');
actualAbsoluteLength = actualRelativeLength + refs.L(:, 1).';
lengthError = referenceRelativeLength - actualRelativeLength;
poseError = referenceRelativePose - actualRelativePose;
equivalentPoseError = [poseError(:, 1:3), config.characteristicLength * poseError(:, 4:6)];

regularTime = regularDerivativeTime(time, config.derivativeSampleTime);
regularLength = interp1(time, actualAbsoluteLength, regularTime, 'linear');
legSpeed = differentiate(regularTime, regularLength);
legAcceleration = differentiate(regularTime, legSpeed);

allValues = [actualRelativeLength(:); actualRelativePose(:); ...
    controlForce(:); feedbackForce(:); feedforwardForce(:)];
finitePassed = all(isfinite(allValues));
lengthPassed = all(actualAbsoluteLength >= model.lmin.' - 1e-8, 'all') && ...
    all(actualAbsoluteLength <= model.lmax.' + 1e-8, 'all');
forcePassed = all(controlForce >= model.actuator.forceMin.' - 1e-6, 'all') && ...
    all(controlForce <= model.actuator.forceMax.' + 1e-6, 'all');
speedPassed = all(abs(legSpeed) <= config.speedLimit.' + 1e-4, 'all');
accelerationPassed = all(abs(legAcceleration) <= config.accelerationLimit.' + 1e-2, 'all');

lengthPeak = max(abs(lengthError), [], 1);
posePeak = max(abs(poseError), [], 1);
maxLengthTrackingPeak = max(lengthPeak);
maxTranslationPeak = max(posePeak(1:3));
maxRotationPeak = max(posePeak(4:6));
trackingPassed = maxLengthTrackingPeak <= 5e-3 && ...
    maxTranslationPeak <= 10e-3 && maxRotationPeak <= deg2rad(1);

report = struct();
report.time = time;
report.poseTime = poseTime;
report.forceTime = forceTime;
report.feedbackForceTime = feedbackForceTime;
report.feedforwardForceTime = feedforwardForceTime;
report.actualRelativeLength = actualRelativeLength;
report.referenceRelativeLength = referenceRelativeLength;
report.actualAbsoluteLength = actualAbsoluteLength;
report.lengthError = lengthError;
report.actualRelativePose = actualRelativePose;
report.referenceRelativePose = referenceRelativePose;
report.poseError = poseError;
report.equivalentPoseError = equivalentPoseError;
report.controlForce = controlForce;
report.feedbackForce = feedbackForce;
report.feedforwardForce = feedforwardForce;
report.derivativeTime = regularTime;
report.legSpeed = legSpeed;
report.legAcceleration = legAcceleration;
report.metrics = struct( ...
    'lengthRms', sqrt(mean(lengthError.^2, 1)), ...
    'lengthPeak', lengthPeak, ...
    'poseRms', sqrt(mean(poseError.^2, 1)), ...
    'posePeak', posePeak, ...
    'equivalentPoseRms', sqrt(mean(equivalentPoseError.^2, 'all')), ...
    'equivalentPosePeak', max(abs(equivalentPoseError), [], 'all'), ...
    'maxLengthTrackingPeak', maxLengthTrackingPeak, ...
    'maxTranslationPeak', maxTranslationPeak, ...
    'maxRotationPeak', maxRotationPeak, ...
    'maxAbsControlForce', max(abs(controlForce), [], 'all'), ...
    'maxAbsFeedbackForce', max(abs(feedbackForce), [], 'all'), ...
    'maxAbsFeedforwardForce', max(abs(feedforwardForce), [], 'all'), ...
    'minAbsoluteLength', min(actualAbsoluteLength, [], 'all'), ...
    'maxAbsoluteLength', max(actualAbsoluteLength, [], 'all'), ...
    'maxAbsLegSpeed', max(abs(legSpeed), [], 'all'), ...
    'maxAbsLegAcceleration', max(abs(legAcceleration), [], 'all'));
report.acceptance = struct( ...
    'simulationCompleted', true, ...
    'finitePassed', finitePassed, ...
    'linearDesignPassed', design.stable, ...
    'lengthPassed', lengthPassed, ...
    'forcePassed', forcePassed, ...
    'speedPassed', speedPassed, ...
    'accelerationPassed', accelerationPassed, ...
    'trackingPassed', trackingPassed);
report.thresholds = struct( ...
    'maxLengthTrackingPeak', 5e-3, ...
    'maxTranslationPeak', 10e-3, ...
    'maxRotationPeak', deg2rad(1));
report.passed = finitePassed && design.stable && lengthPassed && forcePassed && ...
    speedPassed && accelerationPassed && trackingPassed;
end

function regularTime = regularDerivativeTime(time, sampleTime)
regularTime = (time(1):sampleTime:time(end)).';
if regularTime(end) < time(end)
    regularTime(end + 1, 1) = time(end);
end
end

function [time, data] = readTimeseries(value)
if ~isa(value, 'timeseries')
    error('evaluateSimscapePoseForceControl:InvalidSignal', ...
        '仿真输出必须为 timeseries。');
end
time = value.Time(:);
data = squeeze(value.Data);
if size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
if size(data, 1) ~= numel(time) || size(data, 2) ~= 6
    error('evaluateSimscapePoseForceControl:InvalidSignalSize', ...
        '仿真信号必须可整理为 Nx6，当前尺寸为 %s。', mat2str(size(data)));
end
[time, uniqueIndex] = unique(time, 'stable');
data = data(uniqueIndex, :);
end

function derivative = differentiate(time, value)
derivative = zeros(size(value));
for columnIndex = 1:size(value, 2)
    derivative(:, columnIndex) = gradient(value(:, columnIndex), time);
end
end
