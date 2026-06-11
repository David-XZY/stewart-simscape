function report = evaluateSimscapeLengthCascadeControl(simout, refs, model, design, config)
% evaluateSimscapeLengthCascadeControl - 解析并验收纯长度串级控制仿真
arguments
    simout struct
    refs struct
    model struct
    design struct
    config struct
end

[time, actualRelativeLength] = readTimeseries(simout.y.dLm);
[poseTime, actualRelativePose] = readTimeseries(simout.x.Xr);
[commandTime, servoCommand] = readTimeseries(simout.uFeedback);
if isfield(simout.y, 'Taum')
    % 长度伺服分支复用旧 Taum 总线槽传递关节速度，物理量不是驱动力。
    [speedTime, measuredSpeed] = readTimeseries(simout.y.Taum);
else
    speedTime = [];
    measuredSpeed = [];
end

referenceRelativeLength = interp1(refs.t(:), ...
    (refs.Lref - refs.Lref(:, 1)).', time, 'linear');
referenceLengthSpeed = interp1(refs.t(:), refs.Ldref.', time, 'linear');
referenceRelativePose = interp1(refs.t(:), ...
    (refs.q - refs.q0).', poseTime, 'linear');
actualAbsoluteLength = actualRelativeLength + refs.Lref(:, 1).';
lengthError = referenceRelativeLength - actualRelativeLength;
poseError = referenceRelativePose - actualRelativePose;

regularTime = (time(1):config.sampleTime:time(end)).';
if regularTime(end) < time(end)
    regularTime(end + 1, 1) = time(end);
end
regularLength = interp1(time, actualAbsoluteLength, regularTime, 'linear');
LdActual = differentiate(regularTime, regularLength);
LddActual = differentiate(regularTime, LdActual);
if ~isempty(speedTime)
    LdActual = interp1(speedTime, measuredSpeed, regularTime, 'linear');
    LddActual = differentiate(regularTime, LdActual);
end

rawLdCmd = referenceLengthSpeed + lengthError .* design.Kpos.';
LdCmd = rateAndMagnitudeLimit(rawLdCmd, time, config);
LdActualAtTime = interp1(regularTime, LdActual, time, 'linear');
velocityError = LdCmd - LdActualAtTime;

allValues = [actualRelativeLength(:); actualRelativePose(:); ...
    servoCommand(:); LdCmd(:); LdActual(:); LddActual(:)];
finitePassed = all(isfinite(allValues));
lengthPassed = all(actualAbsoluteLength >= model.lmin.' - 1e-8, 'all') && ...
    all(actualAbsoluteLength <= model.lmax.' + 1e-8, 'all');
speedPassed = all(abs(LdActual) <= config.speedLimit.' + 1e-4, 'all');
accelerationPassed = all(abs(LddActual) <= config.accelerationLimit.' + 1e-2, 'all');
linearDesignPassed = design.stable && ...
    design.outerBandwidthHz < design.innerBandwidthHz;

lengthPeak = max(abs(lengthError), [], 1);
posePeak = max(abs(poseError), [], 1);
maxLengthTrackingPeak = max(lengthPeak);
maxTranslationPeak = max(posePeak(1:3));
maxRotationPeak = max(posePeak(4:6));
lengthTrackingPassed = maxLengthTrackingPeak <= 5e-3;
translationTrackingPassed = maxTranslationPeak <= 10e-3;
rotationTrackingPassed = maxRotationPeak <= deg2rad(1);
trackingPassed = lengthTrackingPassed && ...
    translationTrackingPassed && rotationTrackingPassed;

report = struct();
report.time = time;
report.poseTime = poseTime;
report.commandTime = commandTime;
report.Lref = referenceRelativeLength;
report.Ldref = referenceLengthSpeed;
report.LdCmd = LdCmd;
report.Lactual = actualRelativeLength;
report.LdActual = LdActual;
report.servoCommand = servoCommand;
report.lengthError = lengthError;
report.poseError = poseError;
report.metrics = struct( ...
    'lengthRms', sqrt(mean(lengthError.^2, 1)), ...
    'lengthPeak', lengthPeak, ...
    'endpointLengthError', abs(lengthError(end, :)), ...
    'maxEndpointLengthError', max(abs(lengthError(end, :))), ...
    'poseRms', sqrt(mean(poseError.^2, 1)), ...
    'posePeak', posePeak, ...
    'maxLengthTrackingPeak', maxLengthTrackingPeak, ...
    'maxTranslationPeak', maxTranslationPeak, ...
    'maxRotationPeak', maxRotationPeak, ...
    'velocityErrorRms', sqrt(mean(velocityError.^2, 1)), ...
    'velocityErrorPeak', max(abs(velocityError), [], 1), ...
    'maxAbsLegSpeed', max(abs(LdActual), [], 'all'), ...
    'maxAbsLegAcceleration', max(abs(LddActual), [], 'all'), ...
    'commandSaturationRatio', mean(abs(servoCommand) >= ...
        0.999 * config.speedLimit(1), 'all'));
report.acceptance = struct( ...
    'simulationCompleted', true, ...
    'finitePassed', finitePassed, ...
    'linearDesignPassed', linearDesignPassed, ...
    'lengthPassed', lengthPassed, ...
    'speedPassed', speedPassed, ...
    'accelerationPassed', accelerationPassed, ...
    'lengthTrackingPassed', lengthTrackingPassed, ...
    'translationTrackingPassed', translationTrackingPassed, ...
    'rotationTrackingPassed', rotationTrackingPassed, ...
    'trackingPassed', trackingPassed);
report.thresholds = struct( ...
    'maxLengthTrackingPeak', 5e-3, ...
    'maxTranslationPeak', 10e-3, ...
    'maxRotationPeak', deg2rad(1), ...
    'endpointLengthErrorDiagnostic', 1e-4);
report.passed = finitePassed && linearDesignPassed && lengthPassed && ...
    speedPassed && accelerationPassed && trackingPassed;
end

function value = rateAndMagnitudeLimit(rawValue, time, config)
% rateAndMagnitudeLimit - 按模型中的速度和加速度限制重建 LdCmd
upper = config.speedLimit(:).';
acceleration = config.accelerationLimit(:).';
value = min(max(rawValue, -upper), upper);
value(1, :) = min(max(value(1, :), -acceleration * config.sampleTime), ...
    acceleration * config.sampleTime);
for index = 2:size(value, 1)
    deltaLimit = acceleration * (time(index) - time(index - 1));
    delta = min(max(value(index, :) - value(index - 1, :), ...
        -deltaLimit), deltaLimit);
    value(index, :) = value(index - 1, :) + delta;
end
end

function [time, data] = readTimeseries(value)
% readTimeseries - 将模型记录信号整理为 N×6
if ~isa(value, 'timeseries')
    error('evaluateSimscapeLengthCascadeControl:InvalidSignal', ...
        '仿真输出必须为 timeseries。');
end
time = value.Time(:);
data = squeeze(value.Data);
if size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
if size(data, 1) ~= numel(time) || size(data, 2) ~= 6
    error('evaluateSimscapeLengthCascadeControl:InvalidSignalSize', ...
        '仿真信号必须可整理为 N×6，当前尺寸为 %s。', mat2str(size(data)));
end
[time, uniqueIndex] = unique(time, 'stable');
data = data(uniqueIndex, :);
end

function derivative = differentiate(time, value)
% differentiate - 计算规则时间网格上的数值导数
derivative = zeros(size(value));
for columnIndex = 1:size(value, 2)
    derivative(:, columnIndex) = gradient(value(:, columnIndex), time);
end
end
