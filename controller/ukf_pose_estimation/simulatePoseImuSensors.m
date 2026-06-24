function measurements = simulatePoseImuSensors(trajectory, model, overrides)
% simulatePoseImuSensors - 从仿真真值生成相对编码器、姿态和 IMU 测量
arguments
    trajectory struct
    model struct
    overrides struct = struct()
end

options = struct();
options.randomSeed = 41;
options.encoderResolution = 1 / 16500;
options.encoderNoiseStd = 1e-3;
options.relativeLengthBias = zeros(6, 1);
options.relativeLengthDriftRate = zeros(6, 1);
options.encoderScaleError = zeros(6, 1);
options.orientationResolution = deg2rad(0.0055);
options.orientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
options.orientationBias = deg2rad([0.1; -0.1; 0.5]);
options.accelerationNoiseStd = 9.80665e-3;
options.accelerometerBias = 9.80665 * [20; -20; 40] * 1e-3;
options.angularVelocityNoiseStd = deg2rad(0.07);
options.gyroBias = deg2rad([0.5; -0.5; 1.0]);
options.homeCalibrationApplied = true;
options.orientationCalibrationResidual = zeros(3, 1);
options.accelerometerCalibrationResidual = 9.80665 * [15; -15; 35] * 1e-6;
options.gyroCalibrationResidual = deg2rad([8; -8; 8] / 3600);

fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('simulatePoseImuSensors:UnknownOverride', ...
            '未知传感器仿真配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end

rng(options.randomSeed);
time = trajectory.t(:).';
pose = trajectory.qTrue;
velocity = trajectory.qdTrue;
sampleCount = numel(time);
sampleTime = median(diff(time));
translationAcceleration = finiteDifferenceRows(velocity(1:3, :), sampleTime);

anchorLength = sgpIK(pose(:, 1), model).L;
relativeLength = zeros(6, sampleCount);
angularVelocity = zeros(3, sampleCount);
specificForce = zeros(3, sampleCount);
for sampleIndex = 1:sampleCount
    relativeLength(:, sampleIndex) = sgpIK(pose(:, sampleIndex), model).L - anchorLength;
    angularVelocity(:, sampleIndex) = ...
        rpyRateMapZYX(pose(4:6, sampleIndex)) * velocity(4:6, sampleIndex);
    rotation = rpy2rotmZYX(pose(4:6, sampleIndex));
    specificForce(:, sampleIndex) = ...
        rotation.' * (translationAcceleration(:, sampleIndex) - model.g);
end

relativeLength = (1 + options.encoderScaleError(:)) .* relativeLength + ...
    options.relativeLengthBias(:) + options.relativeLengthDriftRate(:) .* (time - time(1));
relativeLength = relativeLength + options.encoderNoiseStd * randn(size(relativeLength));
relativeLength = round(relativeLength / options.encoderResolution) * options.encoderResolution;
orientationBias = options.orientationBias;
accelerometerBias = options.accelerometerBias;
gyroBias = options.gyroBias;
if options.homeCalibrationApplied
    orientationBias = options.orientationCalibrationResidual;
    accelerometerBias = options.accelerometerCalibrationResidual;
    gyroBias = options.gyroCalibrationResidual;
end
orientation = pose(4:6, :) + orientationBias + ...
    options.orientationNoiseStd .* randn(3, sampleCount);
orientation = round(orientation / options.orientationResolution) * options.orientationResolution;
worldAcceleration = translationAcceleration + accelerometerBias + ...
    options.accelerationNoiseStd * randn(3, sampleCount);
specificForce = specificForce + accelerometerBias + ...
    options.accelerationNoiseStd * randn(3, sampleCount);
angularVelocity = angularVelocity + gyroBias + ...
    options.angularVelocityNoiseStd * randn(3, sampleCount);

measurements = struct();
measurements.relativeLength = relativeLength;
measurements.orientation = orientation;
measurements.worldAcceleration = worldAcceleration;
measurements.specificForce = specificForce;
measurements.angularVelocity = angularVelocity;
measurements.anchorPose = pose(:, 1);
measurements.anchorLength = anchorLength;
measurements.sampleTime = sampleTime;
measurements.sensorModel = options;
end

function derivative = finiteDifferenceRows(value, sampleTime)
derivative = zeros(size(value));
if size(value, 2) < 2
    return;
end
derivative(:, 1) = (value(:, 2) - value(:, 1)) / sampleTime;
derivative(:, end) = (value(:, end) - value(:, end - 1)) / sampleTime;
if size(value, 2) > 2
    derivative(:, 2:end - 1) = ...
        (value(:, 3:end) - value(:, 1:end - 2)) / (2 * sampleTime);
end
end
