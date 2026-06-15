function estimate = estimatePoseImuUkfSeries(measurements, model, accelerationMode, overrides)
% estimatePoseImuUkfSeries - 用公共 UKF 批量处理相对编码器与 IMU 测量
arguments
    measurements struct
    model struct
    accelerationMode (1, 1) string
    overrides struct = struct()
end

config = makePoseImuUkfConfig( ...
    model, measurements.sampleTime, measurements.anchorPose, overrides);
estimator = initializePoseImuUkf(config);
sampleCount = size(measurements.relativeLength, 2);
pose = zeros(6, sampleCount);
velocity = zeros(6, sampleCount);
accelerometerBias = zeros(3, sampleCount);
gyroBias = zeros(3, sampleCount);

for sampleIndex = 1:sampleCount
    sample = struct();
    sample.relativeLength = measurements.relativeLength(:, sampleIndex);
    sample.orientation = measurements.orientation(:, sampleIndex);
    sample.angularVelocity = measurements.angularVelocity(:, sampleIndex);
    sample.accelerationMode = accelerationMode;
    if accelerationMode == "world"
        sample.acceleration = measurements.worldAcceleration(:, sampleIndex);
    elseif accelerationMode == "specificForce"
        sample.acceleration = measurements.specificForce(:, sampleIndex);
    else
        error('estimatePoseImuUkfSeries:InvalidAccelerationMode', ...
            'accelerationMode 必须为 world 或 specificForce。');
    end
    [estimator, output] = stepPoseImuUkf(estimator, sample);
    pose(:, sampleIndex) = output.pose;
    velocity(:, sampleIndex) = output.velocity;
    accelerometerBias(:, sampleIndex) = output.accelerometerBias;
    gyroBias(:, sampleIndex) = output.gyroBias;
end

estimate = struct();
estimate.pose = pose;
estimate.velocity = velocity;
estimate.accelerometerBias = accelerometerBias;
estimate.gyroBias = gyroBias;
estimate.accelerationMode = accelerationMode;
estimate.config = config;
end
