function benchmark = benchmarkPwmPoseEstimators(trajectory, overrides)
% benchmarkPwmPoseEstimators - 比较运动学重建与两种 IMU 辅助 UKF
arguments
    trajectory struct
    overrides struct = struct()
end

options = defaultOptions(overrides);
model = buildOptModelCustom();
time = trajectory.t(:).';
sampleTime = median(diff(time));
measurements = simulatePoseImuSensors(trajectory, model, sensorOverrides(options));

sampleCount = numel(time);
kinematicPose = zeros(6, sampleCount);
kinematicVelocity = zeros(6, sampleCount);
kinematicPose(:, 1) = measurements.anchorPose;
for index = 2:sampleCount
    absoluteLength = measurements.anchorLength + measurements.relativeLength(:, index);
    kinematicPose(:, index) = estimateKinematicPose( ...
        absoluteLength, measurements.orientation(:, index), ...
        kinematicPose(:, index - 1), model, options.translationIterations);
    rawVelocity = (kinematicPose(:, index) - kinematicPose(:, index - 1)) / sampleTime;
    kinematicVelocity(:, index) = options.kinematicVelocityFilterAlpha * ...
        kinematicVelocity(:, index - 1) + ...
        (1 - options.kinematicVelocityFilterAlpha) * rawVelocity;
end

ukfOverrides = struct( ...
    'variant', options.ukfVariant, ...
    'encoderNoiseStd', options.ukfEncoderNoiseScale * options.encoderNoiseStd, ...
    'orientationNoiseStd', options.ukfOrientationNoiseStd, ...
    'accelerationNoiseStd', options.accelerationNoiseStd, ...
    'angularVelocityNoiseStd', options.angularVelocityNoiseStd);
ukfWorld = estimatePoseImuUkfSeries(measurements, model, "world", ukfOverrides);
ukfSpecificForce = estimatePoseImuUkfSeries( ...
    measurements, model, "specificForce", ukfOverrides);
if options.accelerationMode == "world"
    selectedUkf = ukfWorld;
else
    selectedUkf = ukfSpecificForce;
end

metrics = buildMetrics(trajectory, kinematicPose, kinematicVelocity, ...
    selectedUkf.pose, selectedUkf.velocity, model, time, options.settlingTime);
metrics.world = buildMetrics(trajectory, kinematicPose, kinematicVelocity, ...
    ukfWorld.pose, ukfWorld.velocity, model, time, options.settlingTime);
metrics.specificForce = buildMetrics(trajectory, kinematicPose, kinematicVelocity, ...
    ukfSpecificForce.pose, ukfSpecificForce.velocity, model, time, options.settlingTime);

benchmark = struct();
benchmark.t = time;
benchmark.truth = struct('pose', trajectory.qTrue, 'velocity', trajectory.qdTrue);
benchmark.measurements = measurements;
benchmark.kinematic = struct('pose', kinematicPose, 'velocity', kinematicVelocity);
benchmark.ukf = selectedUkf;
benchmark.ukfWorld = ukfWorld;
benchmark.ukfSpecificForce = ukfSpecificForce;
benchmark.sensorModel = struct( ...
    'usesNoisyOrientation', any(options.orientationNoiseStd > 0), ...
    'usesFullPoseMeasurement', false, ...
    'usesPositionMeasurement', false, ...
    'usesRelativeEncoder', true, ...
    'encoderResolution', options.encoderResolution, ...
    'orientationResolution', options.orientationResolution, ...
    'encoderNoiseStd', options.encoderNoiseStd, ...
    'orientationNoiseStd', options.orientationNoiseStd, ...
    'accelerationNoiseStd', options.accelerationNoiseStd, ...
    'angularVelocityNoiseStd', options.angularVelocityNoiseStd);
benchmark.metrics = metrics;
benchmark.options = options;
end

function options = defaultOptions(overrides)
options = struct();
options.randomSeed = 41;
options.encoderResolution = 1 / 16500;
options.encoderNoiseStd = 1e-3;
options.relativeLengthBias = zeros(6, 1);
options.relativeLengthDriftRate = zeros(6, 1);
options.encoderScaleError = zeros(6, 1);
options.orientationResolution = deg2rad(0.0055);
options.orientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
options.ukfOrientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
options.ukfVariant = "pose_leg_bias_ukf";
options.orientationBias = deg2rad([0.1; -0.1; 0.5]);
options.accelerationNoiseStd = 9.80665e-3;
options.accelerometerBias = 9.80665 * [20; -20; 40] * 1e-3;
options.angularVelocityNoiseStd = deg2rad(0.07);
options.gyroBias = deg2rad([0.5; -0.5; 1.0]);
options.translationIterations = 4;
options.kinematicVelocityFilterAlpha = 0.75;
options.ukfEncoderNoiseScale = 2.0;
options.accelerationMode = "specificForce";
options.settlingTime = 0.2;
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('benchmarkPwmPoseEstimators:UnknownOverride', ...
            '未知位姿估计基准配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end
end

function overrides = sensorOverrides(options)
overrides = struct( ...
    'randomSeed', options.randomSeed, ...
    'encoderResolution', options.encoderResolution, ...
    'encoderNoiseStd', options.encoderNoiseStd, ...
    'relativeLengthBias', options.relativeLengthBias, ...
    'relativeLengthDriftRate', options.relativeLengthDriftRate, ...
    'encoderScaleError', options.encoderScaleError, ...
    'orientationResolution', options.orientationResolution, ...
    'orientationNoiseStd', options.orientationNoiseStd, ...
    'orientationBias', options.orientationBias, ...
    'accelerationNoiseStd', options.accelerationNoiseStd, ...
    'accelerometerBias', options.accelerometerBias, ...
    'angularVelocityNoiseStd', options.angularVelocityNoiseStd, ...
    'gyroBias', options.gyroBias);
end

function pose = estimateKinematicPose(lengthMeasurement, orientationMeasurement, ...
        previousPose, model, iterationCount)
pose = [previousPose(1:3); orientationMeasurement(:)];
for iteration = 1:iterationCount
    kin = sgpIK(pose, model);
    pose(1:3) = pose(1:3) + kin.u.' \ (lengthMeasurement - kin.L);
end
end

function metrics = buildMetrics(trajectory, kinematicPose, kinematicVelocity, ...
        ukfPose, ukfVelocity, model, time, settlingTime)
kinematicPoseError = kinematicPose - trajectory.qTrue;
ukfPoseError = ukfPose - trajectory.qTrue;
kinematicVelocityError = kinematicVelocity - trajectory.qdTrue;
ukfVelocityError = ukfVelocity - trajectory.qdTrue;
characteristicLength = model.singularity.characteristicLength;
kinematicEquivalentVelocityError = [kinematicVelocityError(1:3, :); ...
    characteristicLength * kinematicVelocityError(4:6, :)];
ukfEquivalentVelocityError = [ukfVelocityError(1:3, :); ...
    characteristicLength * ukfVelocityError(4:6, :)];

sampleCount = size(trajectory.qTrue, 2);
kinematicLegSpeedError = zeros(6, sampleCount);
ukfLegSpeedError = zeros(6, sampleCount);
for index = 1:sampleCount
    trueLegSpeed = sgpJacobian(trajectory.qTrue(:, index), model).Jq * trajectory.qdTrue(:, index);
    kinematicLegSpeed = sgpJacobian(kinematicPose(:, index), model).Jq * kinematicVelocity(:, index);
    ukfLegSpeed = sgpJacobian(ukfPose(:, index), model).Jq * ukfVelocity(:, index);
    kinematicLegSpeedError(:, index) = kinematicLegSpeed - trueLegSpeed;
    ukfLegSpeedError(:, index) = ukfLegSpeed - trueLegSpeed;
end

settled = time >= settlingTime;
relativeUkfPoseError = ukfPoseError - ukfPoseError(:, 1);
metrics = struct();
metrics.kinematicTranslationRms = sqrt(mean(kinematicPoseError(1:3, :).^2, 'all'));
metrics.ukfTranslationRms = sqrt(mean(ukfPoseError(1:3, :).^2, 'all'));
metrics.kinematicTranslationPeak = max(abs(kinematicPoseError(1:3, :)), [], 'all');
metrics.ukfTranslationPeak = max(abs(ukfPoseError(1:3, :)), [], 'all');
metrics.ukfSettledRelativeTranslationRms = ...
    sqrt(mean(relativeUkfPoseError(1:3, settled).^2, 'all'));
metrics.ukfSettledRelativeTranslationPeak = ...
    max(abs(relativeUkfPoseError(1:3, settled)), [], 'all');
metrics.kinematicRotationRms = sqrt(mean(kinematicPoseError(4:6, :).^2, 'all'));
metrics.ukfRotationRms = sqrt(mean(ukfPoseError(4:6, :).^2, 'all'));
metrics.kinematicVelocityRms = sqrt(mean(kinematicEquivalentVelocityError.^2, 'all'));
metrics.ukfVelocityRms = sqrt(mean(ukfEquivalentVelocityError.^2, 'all'));
metrics.kinematicLegSpeedRms = sqrt(mean(kinematicLegSpeedError.^2, 'all'));
metrics.ukfLegSpeedRms = sqrt(mean(ukfLegSpeedError.^2, 'all'));
end
