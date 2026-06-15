function test_35_pose_imu_ukf_precision
% test_35_pose_imu_ukf_precision - 验证极限精度和初始位置误差敏感性
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'actuator_identification'));

model = buildOptModelCustom();
time = 0:0.005:1.5;
amplitude = [0.012; 0.010; 0.018; deg2rad(0.8); deg2rad(0.6); deg2rad(1.0)];
frequency = [0.45; 0.55; 0.40; 0.50; 0.42; 0.48];
pose = repmat(model.qHome, 1, numel(time));
velocity = zeros(size(pose));
for axisIndex = 1:6
    pose(axisIndex, :) = pose(axisIndex, :) + amplitude(axisIndex) * ...
        (1 - cos(2 * pi * frequency(axisIndex) * time));
    velocity(axisIndex, :) = amplitude(axisIndex) * 2 * pi * frequency(axisIndex) * ...
        sin(2 * pi * frequency(axisIndex) * time);
end
trajectory = struct('t', time, 'qTrue', pose, 'qdTrue', velocity);

benchmark1mm = benchmarkPwmPoseEstimators(trajectory, struct('encoderNoiseStd', 1e-3));
benchmark05mm = benchmarkPwmPoseEstimators(trajectory, struct('encoderNoiseStd', 5e-4));
assert(benchmark05mm.metrics.ukfSettledRelativeTranslationRms <= 2.5e-4);
assert(benchmark05mm.metrics.ukfSettledRelativeTranslationPeak <= 1e-3);
assert(benchmark05mm.metrics.ukfVelocityRms <= 5e-3);
assert(benchmark05mm.metrics.ukfRotationRms <= deg2rad(0.03));
assert(benchmark1mm.metrics.ukfSettledRelativeTranslationRms >= ...
    benchmark05mm.metrics.ukfSettledRelativeTranslationRms);

shiftedMeasurements = benchmark05mm.measurements;
anchorError = [3e-3; 0; 0];
shiftedMeasurements.anchorPose(1:3) = shiftedMeasurements.anchorPose(1:3) + anchorError;
shiftedEstimate = estimatePoseImuUkfSeries(shiftedMeasurements, model, "specificForce", struct( ...
    'encoderNoiseStd', 2 * 5e-4, 'accelerationNoiseStd', 0.02));
absoluteError = shiftedEstimate.pose(1:3, :) - pose(1:3, :);
relativeError = absoluteError - absoluteError(:, 1);
settled = time >= 0.2;
assert(abs(mean(absoluteError(1, settled))) >= 2e-3);
assert(sqrt(mean(relativeError(:, settled).^2, 'all')) <= 5e-4);
end
