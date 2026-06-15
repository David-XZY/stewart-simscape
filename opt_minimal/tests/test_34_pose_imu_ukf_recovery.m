function test_34_pose_imu_ukf_recovery
% test_34_pose_imu_ukf_recovery - 验证双加速度模式下的无噪声运动恢复
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'actuator_identification'));

model = buildOptModelCustom();
time = 0:0.005:1.0;
amplitude = [0.008; 0.006; 0.012; deg2rad(0.8); deg2rad(0.6); deg2rad(1.0)];
frequency = [0.7; 0.9; 0.6; 0.8; 0.65; 0.75];
pose = repmat(model.qHome, 1, numel(time));
velocity = zeros(size(pose));
for axisIndex = 1:6
    pose(axisIndex, :) = pose(axisIndex, :) + amplitude(axisIndex) * ...
        (1 - cos(2 * pi * frequency(axisIndex) * time));
    velocity(axisIndex, :) = amplitude(axisIndex) * 2 * pi * frequency(axisIndex) * ...
        sin(2 * pi * frequency(axisIndex) * time);
end
trajectory = struct('t', time, 'qTrue', pose, 'qdTrue', velocity);
measurements = simulatePoseImuSensors(trajectory, model, struct( ...
    'encoderNoiseStd', 0, 'orientationNoiseStd', 0, ...
    'orientationBias', zeros(3, 1), 'accelerationNoiseStd', 0, ...
    'angularVelocityNoiseStd', 0, 'accelerometerBias', zeros(3, 1), ...
    'gyroBias', zeros(3, 1)));

worldEstimate = estimatePoseImuUkfSeries(measurements, model, "world");
specificForceEstimate = estimatePoseImuUkfSeries(measurements, model, "specificForce");
settled = time >= 0.2;

assert(max(abs(worldEstimate.pose(1:3, settled) - pose(1:3, settled)), [], 'all') < 5e-4);
assert(max(abs(specificForceEstimate.pose(1:3, settled) - pose(1:3, settled)), [], 'all') < 5e-4);
assert(max(abs(worldEstimate.pose(4:6, settled) - pose(4:6, settled)), [], 'all') < deg2rad(0.03));
assert(max(abs(specificForceEstimate.pose(4:6, settled) - pose(4:6, settled)), [], 'all') < deg2rad(0.03));
end
