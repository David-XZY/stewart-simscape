function test_33_pose_imu_sensor_contract
% test_33_pose_imu_sensor_contract - 验证实物可部署的 UKF 传感器契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'actuator_identification'));

model = buildOptModelCustom();
time = 0:0.005:0.05;
pose = repmat(model.qHome, 1, numel(time));
pose(1, :) = pose(1, :) + 0.01 * (1 - cos(2 * pi * time));
velocity = zeros(size(pose));
velocity(1, :) = 0.01 * 2 * pi * sin(2 * pi * time);
trajectory = struct('t', time, 'qTrue', pose, 'qdTrue', velocity);
measurements = simulatePoseImuSensors(trajectory, model, struct( ...
    'encoderNoiseStd', 0, 'orientationNoiseStd', 0, ...
    'accelerationNoiseStd', 0, 'angularVelocityNoiseStd', 0, ...
    'orientationCalibrationResidual', zeros(3, 1), ...
    'accelerometerCalibrationResidual', zeros(3, 1), ...
    'gyroCalibrationResidual', zeros(3, 1)));

assert(isequal(size(measurements.relativeLength), [6, numel(time)]));
assert(isequal(size(measurements.orientation), [3, numel(time)]));
assert(isequal(size(measurements.worldAcceleration), [3, numel(time)]));
assert(isequal(size(measurements.specificForce), [3, numel(time)]));
assert(isequal(size(measurements.angularVelocity), [3, numel(time)]));
assert(max(abs(measurements.relativeLength(:, 1))) < 1e-12);
assert(max(abs(measurements.orientation(:, 1) - model.qHome(4:6))) < 1e-12);
assert(max(abs(measurements.angularVelocity(:, 1))) < 1e-12);
assert(measurements.sensorModel.homeCalibrationApplied);
assert(~isfield(measurements, 'position'));
assert(~isfield(measurements, 'pose'));
assert(~isfield(measurements, 'absoluteLength'));
end
