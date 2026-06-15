function test_36_pose_imu_datasheet_parameters
% test_36_pose_imu_datasheet_parameters - 验证 UKF 与传感器仿真采用实物手册参数
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'actuator_identification'));

model = buildOptModelCustom();
sampleTime = 0.005;
config = makePoseImuUkfConfig(model, sampleTime, model.qHome);

gravity = 9.80665;
assert(max(abs(config.orientationNoiseStd - deg2rad([0.1; 0.1; 0.5]))) < eps);
assert(max(abs(config.initialOrientationStd - deg2rad(0.0055))) < eps);
assert(abs(config.accelerationNoiseStd - 1e-3 * gravity) < eps);
assert(abs(config.angularVelocityNoiseStd - deg2rad(0.07)) < eps);
assert(max(abs(config.accelerometerBiasRandomWalkStd - gravity * [10; 10; 30] * 1e-6)) < eps);
assert(max(abs(config.gyroBiasRandomWalkStd - deg2rad([5; 5; 5] / 3600))) < eps);

trajectory = struct('t', [0, sampleTime], ...
    'qTrue', repmat(model.qHome, 1, 2), 'qdTrue', zeros(6, 2));
measurements = simulatePoseImuSensors(trajectory, model);
sensor = measurements.sensorModel;

assert(max(abs(sensor.orientationNoiseStd - deg2rad(0.0055))) < eps);
assert(abs(sensor.accelerationNoiseStd - 1e-3 * gravity) < eps);
assert(max(abs(sensor.accelerometerBias - gravity * [20; -20; 40] * 1e-3)) < eps);
assert(max(abs(sensor.accelerometerCalibrationResidual - ...
    gravity * [15; -15; 35] * 1e-6)) < eps);
assert(abs(sensor.angularVelocityNoiseStd - deg2rad(0.07)) < eps);
assert(max(abs(sensor.gyroBias - deg2rad([0.5; -0.5; 1.0]))) < eps);
assert(max(abs(sensor.gyroCalibrationResidual - deg2rad([8; -8; 8] / 3600))) < eps);
end
