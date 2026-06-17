function test_32_pose_imu_ukf_contract
% test_32_pose_imu_ukf_contract - 验证相对编码器与 IMU UKF 的公共接口
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));

model = buildOptModelCustom();
sampleTime = 0.005;
anchorPose = model.qHome;
config = makePoseImuUkfConfig(model, sampleTime, anchorPose);
estimator = initializePoseImuUkf(config);

sample = struct();
sample.relativeLength = zeros(6, 1);
sample.orientation = anchorPose(4:6);
sample.acceleration = zeros(3, 1);
sample.angularVelocity = zeros(3, 1);
sample.accelerationMode = "world";
[estimator, output] = stepPoseImuUkf(estimator, sample);

assert(numel(estimator.filter.State) == 18);
% 实物航向角初始不确定度会通过非线性腿长观测产生微小的一步均值偏移。
assert(max(abs(output.pose - anchorPose)) < 1e-5);
assert(max(abs(output.velocity)) < 1e-4);
assert(isfield(output, 'predictedPose'));
assert(isfield(output, 'predictedVelocity'));
assert(max(abs(output.accelerometerBias)) < 1e-7);
assert(max(abs(output.gyroBias)) < 1e-7);
assert(isequal(config.anchorLength, sgpIK(anchorPose, model).L));
assert(isequal(config.measurementFields, ["relativeLength", "orientation"]));
assert(~any(config.measurementFields == "position"));
assert(~any(config.measurementFields == "pose"));
end
