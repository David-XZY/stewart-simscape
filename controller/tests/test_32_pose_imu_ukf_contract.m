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

biasConfig = makePoseImuUkfConfig(model, sampleTime, anchorPose, struct( ...
    'variant', "position_bias_ukf"));
biasEstimator = initializePoseImuUkf(biasConfig);
biasSample = sample;
[biasEstimator, biasOutput] = stepPoseImuUkf(biasEstimator, biasSample);

assert(numel(biasEstimator.filter.State) == 15);
assert(isequal(biasConfig.measurementFields, "relativeLength"));
assert(max(abs(biasOutput.pose(4:6) - anchorPose(4:6))) < 1e-12);
assert(isfield(biasOutput, 'legLengthBias'));
assert(numel(biasOutput.legLengthBias) == 6);
assert(isfield(biasOutput, 'accelerometerBias'));
assert(~any(biasConfig.measurementFields == "orientation"));

poseLegBiasConfig = makePoseImuUkfConfig(model, sampleTime, anchorPose, struct( ...
    'variant', "pose_leg_bias_ukf"));
assert(abs(poseLegBiasConfig.poseLegBiasHomeResidualThreshold - 1e-3) < eps);
assert(max(abs(poseLegBiasConfig.poseLegBiasNominalInitialStd - 5e-5)) < eps);
poseLegBiasEstimator = initializePoseImuUkf(poseLegBiasConfig);
[poseLegBiasEstimator, poseLegBiasOutput] = stepPoseImuUkf(poseLegBiasEstimator, sample);

assert(numel(poseLegBiasEstimator.filter.State) == 24);
assert(isequal(poseLegBiasConfig.measurementFields, ["relativeLength", "orientation"]));
assert(max(abs(poseLegBiasOutput.pose - anchorPose)) < 1e-5);
assert(max(abs(poseLegBiasOutput.velocity)) < 1e-4);
assert(isfield(poseLegBiasOutput, 'legLengthBias'));
assert(numel(poseLegBiasOutput.legLengthBias) == 6);
assert(max(abs(poseLegBiasOutput.accelerometerBias)) < 1e-7);
assert(max(abs(poseLegBiasOutput.gyroBias)) < 1e-7);
end
