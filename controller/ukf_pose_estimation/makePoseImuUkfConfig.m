function config = makePoseImuUkfConfig(model, sampleTime, anchorPose, overrides)
% makePoseImuUkfConfig - 构造相对编码器与 IMU UKF 配置
arguments
    model struct
    sampleTime (1, 1) double {mustBePositive, mustBeFinite}
    anchorPose (6, 1) double {mustBeFinite}
    overrides struct = struct()
end

config = struct();
config.variant = "pose_bias_ukf";
config.model = model;
config.sampleTime = sampleTime;
config.anchorPose = anchorPose;
config.anchorLength = sgpIK(anchorPose, model).L;
config.measurementFields = ["relativeLength", "orientation"];
config.encoderNoiseStd = 1e-3;
config.orientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
config.accelerationNoiseStd = 9.80665e-3;
config.angularVelocityNoiseStd = deg2rad(0.07);
config.legLengthBiasRandomWalkStd = 1e-6 * ones(6, 1);
config.accelerometerBiasRandomWalkStd = 9.80665e-6 * [10; 10; 30];
config.gyroBiasRandomWalkStd = deg2rad([5; 5; 5] / 3600);
config.initialPositionStd = 5e-4;
config.initialOrientationStd = deg2rad([0.1; 0.1; 0.5]);
config.initialTranslationVelocityStd = 0.02;
config.initialRotationVelocityStd = deg2rad(0.5);
config.initialLegLengthBiasStd = 2e-3 * ones(6, 1);
config.poseLegBiasHomeResidualThreshold = 1e-3;
config.poseLegBiasNominalInitialStd = 5e-5 * ones(6, 1);
config.initialAccelerometerBiasStd = 9.80665 * [15; 15; 35] * 1e-6;
config.initialGyroBiasStd = deg2rad([8; 8; 8] / 3600);

fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(config, fields{index})
        error('makePoseImuUkfConfig:UnknownOverride', ...
            '未知 UKF 配置项：%s。', fields{index});
    end
    config.(fields{index}) = overrides.(fields{index});
end

config.variant = string(config.variant);
if config.variant == "position_bias_ukf"
    config.measurementFields = "relativeLength";
elseif config.variant == "pose_bias_ukf" || config.variant == "pose_leg_bias_ukf"
    config.measurementFields = ["relativeLength", "orientation"];
else
    error('makePoseImuUkfConfig:UnknownVariant', ...
        '未知 UKF 变体：%s。', config.variant);
end
end
