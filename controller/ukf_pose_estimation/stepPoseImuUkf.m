function [estimator, output] = stepPoseImuUkf(estimator, sample)
% stepPoseImuUkf - 使用 IMU 预测并以相对腿长和姿态校正 UKF
requiredFields = {'relativeLength', 'orientation', 'acceleration', ...
    'angularVelocity', 'accelerationMode'};
for index = 1:numel(requiredFields)
    if ~isfield(sample, requiredFields{index})
        error('stepPoseImuUkf:MissingField', '测量缺少字段：%s。', requiredFields{index});
    end
end

mode = string(sample.accelerationMode);
if mode == "world"
    modeFlag = 0;
elseif mode == "specificForce"
    modeFlag = 1;
else
    error('stepPoseImuUkf:InvalidAccelerationMode', ...
        'accelerationMode 必须为 world 或 specificForce。');
end

imuInput = [sample.acceleration(:); sample.angularVelocity(:); modeFlag; sample.orientation(:)];
if estimator.config.variant == "pose_leg_bias_ukf" && estimator.sampleCount == 0
    estimator = gateInitialLegLengthBiasCovariance(estimator, sample.relativeLength(:));
end
predict(estimator.filter, imuInput);
predictedState = estimator.filter.State;
if estimator.config.variant == "position_bias_ukf"
    correct(estimator.filter, sample.relativeLength(:), sample.orientation(:));
else
    correct(estimator.filter, [sample.relativeLength(:); sample.orientation(:)]);
end
estimator.sampleCount = estimator.sampleCount + 1;

state = estimator.filter.State;
output = struct();
if estimator.config.variant == "position_bias_ukf"
    orientation = sample.orientation(:);
    rpyRate = rpyRateMapZYX(orientation) \ sample.angularVelocity(:);
    output.predictedPose = [predictedState(1:3); orientation];
    output.predictedVelocity = [predictedState(4:6); rpyRate];
    output.pose = [state(1:3); orientation];
    output.velocity = [state(4:6); rpyRate];
    output.legLengthBias = state(7:12);
    output.accelerometerBias = state(13:15);
    output.gyroBias = zeros(3, 1);
elseif estimator.config.variant == "pose_leg_bias_ukf"
    output.predictedPose = predictedState(1:6);
    output.predictedVelocity = predictedState(7:12);
    output.pose = state(1:6);
    output.velocity = state(7:12);
    output.legLengthBias = state(13:18);
    output.accelerometerBias = state(19:21);
    output.gyroBias = state(22:24);
else
    output.predictedPose = predictedState(1:6);
    output.predictedVelocity = predictedState(7:12);
    output.pose = state(1:6);
    output.velocity = state(7:12);
    output.accelerometerBias = state(13:15);
    output.gyroBias = state(16:18);
end
end

function estimator = gateInitialLegLengthBiasCovariance(estimator, relativeLength)
homeResidualRms = sqrt(mean(relativeLength.^2));
if homeResidualRms > estimator.config.poseLegBiasHomeResidualThreshold
    return;
end

stateCovariance = estimator.filter.StateCovariance;
legBiasIndex = 13:18;
nominalVariance = estimator.config.poseLegBiasNominalInitialStd(:).^2;
stateCovariance(legBiasIndex, legBiasIndex) = diag(nominalVariance);
estimator.filter.StateCovariance = stateCovariance;
end
