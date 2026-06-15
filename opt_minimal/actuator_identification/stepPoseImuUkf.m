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

imuInput = [sample.acceleration(:); sample.angularVelocity(:); modeFlag];
predict(estimator.filter, imuInput);
correct(estimator.filter, [sample.relativeLength(:); sample.orientation(:)]);
estimator.sampleCount = estimator.sampleCount + 1;

state = estimator.filter.State;
output = struct();
output.pose = state(1:6);
output.velocity = state(7:12);
output.accelerometerBias = state(13:15);
output.gyroBias = state(16:18);
end
