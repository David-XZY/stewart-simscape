function options = makePwmClosedLoopRefinementOptions()
% makePwmClosedLoopRefinementOptions - 构造与部署隔离的持续激励闭环精炼配置
controlOverrides = struct();
controlOverrides.poseEstimatorMode = "ukf";
controlOverrides.sensorNoiseEnabled = true;
controlOverrides.encoderNoiseStd = 5e-4;
controlOverrides.accelerationMode = "specificForce";
controlOverrides.forceKpPwmPerNewton = 0.5;
controlOverrides.forceKiPwmPerNewtonSecond = 5;
controlOverrides.pwmRateLimit = 2000;
controlOverrides.translationGain = [5.0e4; 5.0e4; 1.5e5];
options = struct('controlOverrides', controlOverrides);
end
