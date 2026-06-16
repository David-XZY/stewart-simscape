function options = makePwmUkfOuterLoopTunedOptions()
% makePwmUkfOuterLoopTunedOptions - 构造真实传感器条件下的 UKF 与外环联合整定配置
options = struct();
options.encoderNoiseStd = 5e-4;
options.orientationResolution = deg2rad(0.0055);
options.orientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
options.accelerationNoiseStd = 9.80665e-3;
options.angularVelocityNoiseStd = deg2rad(0.07);
options.ukfEncoderNoiseScale = 0.2;
options.ukfOrientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
options.ukfAccelerationNoiseStd = 0.02;
options.ukfAngularVelocityNoiseStd = deg2rad(0.07);
options.ukfInitialPositionStd = 5e-4;
options.ukfInitialTranslationVelocityStd = 0.02;
options.ukfWarmupDuration = 0.2;
options.translationGain = [5.6e4; 5.6e4; 1.68e5];
options.translationRateGain = [7.2e3; 7.2e3; 1.8e4];
options.translationIntegralGain = [7.5e4; 7.5e4; 2.0e5];
options.translationIntegralLimit = [5e-3; 5e-3; 5e-3];
options.rotationGain = [2.25e3; 2.25e3; 2.25e3];
options.rotationRateGain = [1.0e3; 1.0e3; 1.0e3];
options.forceCorrectionLimit = 750;
end
