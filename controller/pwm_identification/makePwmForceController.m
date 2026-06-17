function controller = makePwmForceController(mode, model)
% makePwmForceController - 构造高保真基准或灰箱辨识力内环
mode = validatestring(mode, {'oracle', 'identified'});
controller = struct();
controller.type = ['pwm-force-', mode];
controller.mode = mode;
controller.pwmRateLimit = 500;
controller.kpPwmPerNewton = 0.35;
controller.kiPwmPerNewtonSecond = 4.0;
controller.integralLimit = 1200;
if strcmp(mode, 'oracle')
    controller.feedbackSignal = 'trueForce';
    controller.inverseModel = model;
else
    controller.feedbackSignal = 'estimatedForce';
    controller.inverseModel = model.gray;
end
controller.sampleTime = controller.inverseModel.sampleTime;
controller.pwmMax = controller.inverseModel.pwmMax;
end
