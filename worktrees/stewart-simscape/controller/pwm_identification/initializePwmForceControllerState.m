function state = initializePwmForceControllerState(controller)
% initializePwmForceControllerState - 初始化力内环积分与限速状态
state = struct();
state.integralError = zeros(6, 1);
state.previousPwm = zeros(6, 1);
state.sampleTime = controller.sampleTime;
end
