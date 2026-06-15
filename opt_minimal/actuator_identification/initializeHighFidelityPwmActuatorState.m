function state = initializeHighFidelityPwmActuatorState(actuator)
% initializeHighFidelityPwmActuatorState - 初始化高保真执行器电气状态
state = struct();
state.current = zeros(actuator.axisCount, 1);
state.previousAppliedPwm = zeros(actuator.axisCount, 1);
end
