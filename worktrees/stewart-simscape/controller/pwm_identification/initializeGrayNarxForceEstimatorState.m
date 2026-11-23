function state = initializeGrayNarxForceEstimatorState(identified)
% initializeGrayNarxForceEstimatorState - 初始化灰箱电气状态和 NARX 残差状态
state = struct();
state.gray = initializeHighFidelityPwmActuatorState(identified.gray);
state.previousResidual = zeros(identified.axisCount, 1);
state.previousResidual2 = zeros(identified.axisCount, 1);
state.previousSpeed = zeros(identified.axisCount, 1);
state.previousPwm = zeros(identified.axisCount, 1);
state.previousGrayForce = zeros(identified.axisCount, 1);
end
