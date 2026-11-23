function design = designLqiPoseForceController(cartesianPlant, characteristicLength, config)
% designLqiPoseForceController - 生成位姿误差到广义力的 LQI 输出反馈补偿器
arguments
    cartesianPlant
    characteristicLength (1,1) double {mustBePositive}
    config struct
end

plant = minreal(ss(cartesianPlant));
if ~isequal(size(plant), [6, 6])
    error('designLqiPoseForceController:InvalidPlantSize', ...
        'LQI 设计对象必须为 6x6，当前为 %dx%d。', size(plant, 1), size(plant, 2));
end

[A, B, C, D] = ssdata(plant);
if norm(D, 'fro') > 1e-9
    error('designLqiPoseForceController:NonStrictPlant', ...
        '当前 LQI 输出反馈实现要求力到位姿对象为严格真有理。');
end
stateCount = size(A, 1);
errorPlant = ss(A, B, -C, zeros(6, 6));
poseScale = [
    config.lqiTranslationScale * ones(3, 1)
    config.lqiRotationScale * ones(3, 1)];
integralScale = [
    config.lqiIntegralScale * ones(3, 1)
    (config.lqiIntegralScale / characteristicLength) * ones(3, 1)];
velocityScale = config.lqiVelocityScale * ones(6, 1);

stateWeight = C.' * diag(1 ./ poseScale.^2) * C + 1e-9 * eye(stateCount);
integralWeight = diag(1 ./ integralScale.^2);
controlWeight = diag((1 ./ (config.lqiControlScale * ones(6, 1))).^2);
fullStateGain = lqi(errorPlant, blkdiag(stateWeight, integralWeight), controlWeight);
stateGain = fullStateGain(:, 1:stateCount);
integralGain = fullStateGain(:, stateCount + (1:6));

observerStateWeight = eye(stateCount);
observerOutputWeight = diag(velocityScale.^2);
observerGain = lqr(A.', (-C).', observerStateWeight, observerOutputWeight).';

leakRate = 2 * pi * config.lqiIntegralLeakHz;
antiWindupTime = config.lqiAntiWindupTimeConstant;
integralAntiWindupGain = -pinv(integralGain) / antiWindupTime;
% MATLAB lqi 的增广积分状态等价于对输出取负积分；第二组输入为实际执行量与原始命令的差值。
controllerA = [
    A + observerGain * C - B * stateGain, -B * integralGain
    zeros(6, stateCount), -leakRate * eye(6)];
controllerB = [
    observerGain, B
    -eye(6), integralAntiWindupGain];
controllerC = [-stateGain, -integralGain];
controllerD = zeros(6, 12);
Kx = ss(controllerA, controllerB, controllerC, controllerD);

closedLoopA = [
    A, B * controllerC
    controllerB(:, 1:6) * (-C), controllerA];
closedLoopPoles = eig(closedLoopA);
stable = all(real(closedLoopPoles) < -1e-7);

design = struct();
design.feedbackLaw = "lqi-output";
design.Kx = minreal(Kx);
design.stateGain = stateGain;
design.integralGain = integralGain;
design.observerGain = observerGain;
design.integralAntiWindupGain = integralAntiWindupGain;
design.integratorCount = 6;
design.feedbackInputCount = 12;
design.closedLoopPoles = closedLoopPoles;
design.stable = stable;
design.poseScale = poseScale;
design.integralScale = integralScale;
design.velocityScale = velocityScale;
design.controlScale = config.lqiControlScale;
design.integralLeakHz = config.lqiIntegralLeakHz;
design.antiWindupTimeConstant = antiWindupTime;

if ~stable
    error('designLqiPoseForceController:DesignRejected', ...
        'LQI 输出反馈闭环未通过线性稳定性检查。');
end
end
