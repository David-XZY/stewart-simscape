function F = inverseDynamicsCompositeRigidBody(q, qd, qdd, model)
% inverseDynamicsCompositeRigidBody - 合成刚体逆动力学驱动力初值
%
% 文件用途：
%   根据给定 q、qd、qdd 计算实现该运动所需的六条支链轴向驱动力。
%   该函数只用于初值生成与诊断，正式优化中 F 是控制决策变量。
%
% 输入参数：
%   q/qd/qdd [6x1] - 位姿、速度和加速度。
%   model struct - Stewart 模型。
%
% 输出参数：
%   F [6x1] - 逆动力学支链驱动力估计。
%
% 核心公式：
%   Jv(q)'F = W_req(q,qd,qdd)。
%
% 在优化链路中的作用：
%   buildInitialGuessQuinticHS 用本函数为 Unode/Umid 生成物理一致的初值。

Jout = sgpJacobian(q, model);
Wreq = computeCompositeRequiredWrench(q, qd, qdd, model);
matrixToSolve = Jout.Jv.';
if rcond(matrixToSolve) < model.num.rcondMin
    warning('inverseDynamicsCompositeRigidBody:IllConditionedJacobian', ...
        'Jv 转置病态，使用 pinv 生成驱动力初值。');
    F = pinv(matrixToSolve) * Wreq;
else
    F = matrixToSolve \ Wreq;
end
F = min(max(F, model.actuator.forceMin), model.actuator.forceMax);
end
