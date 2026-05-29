function [xdot, aux] = stateDynamicsCompositeRigidBody(x, F, model)
% stateDynamicsCompositeRigidBody - 合成刚体 Stewart 状态动力学
%
% 文件用途：
%   给定状态 x=[q;qd] 和支链驱动力 F，计算状态导数 xdot=[qd;qdd]。
%
% 输入参数：
%   x [12x1] - 状态，[平台位姿; 平台广义速度]。
%   F [6x1]  - 六条支链轴向驱动力，优化控制变量。
%   model struct - Stewart 几何和合成刚体动力学参数。
%
% 输出参数：
%   xdot [12x1] - 状态导数。
%   aux struct  - qdd、Wact、Wbias、H 和病态诊断信息。
%
% 核心公式：
%   Jv(q)'F = W_req(q,qd,qdd)，对 qdd 线性化得 H*qdd+Wbias，
%   qdd = H\(Jv'F-Wbias)。
%
% 在优化链路中的作用：
%   evaluatePathConstraintsAtPoint 在未显式给定 qdd 时调用本函数，用于数值诊断和
%   与隐式动力学残差口径保持一致。

validateattributes(x, {'double'}, {'real', 'finite', 'vector', 'numel', 12}, mfilename, 'x');
validateattributes(F, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'F');
x = x(:);
F = F(:);

q = x(1:6);
qd = x(7:12);
Jout = sgpJacobian(q, model);
Wact = Jout.Jv.' * F;
Wbias = computeCompositeRequiredWrench(q, qd, zeros(6, 1), model);

H = zeros(6, 6);
for dofIndex = 1:6
    unitAcceleration = zeros(6, 1);
    unitAcceleration(dofIndex) = 1;
    H(:, dofIndex) = computeCompositeRequiredWrench(q, qd, unitAcceleration, model) - Wbias;
end

matrixReciprocalCondition = rcond(H);
if matrixReciprocalCondition < model.num.rcondMin
    error('stateDynamicsCompositeRigidBody:IllConditionedMassMatrix', ...
        '合成刚体动力学矩阵 H 病态，rcond=%.3e。', matrixReciprocalCondition);
end

qdd = H \ (Wact - Wbias);
xdot = [qd; qdd];

aux = struct();
aux.qdd = qdd;
aux.Wact = Wact;
aux.Wbias = Wbias;
aux.H = H;
aux.rcondH = matrixReciprocalCondition;
aux.Jout = Jout;
end
