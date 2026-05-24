function Wreq = computeCompositeRequiredWrench(q, qd, qdd, model)
% computeCompositeRequiredWrench - 计算平台弹体合成刚体所需空间力矩
%
% 文件用途：
%   将平台和弹体等效为固定在动平台上的合成刚体，计算其在动平台原点处
%   为实现 q、qd、qdd 所需的空间力矩 [force; moment]。
%
% 输入参数：
%   q/qd/qdd [6x1] - 位姿、广义速度、广义加速度。
%   model struct - 包含合成质量、质心、惯量和重力开关。
%
% 输出参数：
%   Wreq [6x1] - 动平台原点处需求空间力矩。
%
% 核心公式：
%   F=m*(a_com-g)，M=I*alpha+omega×(I*omega)+r_com×F。
%
% 在优化链路中的作用：
%   stateDynamicsCompositeRigidBody 用本函数构造状态动力学，初值逆动力学
%   也用本函数计算给定 qdd 需要的驱动力。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
validateattributes(qd, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'qd');
validateattributes(qdd, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'qdd');
q = q(:);
qd = qd(:);
qdd = qdd(:);

R = rpy2rotmZYX(q(4:6));
[E, Edot] = rpyRateMapZYX(q(4:6), qd(4:6));
omega = E * qd(4:6);
alpha = Edot * qd(4:6) + E * qdd(4:6);

mass = model.dynamics.totalMass;
comWorld = R * model.dynamics.comP;
comAcc = qdd(1:3) + cross(alpha, comWorld) + cross(omega, cross(omega, comWorld));
if model.dynamics.includeGravity
    gravity = model.g;
else
    gravity = zeros(3, 1);
end

force = mass * (comAcc - gravity);
inertiaWorld = R * model.dynamics.inertiaAtCOM_P * R.';
moment = inertiaWorld * alpha + cross(omega, inertiaWorld * omega) + cross(comWorld, force);
Wreq = [force; moment];
end
