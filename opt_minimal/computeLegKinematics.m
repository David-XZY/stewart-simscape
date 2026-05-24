function legKin = computeLegKinematics(q, qd, qdd, model)
% computeLegKinematics - 计算 Stewart 支链完整运动学量
%
% 文件用途：根据平台位姿、速度和加速度计算六条支链的长度、伸缩速度、
% 伸缩加速度、单位方向导数、支链等效角速度和上铰点运动。
%
% 输入参数：
%   q   [6x1] 平台广义坐标 [x;y;z;roll;pitch;yaw]。
%   qd  [6x1] 平台广义速度。
%   qdd [6x1] 平台广义加速度。
%   model struct - 包含 A、B 和 Lc 等模型参数。
%
% 输出参数：
%   legKin struct - 包含 L、Ld、Ldd、u、udot、uddot、omegaLeg、
%   alphaLeg、Ptop、Vtop 和 Atop。
%
% 主要公式：s_i = p + R*B_i - A_i，Ldot_i = u_i.'*Vtop_i，
% Lddot_i = u_i.'*Atop_i + (Vtop_i.'*Vtop_i - Ldot_i^2)/L_i。
%
% 在轨迹优化中的作用：支链速度、支链加速度约束和支链自身惯性计算都
% 依赖本函数。第一版忽略支链绕自身轴线的自转，取 omegaLeg=cross(u,udot)。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
validateattributes(qd, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'qd');
validateattributes(qdd, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'qdd');
q = q(:);
qd = qd(:);
qdd = qdd(:);

kin = sgpIK(q, model);
Jout = sgpJacobian(q, model);

p = q(1:3);
pd = qd(1:3);
pdd = qdd(1:3);
rpy = q(4:6);
rpyDot = qd(4:6);
rpyDDot = qdd(4:6);

[E, Edot] = rpyRateMapZYX(rpy, rpyDot);
omega = E * rpyDot;
alpha = Edot * rpyDot + E * rpyDDot;

Ptop = zeros(3, 6);
Vtop = zeros(3, 6);
Atop = zeros(3, 6);
Ld = Jout.Jq * qd;
Ldd = zeros(6, 1);
udot = zeros(3, 6);
uddot = zeros(3, 6);
omegaLeg = zeros(3, 6);
alphaLeg = zeros(3, 6);

for legIndex = 1:6
    rTop = kin.rB(:, legIndex);
    Ptop(:, legIndex) = p + rTop;

    % 上铰点速度/加速度来自刚体运动公式。
    Vtop(:, legIndex) = pd + cross(omega, rTop);
    Atop(:, legIndex) = pdd + cross(alpha, rTop) + cross(omega, cross(omega, rTop));

    unitDirection = kin.u(:, legIndex);
    legLength = kin.L(legIndex);
    legSpeed = Ld(legIndex);

    Ldd(legIndex) = unitDirection.' * Atop(:, legIndex) + ...
        (Vtop(:, legIndex).' * Vtop(:, legIndex) - legSpeed^2) / legLength;

    udot(:, legIndex) = (Vtop(:, legIndex) - legSpeed * unitDirection) / legLength;
    uddot(:, legIndex) = (Atop(:, legIndex) - Ldd(legIndex) * unitDirection - ...
        2 * legSpeed * udot(:, legIndex)) / legLength;

    omegaLeg(:, legIndex) = cross(unitDirection, udot(:, legIndex));
    alphaLeg(:, legIndex) = cross(unitDirection, uddot(:, legIndex));
end

legKin = struct();
legKin.L = kin.L;
legKin.Ld = Ld;
legKin.Ldd = Ldd;
legKin.u = kin.u;
legKin.udot = udot;
legKin.uddot = uddot;
legKin.omegaLeg = omegaLeg;
legKin.alphaLeg = alphaLeg;
legKin.Ptop = Ptop;
legKin.Vtop = Vtop;
legKin.Atop = Atop;
legKin.R = kin.R;
legKin.rB = kin.rB;
legKin.omega = omega;
legKin.alpha = alpha;
end
