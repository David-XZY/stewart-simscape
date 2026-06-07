function [E, Edot] = rpyRateMapZYX(rpy, rpyDot)
% rpyRateMapZYX - ZYX 欧拉角速度到空间角速度的映射矩阵
%
% 文件用途：计算 omega = E(rpy)*rpy_dot，其中 omega 表示动平台角速度，
% 并在定平台坐标系 {A} 中表达；可选返回 Edot 供角加速度计算。
%
% 输入参数：
%   rpy    [3x1] [roll; pitch; yaw]，单位 rad。
%   rpyDot [3x1] 可选，RPY 角速度，单位 rad/s。
%
% 输出参数：
%   E     [3x3] 从 [roll_dot; pitch_dot; yaw_dot] 到 omega 的映射。
%   Edot  [3x3] E 对时间的一阶导数，使用中心差分计算。
%
% 主要公式：对 R = Rz(yaw)*Ry(pitch)*Rx(roll)，空间角速度为
% omega = yaw_dot*e_z + pitch_dot*Rz*e_y + roll_dot*Rz*Ry*e_x。
%
% 在轨迹优化中的作用：Jq = Jv*blkdiag(I,E)，角加速度为
% alpha = Edot*rpy_dot + E*rpy_ddot，用于完整动力学。

validateattributes(rpy, {'double'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, 'rpy');
rpy = rpy(:);
if nargin < 2
    rpyDot = zeros(3, 1);
else
    validateattributes(rpyDot, {'double'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, 'rpyDot');
    rpyDot = rpyDot(:);
end

pitch = rpy(2);
yaw = rpy(3);

E = [cos(yaw)*cos(pitch), -sin(yaw), 0;
     sin(yaw)*cos(pitch),  cos(yaw), 0;
     -sin(pitch),          0,        1];

if nargout > 1
    h = 1e-6;
    Edot = zeros(3, 3);
    for angleIndex = 1:3
        perturb = zeros(3, 1);
        perturb(angleIndex) = h;
        Eplus = rpyRateMapZYX(rpy + perturb);
        Eminus = rpyRateMapZYX(rpy - perturb);
        Edot = Edot + ((Eplus - Eminus) / (2 * h)) * rpyDot(angleIndex);
    end
end
end
