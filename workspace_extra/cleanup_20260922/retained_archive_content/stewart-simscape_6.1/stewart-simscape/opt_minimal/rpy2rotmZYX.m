function R = rpy2rotmZYX(rpy)
% rpy2rotmZYX - 将 ZYX 欧拉角转换为旋转矩阵
%
% 文件用途：
%   根据广义坐标中的 roll、pitch、yaw 计算动平台坐标系 {B}
%   相对定平台坐标系 {A} 的旋转矩阵。
%
% 输入参数：
%   rpy [3x1] - [roll; pitch; yaw]，单位 rad。
%
% 输出参数：
%   R [3x3] - 从 {B} 到 {A} 的旋转矩阵。
%
% 主要公式：
%   R = Rz(yaw) * Ry(pitch) * Rx(roll)。
%   这里采用 ZYX 顺序，即先绕动坐标 x 轴 roll，再绕中间 y 轴
%   pitch，最后绕定坐标 z 轴 yaw。该写法与工程中运动学脚本常用的
%   Rz*Ry*Rx 形式一致。
%
% 与 Stewart 平台轨迹优化的关系：
%   逆运动学和雅可比都需要用 R 将动平台上铰点 B_i 从动平台坐标系
%   转到定平台坐标系。

validateattributes(rpy, {'double'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, 'rpy');
rpy = rpy(:);

roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);

Rx = [1, 0, 0;
      0, cos(roll), -sin(roll);
      0, sin(roll),  cos(roll)];

Ry = [ cos(pitch), 0, sin(pitch);
       0,          1, 0;
      -sin(pitch), 0, cos(pitch)];

Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw),  cos(yaw), 0;
      0,         0,        1];

R = Rz * Ry * Rx;
end
