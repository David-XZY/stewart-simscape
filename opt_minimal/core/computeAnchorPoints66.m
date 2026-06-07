function P = computeAnchorPoints66(radius, halfAngle, phase)
% computeAnchorPoints66 - 生成 6-6 Stewart 平台铰点坐标
%
% 文件用途：根据铰点分布圆半径、每组铰点半夹角和整体相位，生成六个
% 铰点在对应平台局部坐标系中的位置。
%
% 输入参数：
%   radius    [1x1] 铰点分布圆半径，单位 m。
%   halfAngle [1x1] 每组两个铰点相对组中心线的半夹角，单位 rad。
%   phase     [1x1] 第一组中心线相位角，单位 rad。
%
% 输出参数：
%   P [3x6] 六个铰点坐标，每一列为一个铰点，z 坐标为 0。
%
% 主要公式：theta = phase + [0,120,240] deg +/- halfAngle，
% P_i = radius*[cos(theta_i); sin(theta_i); 0]。
%
% 在轨迹优化中的作用：这是自定义 6-UCU Stewart 模型的回退几何生成
% 函数，用于 buildOptModelCustom。

validateattributes(radius, {'double'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, 'radius');
validateattributes(halfAngle, {'double'}, {'real', 'finite', 'scalar'}, mfilename, 'halfAngle');
validateattributes(phase, {'double'}, {'real', 'finite', 'scalar'}, mfilename, 'phase');

groupAngles = phase + deg2rad([0, 120, 240]);
theta = zeros(1, 6);
theta(1:2:end) = groupAngles - halfAngle;
theta(2:2:end) = groupAngles + halfAngle;

P = [radius * cos(theta);
     radius * sin(theta);
     zeros(1, 6)];
end
