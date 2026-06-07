function signedDistance = signedDistanceSphereToOBB(sphereCenter, sphereRadius, boxCenter, boxRotation, boxHalfSize)
% signedDistanceSphereToOBB - 计算球体到有向包围盒 OBB 的有符号距离
%
% 文件用途：
%   根据球心和半径，计算球体外表面到固定 OBB 外表面的有符号距离。
%
% 输入参数：
%   sphereCenter [3x1] - 球心惯性系坐标。
%   sphereRadius [1x1] - 球半径。
%   boxCenter [3x1] - OBB 中心惯性系坐标。
%   boxRotation [3x3] - OBB 坐标系到惯性系的旋转矩阵。
%   boxHalfSize [3x1] - OBB 三轴半尺寸。
%
% 输出参数：
%   signedDistance [1x1] - 球体表面到 OBB 的有符号距离，负值表示侵入。
%
% 核心公式：
%   xi=R'*(c-p)，delta=abs(xi)-h，
%   d_point_box=||max(delta,0)||+min(max(delta),0)，d=d_point_box-r。
%
% 在优化链路中的作用：
%   碰撞硬约束使用 safeDistance - signedDistance <= 0。

validateattributes(sphereCenter, {'double'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, 'sphereCenter');
validateattributes(sphereRadius, {'double'}, {'real', 'finite', 'scalar', 'nonnegative'}, mfilename, 'sphereRadius');
validateattributes(boxCenter, {'double'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, 'boxCenter');
validateattributes(boxRotation, {'double'}, {'real', 'finite', 'size', [3, 3]}, mfilename, 'boxRotation');
validateattributes(boxHalfSize, {'double'}, {'real', 'finite', 'vector', 'numel', 3, 'positive'}, mfilename, 'boxHalfSize');

localPoint = boxRotation.' * (sphereCenter(:) - boxCenter(:));
delta = abs(localPoint) - boxHalfSize(:);
outsideDistance = norm(max(delta, 0));
insideDistance = min(max(delta), 0);
signedDistance = outsideDistance + insideDistance - sphereRadius;
end
