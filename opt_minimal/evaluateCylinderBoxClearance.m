function clearance = evaluateCylinderBoxClearance(q, scene)
% evaluateCylinderBoxClearance - 计算运动圆柱体到固定长方体下表面的分离间隙
%
% 文件用途：
%   对给定位姿 q 计算圆柱体中心、轴线和下表面分离间隙，用于路径约束、
%   dense 后验验证、绘图和测试。
%
% 输入参数：
%   q [6x1]      - 平台位姿 [p;roll;pitch;yaw]。
%   scene struct - 圆柱体、长方体和碰撞参数。
%
% 输出参数：
%   clearance struct - 包含 distance、pCylinder_B、axis_B、rhoZ、mu 等诊断量。
%
% 核心公式：
%   mu=e_z^T a_C^B，rho_z=L/2*abs(mu)+r*sqrt(max(0,1-mu^2))，
%   d_box=-h_z-(z_C^B+rho_z)。
%
% 在优化链路中的作用：
%   evaluatePathConstraintsAtPoint 与 validateTrajectoryDenseImplicit 统一调用本函数。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
q = q(:);

R = rpy2rotmZYX(q(4:6));
pCylinder_S = q(1:3) + R * scene.objectCylinder.center_P;
axis_S = R * scene.objectCylinder.axis_P;
pCylinder_B = scene.box.R_S.' * (pCylinder_S - scene.box.center_S);
axis_B = scene.box.R_S.' * axis_S;
axis_B = axis_B / norm(axis_B);

mu = axis_B(3);
rhoZ = 0.5 * scene.objectCylinder.length * abs(mu) + ...
    scene.objectCylinder.radius * sqrt(max(0, 1 - mu^2));
distance = -scene.box.halfSize(3) - (pCylinder_B(3) + rhoZ);

clearance = struct();
clearance.distance = distance;
clearance.distances = distance;
clearance.minDistance = distance;
clearance.minIndex = 1;
clearance.pCylinder_S = pCylinder_S;
clearance.pCylinder_B = pCylinder_B;
clearance.axis_S = axis_S;
clearance.axis_B = axis_B;
clearance.mu = mu;
clearance.rhoZ = rhoZ;
clearance.finalGapError = distance - scene.collision.finalGap;
clearance.safeDistanceResidual = scene.collision.safeDistance - distance;
end
