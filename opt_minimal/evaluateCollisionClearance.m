function clearance = evaluateCollisionClearance(q, scene)
% evaluateCollisionClearance - 计算弹体/挂耳到挂架 OBB 的 29 条间隙
%
% 文件用途：
%   将平台上的弹体球链和挂耳球变换到惯性系，逐个计算到固定挂架 OBB 的
%   有符号距离。
%
% 输入参数：
%   q [6x1] - 平台位姿。
%   scene struct - 包含 collisionGeometry。
%
% 输出参数：
%   clearance struct - 包含 distances、minDistance、minIndex 和球心坐标。
%
% 核心公式：
%   c_W = p + R(q)*c_P，d = signedDistanceSphereToOBB(c_W, r, OBB)。
%
% 在优化链路中的作用：
%   evaluatePathConstraintsAtPoint 将 safeDistance - distances 作为硬碰撞约束。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
q = q(:);
collision = scene.collisionGeometry;
R = rpy2rotmZYX(q(4:6));
p = q(1:3);

bodyCentersWorld = p + R * collision.bodySphereCenters_P;
lugCentersWorld = p + R * collision.lugSphereCenters_P;
allCenters = [bodyCentersWorld, lugCentersWorld];
allRadii = [collision.bodySphereRadius * ones(1, size(bodyCentersWorld, 2)), ...
    collision.lugSphereRadius * ones(1, size(lugCentersWorld, 2))];

distances = zeros(numel(allRadii), 1);
for sphereIndex = 1:numel(allRadii)
    distances(sphereIndex) = signedDistanceSphereToOBB(allCenters(:, sphereIndex), allRadii(sphereIndex), ...
        collision.rack.center, collision.rack.R, collision.rack.halfSize);
end

[minDistance, minIndex] = min(distances);
clearance = struct();
clearance.distances = distances;
clearance.minDistance = minDistance;
clearance.minIndex = minIndex;
clearance.centersWorld = allCenters;
clearance.radii = allRadii(:);
end
