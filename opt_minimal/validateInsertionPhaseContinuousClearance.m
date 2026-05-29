function report = validateInsertionPhaseContinuousClearance(scene)
% validateInsertionPhaseContinuousClearance - 验证送入阶段固定分离平面的连续安全性
%
% 文件用途：
%   基于 q2(s) 参数化，解析检查 s 属于 [-0.65,0] 时圆柱体姿态、y 坐标和
%   x-z 送入直线关系严格保持，固定分离法向间隙从阶段 1 安全距离过渡到贴近距离。
%
% 输入参数：
%   scene struct - 圆柱体-长方体两阶段场景。
%
% 输出参数：
%   report struct - 固定法向、解析间隙、高密采样最小间隙和通过标识。
%
% 核心公式：
%   n2=R_B*[0;0;1]，q2(s) 沿 scene.phase 中的 [x,z] 直线从途径点送至目标点。
%
% 在优化链路中的作用：
%   run_01_hs_dynamic_opt 和 dense 验证报告阶段 2 连续解析安全结论。

n2 = scene.box.R_S * [0; 0; 1];
sValues = linspace(scene.phase.pCylinderWaypoint_B(1), scene.phase.pCylinderGoal_B(1), 101);
gaps = zeros(size(sValues));
lateral = zeros(size(sValues));
height = zeros(size(sValues));
axisErrors = zeros(size(sValues));
for i = 1:numel(sValues)
    q = insertionPoseFromScalar(sValues(i), 0, 0, scene);
    distanceInfo = evaluateCylinderBoxDistanceNumeric(q, scene);
    gaps(i) = supportGapFixedNormal(q, n2, scene);
    lateral(i) = abs(distanceInfo.pCylinder_B(2));
    height(i) = abs(distanceInfo.pCylinder_B(3) - insertionLineHeightNumeric(distanceInfo.pCylinder_B(1), scene));
    axisErrors(i) = norm(distanceInfo.axis_B - [1; 0; 0]);
end

report = struct();
report.fixedNormal = n2;
report.analyticGap = scene.collision.finalGap;
report.sampleMinGap = min(gaps);
report.sampleMaxGapError = max(abs(gaps - linspace(scene.collision.waypointGap, scene.collision.finalGap, numel(gaps))));
report.maxLateralError = max(lateral);
report.maxHeightError = max(height);
report.maxAxisError = max(axisErrors);
report.passed = report.sampleMinGap >= scene.collision.finalGap - 1e-10 && report.maxLateralError <= 1e-12 && ...
    report.maxHeightError <= 1e-12 && report.maxAxisError <= 1e-12;
end

function q = insertionPoseFromScalar(s, sd, sdd, scene) %#ok<INUSD>
q = zeros(6, 1);
cB = [s; 0; insertionLineHeightNumeric(s, scene)];
q(1:3) = scene.box.center_S + scene.box.R_S * (cB - scene.objectCylinder.center_P);
q(4:6) = scene.box.rpy;
end

function z = insertionLineHeightNumeric(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end

function gap = supportGapFixedNormal(q, n, scene)
R = rpy2rotmZYX(q(4:6));
cylinderCenter = q(1:3) + R * scene.objectCylinder.center_P;
cylinderAxis = R * scene.objectCylinder.axis_P;
boxMin = n.' * scene.box.center_S - sum(scene.box.halfSize(:) .* abs(scene.box.R_S.' * n));
axial = 0.5 * scene.objectCylinder.length * abs(n.' * cylinderAxis);
radial = scene.objectCylinder.radius * sqrt(max(0, 1 - (n.' * cylinderAxis)^2));
cylinderMax = n.' * cylinderCenter + axial + radial;
gap = boxMin - cylinderMax;
end
