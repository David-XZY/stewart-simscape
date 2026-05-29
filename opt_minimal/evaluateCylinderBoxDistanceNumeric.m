function distanceInfo = evaluateCylinderBoxDistanceNumeric(q, scene)
% evaluateCylinderBoxDistanceNumeric - 独立后验评价有限圆柱体到 OBB 的分离距离
%
% 文件用途：
%   不使用 NLP 中的分离变量，直接在候选分离法向上最大化凸体支持函数间隙，
%   报告有限圆柱体与固定有向长方体的数值分离距离。
%
% 输入参数：
%   q [6x1]      - 平台位姿 [p;roll;pitch;yaw]。
%   scene struct - 圆柱体-长方体场景。
%
% 输出参数：
%   distanceInfo struct - 包含 distance、bestNormal、candidateGaps 和几何诊断。
%
% 核心公式：
%   distance=max_{||n||=1} sigma_B_min(n)-sigma_C_max(q,n)。本函数从长方体轴、
%   圆柱轴、中心连线和局部优化候选中取最大值；若位姿满足本任务“从下方接近”
%   假设，固定长方体 z 轴候选给出解析真实距离。
%
% 在优化链路中的作用：
%   validateTrajectoryDenseImplicit 使用本函数进行独立密集后验检查。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
q = q(:);

R = rpy2rotmZYX(q(4:6));
cylinderCenter = q(1:3) + R * scene.objectCylinder.center_P;
cylinderAxis = R * scene.objectCylinder.axis_P;
cylinderAxis = cylinderAxis / norm(cylinderAxis);

candidateNormals = buildCandidateNormals(cylinderCenter, cylinderAxis, scene);
candidateGaps = zeros(1, size(candidateNormals, 2));
for i = 1:size(candidateNormals, 2)
    candidateGaps(i) = supportGap(candidateNormals(:, i), cylinderCenter, cylinderAxis, scene);
end

[distance, bestIndex] = max(candidateGaps);
bestNormal = candidateNormals(:, bestIndex);
if distance < scene.collision.safeDistance + 0.02
    [distance, bestNormal] = refineNormalByFminsearch(bestNormal, cylinderCenter, cylinderAxis, scene);
end

distanceInfo = struct();
distanceInfo.distance = distance;
distanceInfo.minDistance = distance;
distanceInfo.distances = distance;
distanceInfo.bestNormal = bestNormal;
distanceInfo.candidateGaps = candidateGaps;
distanceInfo.pCylinder_S = cylinderCenter;
distanceInfo.axis_S = cylinderAxis;
distanceInfo.pCylinder_B = scene.box.R_S.' * (cylinderCenter - scene.box.center_S);
distanceInfo.axis_B = scene.box.R_S.' * cylinderAxis;
distanceInfo.method = 'support-gap candidate normals plus fminsearch refinement when near safety boundary';
end

function normals = buildCandidateNormals(cylinderCenter, cylinderAxis, scene)
axes = scene.box.R_S;
centerVector = scene.box.center_S - cylinderCenter;
raw = [axes, -axes, cylinderAxis, -cylinderAxis, centerVector, -centerVector];
normals = zeros(3, 0);
for i = 1:size(raw, 2)
    v = raw(:, i);
    nv = norm(v);
    if nv > 1e-12
        normals(:, end+1) = v / nv; %#ok<AGROW>
    end
end
end

function gap = supportGap(n, cylinderCenter, cylinderAxis, scene)
boxMin = n.' * scene.box.center_S - sum(scene.box.halfSize(:) .* abs(scene.box.R_S.' * n));
axial = 0.5 * scene.objectCylinder.length * abs(n.' * cylinderAxis);
radial = scene.objectCylinder.radius * sqrt(max(0, 1 - (n.' * cylinderAxis)^2));
cylinderMax = n.' * cylinderCenter + axial + radial;
gap = boxMin - cylinderMax;
end

function [bestGap, bestNormal] = refineNormalByFminsearch(initialNormal, cylinderCenter, cylinderAxis, scene)
initialAngles = normalToAngles(initialNormal);
options = optimset('Display', 'off', 'MaxIter', 80, 'MaxFunEvals', 180, ...
    'TolX', 1e-10, 'TolFun', 1e-10);
objective = @(angles) -supportGap(anglesToNormal(angles), cylinderCenter, cylinderAxis, scene);
[anglesOpt, valueOpt] = fminsearch(objective, initialAngles, options);
candidateNormal = anglesToNormal(anglesOpt);
candidateGap = -valueOpt;
initialGap = supportGap(initialNormal, cylinderCenter, cylinderAxis, scene);
if candidateGap >= initialGap
    bestGap = candidateGap;
    bestNormal = candidateNormal;
else
    bestGap = initialGap;
    bestNormal = initialNormal;
end
end

function angles = normalToAngles(n)
n = n(:) / norm(n);
az = atan2(n(2), n(1));
el = asin(max(-1, min(1, n(3))));
angles = [az; el];
end

function n = anglesToNormal(angles)
az = angles(1);
el = max(-pi/2, min(pi/2, angles(2)));
n = [cos(az)*cos(el); sin(az)*cos(el); sin(el)];
end
