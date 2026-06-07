function [qWaypoint, qGoal, info] = computeCylinderWaypointAndGoal(scene)
% computeCylinderWaypointAndGoal - 由圆柱体和长方体几何计算途径点与目标点
%
% 文件用途：
%   将长方体坐标系中的圆柱中心目标转换为动平台位姿，避免在主脚本中散落硬编码。
%
% 输入参数：
%   scene struct - 包含 box 与 objectCylinder 几何参数。
%
% 输出参数：
%   qWaypoint [6x1] - 途径点平台位姿。
%   qGoal     [6x1] - 目标点平台位姿。
%   info struct      - 保存中间几何量，供测试与摘要使用。
%
% 核心公式：
%   p_P^S=p_B^S+R_B^S*(p_C^B-p_C^P)，姿态固定为长方体 RPY。
%
% 在优化链路中的作用：
%   buildCylinderBoxTransferScene 调用本函数生成两阶段边界。

pBox = scene.box.center_S;
RBox = scene.box.R_S;
pOffset = scene.objectCylinder.center_P;

pCylinderWaypoint_B = scene.phase.pCylinderWaypoint_B;
pCylinderGoal_B = scene.phase.pCylinderGoal_B;

qWaypoint = [pBox + RBox * (pCylinderWaypoint_B - pOffset); scene.box.rpy];
qGoal = [pBox + RBox * (pCylinderGoal_B - pOffset); scene.box.rpy];

info = struct();
info.pCylinderWaypoint_B = pCylinderWaypoint_B;
info.pCylinderGoal_B = pCylinderGoal_B;
info.pBox = pBox;
info.RBox = RBox;
info.pOffset = pOffset;
if isfield(scene, 'hood') && isfield(scene.hood, 'roof')
    roofLowerFaceZ_B = scene.hood.roof.center_B(3) - scene.hood.roof.halfSize(3);
    info.roofLowerFaceZ_B = roofLowerFaceZ_B;
    info.stage2RoofStartGap = roofLowerFaceZ_B - ...
        (pCylinderWaypoint_B(3) + scene.objectCylinder.radius);
    info.roofGoalGap = roofLowerFaceZ_B - ...
        (pCylinderGoal_B(3) + scene.objectCylinder.radius);
    info.sideGapAtGoal = scene.hood.innerHalfWidth - scene.objectCylinder.radius;
    info.hoodObstacleNames = {scene.hood.obstacles.name};
end
end
