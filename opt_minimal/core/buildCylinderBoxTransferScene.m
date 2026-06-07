function scene = buildCylinderBoxTransferScene(model)
% buildCylinderBoxTransferScene - 构建运动圆柱体到固定长方体的两阶段场景
%
% 文件用途：
%   集中定义固定长方体、运动圆柱体、两阶段时间网格、初始位姿、途径点和目标点。
%   本文件是默认优化入口唯一的场景来源。
%
% 输入参数：
%   model struct - Stewart 平台模型，用于读取 qHome 作为默认初始位姿。
%
% 输出参数：
%   scene struct - 圆柱体、长方体、碰撞距离、两阶段边界和离散参数。
%
% 核心公式：
%   q=[p;roll;pitch;yaw]，R=Rz(yaw)*Ry(pitch)*Rx(roll)。
%   qWaypoint/qGoal 由 pBox+RBox*(pCylinder_B-pCylinder_P) 自动计算。
%
% 在优化链路中的作用：
%   run_01_ihsid_trajectory 调用本函数，后续初值、NLP、验证、绘图和导出均读取 scene。

scene = struct();
scene.name = '运动圆柱体-固定长方体两阶段轨迹优化';

scene.q0 = [0; 0; 1.00; 0; 0; 0];
if isfield(model, 'qHome')
    scene.q0 = model.qHome;
end
scene.qd0 = zeros(6, 1);
scene.qdGoal = zeros(6, 1);

scene.box.center_S = [0.15; -0.20; 1.80];
scene.box.rpy = deg2rad([5; 20; 45]);
scene.box.R_S = rpy2rotmZYX(scene.box.rpy);
scene.box.size = [0.80; 0.20; 0.20];
scene.box.halfSize = scene.box.size / 2;

scene.hood.enabled = true;
scene.hood.innerHalfWidth = 0.19;
scene.hood.skirtThickness = 0.03;
scene.hood.skirtDepth = 0.15;
scene.hood.roof = makeHoodObstacle('roof', [0; 0; -0.085], [0.80; 0.44; 0.03], scene.box);
scene.hood.leftSkirt = makeHoodObstacle('leftSkirt', [0; 0.205; -0.175], [0.80; 0.03; 0.15], scene.box);
scene.hood.rightSkirt = makeHoodObstacle('rightSkirt', [0; -0.205; -0.175], [0.80; 0.03; 0.15], scene.box);
scene.hood.obstacles = [scene.hood.roof, scene.hood.leftSkirt, scene.hood.rightSkirt];

scene.objectCylinder.length = 1.20;
scene.objectCylinder.radius = 0.15;
scene.objectCylinder.mass = 100;
scene.objectCylinder.center_P = [0; 0; 0.15];
scene.objectCylinder.axis_P = [1; 0; 0];

scene.collision.safeDistance = 0.015;
scene.collision.finalGap = 0.005;
scene.collision.stage1ConstraintDistance = scene.collision.safeDistance + 0.001;
scene.collision.entryBottomGap = 0.018;
scene.collision.smoothingEps = 1e-6;
scene.collision.separatorStrictUnit = true;

scene.phase.durationApproach = 5.0;
scene.phase.durationInsertion = 2.5;
scene.phase.numIntervalsApproach = 40;
scene.phase.numIntervalsInsertion = 20;
roofLowerFaceZ_B = scene.hood.roof.center_B(3) - scene.hood.roof.halfSize(3);
skirtBottomZ_B = roofLowerFaceZ_B - scene.hood.skirtDepth;
waypointHeight_B = skirtBottomZ_B - scene.objectCylinder.radius - scene.collision.entryBottomGap;
goalHeight_B = roofLowerFaceZ_B - scene.objectCylinder.radius - scene.collision.finalGap;
scene.phase.pCylinderWaypoint_B = [-0.10; 0; waypointHeight_B];
scene.phase.pCylinderGoal_B = [0.00; 0; goalHeight_B];
scene.phase.insertionDirection_B = scene.phase.pCylinderGoal_B - scene.phase.pCylinderWaypoint_B;
scene.collision.stage2RoofStartGap = roofLowerFaceZ_B - ...
    (scene.phase.pCylinderWaypoint_B(3) + scene.objectCylinder.radius);

[scene.qWaypoint, scene.qGoal, targetInfo] = computeCylinderWaypointAndGoal(scene);
scene.target = targetInfo;
scene.collisionGeometry = buildCylinderBoxCollisionModel(scene);

assertPoseClose(scene.qWaypoint, [-0.08829634; -0.36828646; 1.30248767; 0.08726646; 0.34906585; 0.78539816], ...
    5e-8, 'qWaypoint');
assertPoseClose(scene.qGoal, [0.02746608; -0.27261490; 1.42087269; 0.08726646; 0.34906585; 0.78539816], ...
    5e-8, 'qGoal');

goalClearance = evaluateCylinderBoxClearance(scene.qGoal, scene);
if abs(goalClearance.distance - scene.collision.finalGap) > 1e-10
    error('buildCylinderBoxTransferScene:GoalGapMismatch', ...
        '目标点终端间隙 %.12g 与要求 %.12g 不一致。', ...
        goalClearance.distance, scene.collision.finalGap);
end
sideGap = scene.hood.innerHalfWidth - scene.objectCylinder.radius;
if sideGap < scene.collision.safeDistance
    error('buildCylinderBoxTransferScene:SideGapTooSmall', ...
        '目标位姿侧向理论间隙 %.12g 小于安全距离 %.12g。', sideGap, scene.collision.safeDistance);
end
legWaypoint = sgpIK(scene.qWaypoint, model);
legGoal = sgpIK(scene.qGoal, model);
allLegs = [legWaypoint.L(:); legGoal.L(:)];
if any(allLegs < min(model.lmin(:))) || any(allLegs > max(model.lmax(:)))
    error('buildCylinderBoxTransferScene:LegLengthInfeasible', ...
        '途径点或目标点腿长超出模型上下限。');
end
end

function obstacle = makeHoodObstacle(name, center_B, sizeValue, box)
% makeHoodObstacle - 根据罩体局部 OBB 参数补全全局几何字段
obstacle = struct();
obstacle.name = name;
obstacle.center_B = center_B(:);
obstacle.size = sizeValue(:);
obstacle.halfSize = obstacle.size / 2;
obstacle.center_S = box.center_S + box.R_S * obstacle.center_B;
obstacle.R_S = box.R_S;
obstacle.axes_S = box.R_S;
end

function assertPoseClose(actualValue, expectedValue, toleranceValue, name)
err = max(abs(actualValue(:) - expectedValue(:)));
if err > toleranceValue
    error('buildCylinderBoxTransferScene:ReferencePoseMismatch', ...
        '%s 自动计算值与参考值不一致，最大误差 %.3e。', name, err);
end
end
