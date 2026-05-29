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
%   run_01_hs_dynamic_opt 调用本函数，后续初值、NLP、验证、绘图和导出均读取 scene。

scene = struct();
scene.name = '运动圆柱体-固定长方体两阶段轨迹优化';

scene.q0 = [0; 0; 1.00; 0; 0; 0];
if isfield(model, 'qHome')
    scene.q0 = model.qHome;
end
scene.qd0 = zeros(6, 1);
scene.qdGoal = zeros(6, 1);

scene.box.center_S = [0.15; -0.20; 1.80];
scene.box.rpy = deg2rad([5; 15; 10]);
scene.box.R_S = rpy2rotmZYX(scene.box.rpy);
scene.box.size = [0.80; 0.20; 0.20];
scene.box.halfSize = scene.box.size / 2;

scene.objectCylinder.length = 1.20;
scene.objectCylinder.radius = 0.15;
scene.objectCylinder.mass = 100;
scene.objectCylinder.center_P = [0; 0; 0.15];
scene.objectCylinder.axis_P = [1; 0; 0];

scene.collision.safeDistance = 0.010;
scene.collision.finalGap = 0.005;
scene.collision.stage1ConstraintDistance = scene.collision.safeDistance;
scene.collision.waypointGap = scene.collision.safeDistance;
scene.collision.smoothingEps = 1e-6;
scene.collision.separatorStrictUnit = true;

scene.phase.durationApproach = 5.0;
scene.phase.durationInsertion = 2.5;
scene.phase.numIntervalsApproach = 40;
scene.phase.numIntervalsInsertion = 20;
waypointHeight_B = -(scene.box.halfSize(3) + scene.objectCylinder.radius + scene.collision.waypointGap);
goalHeight_B = -(scene.box.halfSize(3) + scene.objectCylinder.radius + scene.collision.finalGap);
scene.phase.pCylinderWaypoint_B = [-0.65; 0; waypointHeight_B];
scene.phase.pCylinderGoal_B = [0.00; 0; goalHeight_B];
scene.phase.insertionDirection_B = [1; 0; 0];

[scene.qWaypoint, scene.qGoal, targetInfo] = computeCylinderWaypointAndGoal(scene);
scene.target = targetInfo;
scene.collisionGeometry = buildCylinderBoxCollisionModel(scene);

assertPoseClose(scene.qWaypoint, [-0.57862443; -0.29219104; 1.57370980; 0.08726646; 0.26179939; 0.17453293], ...
    5e-8, 'qWaypoint');
assertPoseClose(scene.qGoal, [0.04103414; -0.18337102; 1.41028867; 0.08726646; 0.26179939; 0.17453293], ...
    5e-8, 'qGoal');

goalClearance = evaluateCylinderBoxClearance(scene.qGoal, scene);
if abs(goalClearance.distance - scene.collision.finalGap) > 1e-10
    error('buildCylinderBoxTransferScene:GoalGapMismatch', ...
        '目标点终端间隙 %.12g 与要求 %.12g 不一致。', ...
        goalClearance.distance, scene.collision.finalGap);
end
end

function assertPoseClose(actualValue, expectedValue, toleranceValue, name)
err = max(abs(actualValue(:) - expectedValue(:)));
if err > toleranceValue
    error('buildCylinderBoxTransferScene:ReferencePoseMismatch', ...
        '%s 自动计算值与参考值不一致，最大误差 %.3e。', name, err);
end
end
