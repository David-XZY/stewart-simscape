function scene = buildPreAlignmentScene(model)
% buildPreAlignmentScene - 构建弹体预对准场景参数
%
% 文件用途：
%   集中配置初始状态、挂架位姿、槽口几何、弹体/挂耳尺寸以及碰撞参数，
%   并由几何关系自动计算预对准目标位姿 qPre。
%
% 输入参数：
%   model struct - Stewart 模型，仅用于读取 qHome 等默认参数。
%
% 输出参数：
%   scene struct - 包含 q0、qPre、挂架、弹体、挂耳和碰撞参数。
%
% 核心公式：
%   qPre(1:3)=p_S+R_S*(pPreLeadCenter_R-pLeadCenter_P)，
%   qPre(4:6)=rack.rpy。
%
% 在优化链路中的作用：
%   run_01_hs_dynamic_opt 通过本函数获得唯一的场景和目标位姿来源。

scene = struct();
scene.q0 = [0; 0; 1.00; 0; 0; 0];
if isfield(model, 'qHome')
    scene.q0 = model.qHome;
end
scene.qd0 = zeros(6, 1);

scene.rack.p_S = [0.15; -0.20; 1.80];
scene.rack.rpy = deg2rad([5; 15; 10]);
scene.rack.R_S = rpy2rotmZYX(scene.rack.rpy);
scene.insertionDirection_R = [1; 0; 0];
scene.preAlign.duration = 5.0;

scene.slot.xEntry_R = -0.40;
scene.slot.yCenters_R = [+0.08, -0.08];
scene.slot.width = 0.05;
scene.slot.zReference_R = -0.10;
scene.slot.preDistance = 0.05;
scene.slot.pPreLeadCenter_R = [-0.45; 0; -0.10];

scene.lug.pLeadCenter_P = [0.20; 0; 0.32];
scene.lug.size = [0.04; 0.04; 0.04];
scene.lug.centers_P = [ ...
     0.20,  0.20, -0.20, -0.20;
     0.08, -0.08,  0.08, -0.08;
     0.32,  0.32,  0.32,  0.32 ];

scene.munition.length = 1.20;
scene.munition.radius = 0.15;
scene.munition.mass = 100;
scene.munition.pCenterInPlatform = [0; 0; 0.15];
scene.munition.RInPlatform = eye(3);

scene.platform.mass = 50;
scene.platform.radius = 0.50;
scene.rack.size = [0.80; 0.20; 0.20];

scene.collision.bodySphereCount = 25;
scene.collision.bodySphereSpacing = 0.05;
scene.collision.bodySphereRadius = sqrt(0.15^2 + (0.05/2)^2);
scene.collision.lugSphereRadius = 0.5 * norm(scene.lug.size);
scene.collision.safeDistance = 0.010;

scene.qPre = computePreAlignmentTarget(scene);
scene.qdPre = zeros(6, 1);
scene.collisionGeometry = buildCollisionGeometry(scene);
end
