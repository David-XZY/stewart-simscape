function collisionModel = buildCylinderBoxCollisionModel(scene)
% buildCylinderBoxCollisionModel - 构建有限圆柱体和固定有向长方体凸体碰撞模型
%
% 文件用途：
%   保存阶段 1 分离证书约束和后验距离评价所需的圆柱体、长方体和安全参数。
%
% 输入参数：
%   scene struct - 圆柱体、长方体和碰撞配置。
%
% 输出参数：
%   collisionModel struct - 有限圆柱体、OBB 轴、半尺寸、安全间隙和保守平滑量。
%
% 核心公式：
%   对单位法向 n，g=n'*(p_B-c)-h_B'|R_B'n|-L/2*|n'a|-r*||(I-aa')n||。
%
% 在优化链路中的作用：
%   buildCylinderBoxTransferScene 生成 scene.collisionGeometry，供 NLP、验证和绘图共享。

collisionModel = struct();
collisionModel.box.center_S = scene.box.center_S;
collisionModel.box.R_S = scene.box.R_S;
collisionModel.box.axes_S = scene.box.R_S;
collisionModel.box.size = scene.box.size;
collisionModel.box.halfSize = scene.box.halfSize;
collisionModel.objectCylinder = scene.objectCylinder;
collisionModel.safeDistance = scene.collision.safeDistance;
collisionModel.finalGap = scene.collision.finalGap;
collisionModel.smoothingEps = scene.collision.smoothingEps;
collisionModel.separatorStrictUnit = scene.collision.separatorStrictUnit;
end
