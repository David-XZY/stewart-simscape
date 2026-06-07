function geometry = buildCylinderBoxGeometry(scene)
% buildCylinderBoxGeometry - 保存圆柱体和长方体碰撞几何
%
% 文件用途：
%   用统一结构保存真实圆柱体与固定长方体参数，不再生成球包络。
%
% 输入参数：
%   scene struct - 圆柱体、长方体和碰撞配置。
%
% 输出参数：
%   geometry struct - 数值验证、绘图和动画共享的几何结构。
%
% 核心公式：
%   圆柱体中心和轴线由平台位姿变换；长方体由中心、姿态和半尺寸定义。
%
% 在优化链路中的作用：
%   buildCylinderBoxTransferScene 生成 scene.collisionGeometry。

geometry = struct();
geometry.box = scene.box;
geometry.objectCylinder = scene.objectCylinder;
geometry.safeDistance = scene.collision.safeDistance;
geometry.finalGap = scene.collision.finalGap;
geometry.smoothingEps = scene.collision.smoothingEps;
end
