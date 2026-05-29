function collision = buildCollisionGeometry(scene)
% buildCollisionGeometry - 构建弹体、挂耳和挂架碰撞几何
%
% 文件用途：
%   生成弹体球链、四个挂耳包络球和固定挂架 OBB 的几何参数。
%
% 输入参数：
%   scene struct - 包含弹体、挂耳、挂架和碰撞冻结参数。
%
% 输出参数：
%   collision struct - 碰撞检查所需的局部球心、半径、OBB 位姿和安全距离。
%
% 核心公式：
%   弹体沿平台 x 轴用 25 个球近似，球心间距 0.05 m。
%
% 在优化链路中的作用：
%   evaluateCollisionClearance 读取本结构计算 29 条碰撞间隙。

collision = scene.collision;
bodyOffsets = ((0:collision.bodySphereCount-1) - (collision.bodySphereCount-1)/2) * ...
    collision.bodySphereSpacing;
collision.bodySphereCenters_P = scene.munition.pCenterInPlatform + ...
    scene.munition.RInPlatform * [bodyOffsets; zeros(1, collision.bodySphereCount); zeros(1, collision.bodySphereCount)];
collision.lugSphereCenters_P = scene.lug.centers_P;
collision.rack.center = scene.rack.p_S;
collision.rack.R = scene.rack.R_S;
collision.rack.halfSize = scene.rack.size / 2;
collision.rack.size = scene.rack.size;
end
