function qPre = computePreAlignmentTarget(scene)
% computePreAlignmentTarget - 根据槽口与先导挂耳几何计算预对准位姿
%
% 文件用途：
%   避免在主脚本中手写目标位姿魔法常数，使用挂架坐标系下的预对准
%   先导挂耳组中心位置和平台坐标系下的先导挂耳组中心位置计算 qPre。
%
% 输入参数：
%   scene struct - 至少包含 rack.p_S、rack.R_S、rack.rpy、
%   slot.pPreLeadCenter_R 和 lug.pLeadCenter_P。
%
% 输出参数：
%   qPre [6x1] - 预对准平台位姿 [p; rpy]。
%
% 核心公式：
%   pPre = p_S + R_S*(pPreLeadCenter_R - pLeadCenter_P)，
%   rpyPre = rack.rpy。
%
% 在优化链路中的作用：
%   主脚本用 qPre 作为 HS 末端状态边界条件。

qPre = zeros(6, 1);
qPre(1:3) = scene.rack.p_S + scene.rack.R_S * ...
    (scene.slot.pPreLeadCenter_R - scene.lug.pLeadCenter_P);
qPre(4:6) = scene.rack.rpy;
end
