function refs = exportTrajectoryToSimscape(traj, scene, disc)
% exportTrajectoryToSimscape - 导出圆柱体两阶段轨迹供 Simscape 离线验证
%
% 文件用途：
%   将优化得到的节点状态和驱动力控制整理为 refs 结构体。Simscape 不参与
%   优化器，只用于后续松耦合验证。
%
% 输入参数：
%   traj struct - 主脚本生成的轨迹结构。
%   scene struct - 圆柱体-长方体两阶段场景。
%   disc struct  - 两阶段离散参数。
%
% 输出参数：
%   refs struct - 包含时间、位姿、速度、驱动力和目标位姿。
%
% 核心公式：
%   第一版只导出节点量，不做 Simulink timeseries 封装。
%
% 在优化链路中的作用：
%   保存结果 MAT 文件时同步保存 refs，方便后续离线接入 Simscape。

refs = struct();
if nargin < 3 || isempty(disc)
    disc = struct('durationApproach', NaN, 'durationInsertion', NaN, ...
        'waypointNodeIndex', NaN);
end
refs.t = traj.t;
refs.q = traj.Q;
refs.qd = traj.V;
refs.Fleg = traj.Unode;
refs.qWaypoint = scene.qWaypoint;
refs.qGoal = scene.qGoal;
refs.q0 = scene.q0;
refs.T1 = disc.durationApproach;
refs.T2 = disc.durationInsertion;
refs.tStageBoundary = disc.durationApproach;
refs.stageBoundaryNode = disc.waypointNodeIndex;
refs.description = '运动圆柱体到固定长方体的两阶段 HS 优化节点参考轨迹';
end
