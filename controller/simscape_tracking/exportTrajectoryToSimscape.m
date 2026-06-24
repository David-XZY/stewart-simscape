function [refs, references] = exportTrajectoryToSimscape(traj, scene, disc)
% exportTrajectoryToSimscape - 导出 IHSID 轨迹供 Simscape 控制接口使用
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
%   refs struct       - 包含原始节点时间、位姿、速度、驱动力和支链长度。
%   references struct - 与现有 Simscape 模型 From Workspace 输入兼容的 timeseries。
%
% 核心公式：
%   references.r、references.rL 和 references.uFF 的数据维度均为 N×6。
%   r/rL 分别表示相对初始位姿和相对初始支链长度；uFF 是 IHSID 驱动力前馈。
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
refs.L = traj.L;
refs.qWaypoint = scene.qWaypoint;
refs.qGoal = scene.qGoal;
refs.q0 = scene.q0;
refs.T1 = disc.durationApproach;
refs.T2 = disc.durationInsertion;
refs.tStageBoundary = disc.durationApproach;
refs.stageBoundaryNode = disc.waypointNodeIndex;
refs.description = 'standard IHSID 40x20 limited-memory 两阶段节点参考轨迹';

time = refs.t(:);
references = struct();
references.r = timeseries((refs.q - refs.q0).', time);
references.rL = timeseries((refs.L - refs.L(:, 1)).', time);
references.uFF = timeseries(refs.Fleg.', time);
references.rJq = timeseries(zeros(6, 6, numel(time)), time);
references.uCT = timeseries(zeros(numel(time), 6), time);
references.description = '供 stewart_platform_model.slx 使用的 IHSID 相对参考轨迹';
end
