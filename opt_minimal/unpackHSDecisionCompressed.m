function [Xnode, Fnode, Fmid, Xinternal] = unpackHSDecisionCompressed(z, scene, disc)
% unpackHSDecisionCompressed - 解包压缩 HS 决策变量并补入固定端点
%
% 文件用途：
%   将 474 维决策向量还原为内部节点状态、节点控制力和中点控制力，并把 scene 中
%   固定的起点/终点状态补入完整 Xnode。
%
% 输入参数：
%   z [474x1]  - 压缩 HS 决策变量。
%   scene      - 预对准场景，包含 q0、qd0、qPre、qdPre。
%   disc       - HS 离散参数，包含 numNodes 和 numMidpoints。
%
% 输出参数：
%   Xnode      [12x21] - 完整节点状态，首尾列由 scene 常数给出。
%   Fnode      [6x21]  - 节点控制力。
%   Fmid       [6x20]  - 中点控制力。
%   Xinternal  [12x19] - 内部节点状态。
%
% 核心公式：
%   Xnode = [[q0;qd0], Xinternal, [qPre;qdPre]]。
%
% 在优化链路中的作用：
%   cost、nonlcon、结果分析和导出统一由本函数解释 z；端点不再通过 ceq 强制。
expectedLength = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints;
validateattributes(z, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'z');
if expectedLength ~= 474
    error('unpackHSDecisionCompressed:InvalidProblemSize', ...
        'compressed HS 问题规模必须为 474，当前配置为 %d。', expectedLength);
end
z = z(:);

cursor = 0;
internalCount = 12 * (disc.numNodes - 2);
Xinternal = reshape(z(cursor + (1:internalCount)), 12, disc.numNodes - 2);
cursor = cursor + internalCount;

nodeForceCount = 6 * disc.numNodes;
Fnode = reshape(z(cursor + (1:nodeForceCount)), 6, disc.numNodes);
cursor = cursor + nodeForceCount;

midForceCount = 6 * disc.numMidpoints;
Fmid = reshape(z(cursor + (1:midForceCount)), 6, disc.numMidpoints);

xStart = [scene.q0; scene.qd0];
xEnd = [scene.qPre; scene.qdPre];
Xnode = [xStart, Xinternal, xEnd];
end
