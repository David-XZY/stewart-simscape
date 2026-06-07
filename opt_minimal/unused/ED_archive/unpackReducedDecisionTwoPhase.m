function [Xnode, Fnode, Fmid, Xinternal, separator] = unpackReducedDecisionTwoPhase(z, scene, disc)
% unpackReducedDecisionTwoPhase - 解包消去加速度后的两阶段决策变量
%
% 输入：
%   z     - [Xinternal(:); Fnode(:); Fmid(:); separator(:)]。
%   scene - 提供固定起点和终点状态。
%   disc  - 两阶段离散参数。
%
% 输出：
%   Xnode、Fnode、Fmid、Xinternal、separator - 与 reduced NLP 一致的变量块。
expectedLength = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates;
validateattributes(z, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'z');
z = z(:);
cursor = 0;
internalCount = 12*(disc.numNodes - 2);
Xinternal = reshape(z(cursor + (1:internalCount)), 12, disc.numNodes - 2);
cursor = cursor + internalCount;

nodeForceCount = 6*disc.numNodes;
Fnode = reshape(z(cursor + (1:nodeForceCount)), 6, disc.numNodes);
cursor = cursor + nodeForceCount;

midForceCount = 6*disc.numMidpoints;
Fmid = reshape(z(cursor + (1:midForceCount)), 6, disc.numMidpoints);
cursor = cursor + midForceCount;

separatorCount = 8*disc.numCollisionCertificates;
separator = reshape(z(cursor + (1:separatorCount)), 8, disc.numCollisionCertificates);

Xnode = [[scene.q0; scene.qd0], Xinternal, [scene.qGoal; scene.qdGoal]];
end
