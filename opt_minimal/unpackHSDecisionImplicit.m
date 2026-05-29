function [Xnode, Anode, Amid, Fnode, Fmid, Xinternal, separator] = unpackHSDecisionImplicit(z, scene, disc)
% unpackHSDecisionImplicit - 解包隐式 HS 决策变量并补入固定端点
%
% 用途：
%   将 CasADi/IPOPT 决策向量还原为节点状态、节点/中点加速度、节点/中点
%   驱动力和阶段 1 分离证书；首尾状态由场景常量给定，不进入决策变量。
%
% 输入：
%   z     [numZx1] - [Xinternal(:); Anode(:); Amid(:); Fnode(:); Fmid(:); separator(:)]
%   scene struct  - 含 q0、qd0、qGoal、qdGoal
%   disc  struct  - HS 离散配置
%
% 输出：
%   Xnode [12xnumNodes] - 完整节点状态，首尾列固定
%   Anode [6xnumNodes]、Amid [6xnumMidpoints]、Fnode [6xnumNodes]、Fmid [6xnumMidpoints]
%   Xinternal [12xN] - 内部状态变量块
%   separator [8xP] - 阶段 1 分离证书变量 [n;eta;zeta;rho]
%
% 核心公式：
%   Xnode = [[scene.q0;scene.qd0], Xinternal, [scene.qGoal;scene.qdGoal]]。
%
% 优化链路位置：
%   NLP 构建、数值重建、密集验证均依赖本函数解释变量布局。

expectedLength = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + ...
    6*disc.numNodes + 6*disc.numMidpoints;
if isfield(disc, 'numStage1CollisionPoints')
    expectedLength = expectedLength + 8*disc.numStage1CollisionPoints;
end
validateattributes(z, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'z');
z = z(:);

cursor = 0;
internalCount = 12 * (disc.numNodes - 2);
Xinternal = reshape(z(cursor + (1:internalCount)), 12, disc.numNodes - 2);
cursor = cursor + internalCount;

nodeAccelCount = 6 * disc.numNodes;
Anode = reshape(z(cursor + (1:nodeAccelCount)), 6, disc.numNodes);
cursor = cursor + nodeAccelCount;

midAccelCount = 6 * disc.numMidpoints;
Amid = reshape(z(cursor + (1:midAccelCount)), 6, disc.numMidpoints);
cursor = cursor + midAccelCount;

nodeForceCount = 6 * disc.numNodes;
Fnode = reshape(z(cursor + (1:nodeForceCount)), 6, disc.numNodes);
cursor = cursor + nodeForceCount;

midForceCount = 6 * disc.numMidpoints;
Fmid = reshape(z(cursor + (1:midForceCount)), 6, disc.numMidpoints);
cursor = cursor + midForceCount;

if isfield(disc, 'numStage1CollisionPoints') && disc.numStage1CollisionPoints > 0
    separatorCount = 8 * disc.numStage1CollisionPoints;
    separator = reshape(z(cursor + (1:separatorCount)), 8, disc.numStage1CollisionPoints);
else
    separator = zeros(8, 0);
end

xStart = [scene.q0; scene.qd0];
xEnd = [scene.qGoal; scene.qdGoal];
Xnode = [xStart, Xinternal, xEnd];
end
