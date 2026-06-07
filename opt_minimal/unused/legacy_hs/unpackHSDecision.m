function [Xnode, Xmid, Unode, Umid] = unpackHSDecision(z, disc)
% unpackHSDecision - 解包状态/控制 Hermite-Simpson 决策变量
%
% 文件用途：
%   将 fmincon 决策向量还原为 Xnode、Xmid、Unode、Umid。
%
% 输入参数：
%   z [738x1] - 决策变量。
%   disc struct - 至少包含 numNodes 和 numIntervals。
%
% 输出参数：
%   Xnode [12xN]     节点状态。
%   Xmid  [12x(N-1)] 显式中点状态。
%   Unode [6xN]      节点驱动力。
%   Umid  [6x(N-1)]  中点驱动力。
%
% 核心公式：
%   按 12N、12M、6N、6M 顺序切分，其中 M=N-1。
%
% 在优化链路中的作用：
%   所有 HS 目标、约束和结果重建都使用本函数保证变量含义一致。

nodeCount = disc.numNodes;
intervalCount = disc.numIntervals;
expectedLength = 12*nodeCount + 12*intervalCount + 6*nodeCount + 6*intervalCount;
validateattributes(z, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'z');
z = z(:);

cursor = 0;
Xnode = reshape(z(cursor + (1:12*nodeCount)), 12, nodeCount);
cursor = cursor + 12*nodeCount;
Xmid = reshape(z(cursor + (1:12*intervalCount)), 12, intervalCount);
cursor = cursor + 12*intervalCount;
Unode = reshape(z(cursor + (1:6*nodeCount)), 6, nodeCount);
cursor = cursor + 6*nodeCount;
Umid = reshape(z(cursor + (1:6*intervalCount)), 6, intervalCount);
end
