function data = assembleReducedTrajectoryData(Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xinternal, separator, fNode, fMid, model, scene, disc)
% assembleReducedTrajectoryData - 组装 reduced 方法的数值轨迹诊断结构
%
% 输入：
%   Xnode/Xmid、Anode/Amid、Fnode/Fmid - 节点和中点轨迹量。
%   Xinternal、separator、fNode/fMid - 决策变量块和状态导数。
%
% 输出：
%   data - 包含 point、动力学残差和重建轨迹的统一结构。
nodePoint = cell(1, disc.numNodes);
nodeDynResidual = zeros(6, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    nodePoint{nodeIndex} = evaluatePathConstraintsAtPoint( ...
        Xnode(:, nodeIndex), Fnode(:, nodeIndex), model, scene, Anode(:, nodeIndex));
    nodeDynResidual(:, nodeIndex) = nodePoint{nodeIndex}.dynAux.rDyn;
end

midPoint = cell(1, disc.numMidpoints);
midDynResidual = zeros(6, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    midPoint{midIndex} = evaluatePathConstraintsAtPoint( ...
        Xmid(:, midIndex), Fmid(:, midIndex), model, scene, Amid(:, midIndex));
    midDynResidual(:, midIndex) = midPoint{midIndex}.dynAux.rDyn;
end

data = struct();
data.Xinternal = Xinternal;
data.Xnode = Xnode;
data.Xmid = Xmid;
data.Anode = Anode;
data.Amid = Amid;
data.Fnode = Fnode;
data.Fmid = Fmid;
data.separator = separator;
data.fNode = fNode;
data.fMid = fMid;
data.nodePoint = nodePoint;
data.midPoint = midPoint;
data.nodeDynResidual = nodeDynResidual;
data.midDynResidual = midDynResidual;
end
