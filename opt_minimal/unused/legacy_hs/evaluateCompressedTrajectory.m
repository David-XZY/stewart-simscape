function data = evaluateCompressedTrajectory(z, model, scene, disc)
% evaluateCompressedTrajectory - 统一评价压缩 HS 轨迹的状态、动力学和路径约束点
%
% 文件用途：
%   从 474 维决策变量出发，解包完整节点状态与控制力，计算节点动力学导数、压缩
%   HS 中点状态、中点动力学导数，以及所有节点/中点的路径约束诊断量。
%
% 输入参数：
%   z [474x1]  - 压缩 HS 决策变量。
%   model      - Stewart 机构和合成刚体动力学模型。
%   scene      - 预对准场景与碰撞几何。
%   disc       - HS 离散参数。
%
% 输出参数：
%   data struct - 包含 Xnode、Xmid、Fnode、Fmid、fNode、fMid、nodePoint、midPoint。
%
% 核心公式：
%   nodePoint{k}=evaluatePathConstraintsAtPoint(Xnode(:,k),Fnode(:,k))；
%   fNode(:,k)=nodePoint{k}.xdot；
%   Xmid 由 computeHSMidpointStateCompressed 生成；
%   fMid(:,k)=midPoint{k}.xdot。这样节点/中点动力学只评价一次。
%
% 在优化链路中的作用：
%   costHSDynamicCompressed、nonlconHSDynamicCompressed、结果重建和验证共用本函数，确保
%   节点/中点路径约束定义一致。
[Xnode, Fnode, Fmid, Xinternal] = unpackHSDecisionCompressed(z, scene, disc);

fNode = zeros(12, disc.numNodes);
nodePoint = cell(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    nodePoint{nodeIndex} = evaluatePathConstraintsAtPoint( ...
        Xnode(:, nodeIndex), Fnode(:, nodeIndex), model, scene);
    fNode(:, nodeIndex) = nodePoint{nodeIndex}.xdot;
end

Xmid = computeHSMidpointStateCompressed(Xnode, fNode, disc);
fMid = zeros(12, disc.numMidpoints);
midPoint = cell(1, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    midPoint{midIndex} = evaluatePathConstraintsAtPoint( ...
        Xmid(:, midIndex), Fmid(:, midIndex), model, scene);
    fMid(:, midIndex) = midPoint{midIndex}.xdot;
end

data = struct();
data.Xinternal = Xinternal;
data.Xnode = Xnode;
data.Xmid = Xmid;
data.Fnode = Fnode;
data.Fmid = Fmid;
data.fNode = fNode;
data.fMid = fMid;
data.nodePoint = nodePoint;
data.midPoint = midPoint;
end
