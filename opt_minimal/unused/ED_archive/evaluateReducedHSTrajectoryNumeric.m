function data = evaluateReducedHSTrajectoryNumeric(z, model, scene, disc)
% evaluateReducedHSTrajectoryNumeric - 重建 CHSED reduced 解并计算节点/中点诊断
%
% 输入：
%   z、model、scene、disc - CHSED reduced 决策变量和公共模型配置。
%
% 输出：
%   data - 与 CHSID 重建结构兼容的轨迹、加速度、力、分离证书和诊断点。
[Xnode, Fnode, Fmid, Xinternal, separator] = unpackReducedDecisionTwoPhase(z, scene, disc);
fNode = zeros(12, disc.numNodes);
Anode = zeros(6, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    [xdot, aux] = stateDynamicsCompositeRigidBody(Xnode(:, nodeIndex), Fnode(:, nodeIndex), model);
    fNode(:, nodeIndex) = xdot;
    Anode(:, nodeIndex) = aux.qdd;
end
Xmid = computeHSMidpointStateCompressed(Xnode, fNode, disc);
fMid = zeros(12, disc.numMidpoints);
Amid = zeros(6, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    [xdot, aux] = stateDynamicsCompositeRigidBody(Xmid(:, midIndex), Fmid(:, midIndex), model);
    fMid(:, midIndex) = xdot;
    Amid(:, midIndex) = aux.qdd;
end
data = assembleReducedTrajectoryData(Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xinternal, separator, fNode, fMid, model, scene, disc);
data.method = 'CHSED';
data.maxDefectResidual = computeHSDefectMax(Xnode, fNode, fMid, disc);
end

function maxDefect = computeHSDefectMax(Xnode, fNode, fMid, disc)
values = zeros(12, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    values(:, intervalIndex) = Xnode(:, intervalIndex+1) - Xnode(:, intervalIndex) - ...
        disc.h/6*(fNode(:, intervalIndex) + 4*fMid(:, intervalIndex) + fNode(:, intervalIndex+1));
end
maxDefect = max(abs(values(:)));
end
