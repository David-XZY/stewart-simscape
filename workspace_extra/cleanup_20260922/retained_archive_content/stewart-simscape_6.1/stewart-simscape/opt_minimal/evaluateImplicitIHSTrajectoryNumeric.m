function data = evaluateImplicitIHSTrajectoryNumeric(z, model, scene, disc)
% evaluateImplicitIHSTrajectoryNumeric - 重建 IHSID 解并计算中点一致性诊断
[Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xinternal, separator] = unpackIHSDecisionImplicit(z, scene, disc);

fNode = [Xnode(7:12, :); Anode];
fMid = [Xmid(7:12, :); Amid];

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

midConsistencyResidual = zeros(12, disc.numIntervals);
hsDefectResidual = zeros(12, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    midConsistencyResidual(:, intervalIndex) = Xmid(:, intervalIndex) - ...
        0.5*(Xnode(:, intervalIndex) + Xnode(:, intervalIndex+1)) - ...
        disc.h/8*(fNode(:, intervalIndex) - fNode(:, intervalIndex+1));
    hsDefectResidual(:, intervalIndex) = Xnode(:, intervalIndex+1) - Xnode(:, intervalIndex) - ...
        disc.h/6*(fNode(:, intervalIndex) + 4*fMid(:, intervalIndex) + fNode(:, intervalIndex+1));
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
data.method = 'IHSID';
data.midConsistencyResidual = midConsistencyResidual;
data.maxMidConsistencyResidual = max(abs(midConsistencyResidual(:)));
data.hsDefectResidual = hsDefectResidual;
data.maxDefectResidual = max(abs(hsDefectResidual(:)));
end
