function [Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xinternal, separator] = unpackIHSDecisionImplicit(z, scene, disc)
% unpackIHSDecisionImplicit - 解包 IHSID 决策变量并补入固定首尾状态
expectedLength = 12*(disc.numNodes-2) + 12*disc.numMidpoints + 6*disc.numNodes + ...
    6*disc.numMidpoints + 6*disc.numNodes + 6*disc.numMidpoints;
numCollisionCertificates = getNumCollisionCertificates(disc);
expectedLength = expectedLength + 8*numCollisionCertificates;
validateattributes(z, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'z');
z = z(:);

cursor = 0;
internalCount = 12 * (disc.numNodes - 2);
Xinternal = reshape(z(cursor + (1:internalCount)), 12, disc.numNodes - 2);
cursor = cursor + internalCount;

midStateCount = 12 * disc.numMidpoints;
Xmid = reshape(z(cursor + (1:midStateCount)), 12, disc.numMidpoints);
cursor = cursor + midStateCount;

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

if numCollisionCertificates > 0
    separatorCount = 8 * numCollisionCertificates;
    separator = reshape(z(cursor + (1:separatorCount)), 8, numCollisionCertificates);
else
    separator = zeros(8, 0);
end

Xnode = [[scene.q0; scene.qd0], Xinternal, [scene.qGoal; scene.qdGoal]];
end

function n = getNumCollisionCertificates(disc)
if isfield(disc, 'numCollisionCertificates')
    n = disc.numCollisionCertificates;
elseif isfield(disc, 'numStage1CollisionPoints')
    n = disc.numStage1CollisionPoints;
else
    n = 0;
end
end
