function data = evaluateFatropNativeHSTrajectoryNumeric(z, model, scene, disc)
% evaluateFatropNativeHSTrajectoryNumeric - 重建 FATROP-native HS 数值轨迹
raw = unpackFatropNativeHSDecision(z, scene, disc);
Xnode = raw.Xnode;
Fleft = raw.Fleft;
Fmid = raw.Fmid;
Fright = raw.Fright;

fLeft = zeros(12, disc.numIntervals);
fRight = zeros(12, disc.numIntervals);
Aleft = zeros(6, disc.numIntervals);
Aright = zeros(6, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    [fLeft(:, intervalIndex), auxLeft] = stateDynamicsCompositeRigidBody( ...
        Xnode(:, intervalIndex), Fleft(:, intervalIndex), model);
    [fRight(:, intervalIndex), auxRight] = stateDynamicsCompositeRigidBody( ...
        Xnode(:, intervalIndex+1), Fright(:, intervalIndex), model);
    Aleft(:, intervalIndex) = auxLeft.qdd;
    Aright(:, intervalIndex) = auxRight.qdd;
end

Xmid = zeros(12, disc.numIntervals);
fMid = zeros(12, disc.numIntervals);
Amid = zeros(6, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    Xmid(:, intervalIndex) = rk4IntegrateNative(Xnode(:, intervalIndex), ...
        Fleft(:, intervalIndex), Fmid(:, intervalIndex), Fright(:, intervalIndex), 0, 0.5, disc.h, model);
    [fMid(:, intervalIndex), auxMid] = stateDynamicsCompositeRigidBody( ...
        Xmid(:, intervalIndex), Fmid(:, intervalIndex), model);
    Amid(:, intervalIndex) = auxMid.qdd;
end

Fnode = zeros(6, disc.numNodes);
Anode = zeros(6, disc.numNodes);
fNode = zeros(12, disc.numNodes);
Fnode(:, 1:disc.numIntervals) = Fleft;
Fnode(:, end) = Fright(:, end);
Anode(:, 1:disc.numIntervals) = Aleft;
Anode(:, end) = Aright(:, end);
fNode(:, 1:disc.numIntervals) = fLeft;
fNode(:, end) = fRight(:, end);

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

hsDefectResidual = zeros(12, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    XendProp = rk4IntegrateNative(Xnode(:, intervalIndex), Fleft(:, intervalIndex), ...
        Fmid(:, intervalIndex), Fright(:, intervalIndex), 0, 1.0, disc.h, model);
    hsDefectResidual(:, intervalIndex) = Xnode(:, intervalIndex+1) - XendProp;
end

data = struct();
data.method = 'FATROP_NATIVE_HS';
data.Xnode = Xnode;
data.Xinternal = Xnode(:, 2:end-1);
data.Xmid = Xmid;
data.Anode = Anode;
data.Amid = Amid;
data.Fnode = Fnode;
data.Fmid = Fmid;
data.Fleft = Fleft;
data.Fright = Fright;
data.fNode = fNode;
data.fMid = fMid;
data.fLeft = fLeft;
data.fRight = fRight;
data.nodePoint = nodePoint;
data.midPoint = midPoint;
data.nodeDynResidual = nodeDynResidual;
data.midDynResidual = midDynResidual;
data.hsDefectResidual = hsDefectResidual;
data.maxDefectResidual = max(abs(hsDefectResidual(:)));
data.maxMidConsistencyResidual = 0;
end

function X1 = rk4IntegrateNative(X0, Fleft, Fmid, Fright, tau0, tau1, h, model)
dt = h * (tau1 - tau0);
f1 = rk4SlopeNative(X0, Fleft, Fmid, Fright, tau0, model);
f2 = rk4SlopeNative(X0 + 0.5*dt*f1, Fleft, Fmid, Fright, 0.5*(tau0 + tau1), model);
f3 = rk4SlopeNative(X0 + 0.5*dt*f2, Fleft, Fmid, Fright, 0.5*(tau0 + tau1), model);
f4 = rk4SlopeNative(X0 + dt*f3, Fleft, Fmid, Fright, tau1, model);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function slope = rk4SlopeNative(X, Fleft, Fmid, Fright, tau, model)
F = quadraticForceNative(Fleft, Fmid, Fright, tau);
[slope, ~] = stateDynamicsCompositeRigidBody(X, F, model);
end

function F = quadraticForceNative(Fleft, Fmid, Fright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
F = L0*Fleft + Lc*Fmid + L1*Fright;
end
