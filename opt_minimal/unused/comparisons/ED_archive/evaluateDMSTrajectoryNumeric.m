function data = evaluateDMSTrajectoryNumeric(z, model, scene, disc)
% evaluateDMSTrajectoryNumeric - 重建 DMSED reduced 解并计算积分中点诊断
%
% 输入：
%   z、model、scene、disc - DMSED reduced 决策变量和公共模型配置。
%
% 输出：
%   data - 节点、RK4 积分中点、加速度、力、射击残差和诊断结构。
[Xnode, Fnode, Fmid, Xinternal, separator] = unpackReducedDecisionTwoPhase(z, scene, disc);
fNode = zeros(12, disc.numNodes);
Anode = zeros(6, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    [xdot, aux] = stateDynamicsCompositeRigidBody(Xnode(:, nodeIndex), Fnode(:, nodeIndex), model);
    fNode(:, nodeIndex) = xdot;
    Anode(:, nodeIndex) = aux.qdd;
end

Xmid = zeros(12, disc.numMidpoints);
XendProp = zeros(12, disc.numIntervals);
fMid = zeros(12, disc.numMidpoints);
Amid = zeros(6, disc.numMidpoints);
for intervalIndex = 1:disc.numIntervals
    Xmid(:, intervalIndex) = rk4IntegrateNumeric(Xnode(:, intervalIndex), ...
        Fnode(:, intervalIndex), Fmid(:, intervalIndex), Fnode(:, intervalIndex+1), 0, 0.5, disc.h, model);
    XendProp(:, intervalIndex) = rk4IntegrateNumeric(Xmid(:, intervalIndex), ...
        Fnode(:, intervalIndex), Fmid(:, intervalIndex), Fnode(:, intervalIndex+1), 0.5, 1.0, disc.h, model);
    [xdotMid, auxMid] = stateDynamicsCompositeRigidBody(Xmid(:, intervalIndex), Fmid(:, intervalIndex), model);
    fMid(:, intervalIndex) = xdotMid;
    Amid(:, intervalIndex) = auxMid.qdd;
end

data = assembleReducedTrajectoryData(Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xinternal, separator, fNode, fMid, model, scene, disc);
data.method = 'DMSED';
data.XendProp = XendProp;
data.shootingDefect = Xnode(:, 2:end) - XendProp;
data.maxDefectResidual = max(abs(data.shootingDefect(:)));
end

function X1 = rk4IntegrateNumeric(X0, Fleft, Fmid, Fright, tau0, tau1, h, model)
dt = h * (tau1 - tau0);
f1 = rk4SlopeNumeric(X0, Fleft, Fmid, Fright, tau0, model);
f2 = rk4SlopeNumeric(X0 + 0.5*dt*f1, Fleft, Fmid, Fright, 0.5*(tau0+tau1), model);
f3 = rk4SlopeNumeric(X0 + 0.5*dt*f2, Fleft, Fmid, Fright, 0.5*(tau0+tau1), model);
f4 = rk4SlopeNumeric(X0 + dt*f3, Fleft, Fmid, Fright, tau1, model);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function slope = rk4SlopeNumeric(X, Fleft, Fmid, Fright, tau, model)
F = quadraticForceNumeric(Fleft, Fmid, Fright, tau);
slope = stateDynamicsCompositeRigidBody(X, F, model);
end

function F = quadraticForceNumeric(Fleft, Fmid, Fright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
F = L0*Fleft + Lc*Fmid + L1*Fright;
end
