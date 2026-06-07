function data = evaluateImplicitDMSTrajectoryNumeric(z, model, scene, disc)
% evaluateImplicitTrajectoryNumeric - 重建隐式 HS 解并计算节点/中点诊断
%
% 用途：
%   按 disc 定义的隐式变量顺序解包，使用 A=qdd 直接构造状态导数和腿加速度；
%   计算节点、中点路径约束、隐式动力学残差和目标函数所需量。
%
% 输入：
%   z [numZx1] - 隐式 HS 决策向量
%   model      - Stewart 几何、动力学和约束参数
%   scene      - 圆柱体-长方体两阶段场景与碰撞几何
%   disc       - HS 离散参数
%
% 输出：
%   data struct - Xnode、Xmid、Anode、Amid、Fnode、Fmid、fNode、fMid、point 诊断等
%
% 核心公式：
%   fNode=[qd;Anode]；Xmid=0.5*(Xk+Xk1)+h/8*(fk-fk1)；
%   rDyn=Jv(q)'*F-computeCompositeRequiredWrench(q,qd,A,model)。
%
% 优化链路位置：
%   求解前初值诊断、求解后重建、密集验证和 summary 输出复用本函数。

[Xnode, Anode, Amid, Fnode, Fmid, Xinternal, separator] = unpackHSDecisionImplicit(z, scene, disc);

fNode = [Xnode(7:12, :); Anode];
Xmid = zeros(12, disc.numMidpoints);
XendProp = zeros(12, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    Xmid(:, intervalIndex) = rk4IntegrateImplicitANumeric(Xnode(:, intervalIndex), ...
        Anode(:, intervalIndex), Amid(:, intervalIndex), Anode(:, intervalIndex+1), 0, 0.5, disc.h);
    XendProp(:, intervalIndex) = rk4IntegrateImplicitANumeric(Xmid(:, intervalIndex), ...
        Anode(:, intervalIndex), Amid(:, intervalIndex), Anode(:, intervalIndex+1), 0.5, 1.0, disc.h);
end
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
data.method = 'DMSID';
data.XendProp = XendProp;
data.shootingDefect = Xnode(:, 2:end) - XendProp;
data.maxDefectResidual = max(abs(data.shootingDefect(:)));
end

function X1 = rk4IntegrateImplicitANumeric(X0, Aleft, Amid, Aright, tau0, tau1, h)
% rk4IntegrateImplicitANumeric - DMSID 数值重建使用的 A(t) 二次插值 RK4
dt = h * (tau1 - tau0);
f1 = dmsidSlopeNumeric(X0, Aleft, Amid, Aright, tau0);
f2 = dmsidSlopeNumeric(X0 + 0.5*dt*f1, Aleft, Amid, Aright, 0.5*(tau0+tau1));
f3 = dmsidSlopeNumeric(X0 + 0.5*dt*f2, Aleft, Amid, Aright, 0.5*(tau0+tau1));
f4 = dmsidSlopeNumeric(X0 + dt*f3, Aleft, Amid, Aright, tau1);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function slope = dmsidSlopeNumeric(X, Aleft, Amid, Aright, tau)
A = quadraticAccelNumeric(Aleft, Amid, Aright, tau);
slope = [X(7:12); A];
end

function A = quadraticAccelNumeric(Aleft, Amid, Aright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
A = L0*Aleft + Lc*Amid + L1*Aright;
end
