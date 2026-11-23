function [z0, initialData] = buildInitialGuessQuinticHSCompressed(model, scene, disc)
% buildInitialGuessQuinticHSCompressed - 构造压缩 HS 五次插值初值
%
% 文件用途：
%   使用五次时间律生成 q0 到 qPre 的节点状态初值，并用合成刚体逆动力学生成节点和
%   中点控制力初值。初值中不包含加速度决策变量，也不包含 Xmid 决策变量。
%
% 输入参数：
%   model struct - Stewart 模型与合成刚体动力学参数。
%   scene struct - 预对准场景，包含 q0、qPre、qd0、qdPre。
%   disc struct  - HS 离散参数。
%
% 输出参数：
%   z0 [474x1] - 压缩 HS 决策变量初值。
%   initialData struct - 五次插值的节点/中点 q、qd、qdd 和控制力诊断。
%
% 核心公式：
%   s=10*tau^3-15*tau^4+6*tau^5，q=q0+s*(qPre-q0)。
%
% 在优化链路中的作用：
%   只为 fmincon 提供起始点；正式 NLP 中控制力仍由 z 决定，不在约束函数内反算替代。
q0 = scene.q0;
qf = scene.qPre;
deltaQ = qf - q0;

Qnode = zeros(6, disc.numNodes);
Vnode = zeros(6, disc.numNodes);
Anode = zeros(6, disc.numNodes);
Fnode = zeros(6, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    [shape, shapeDot, shapeDDot] = quinticShape(disc.tNode(nodeIndex), disc.duration);
    Qnode(:, nodeIndex) = q0 + shape * deltaQ;
    Vnode(:, nodeIndex) = shapeDot * deltaQ;
    Anode(:, nodeIndex) = shapeDDot * deltaQ;
    Fnode(:, nodeIndex) = inverseDynamicsCompositeRigidBody( ...
        Qnode(:, nodeIndex), Vnode(:, nodeIndex), Anode(:, nodeIndex), model);
end

QmidCurve = zeros(6, disc.numMidpoints);
VmidCurve = zeros(6, disc.numMidpoints);
AmidCurve = zeros(6, disc.numMidpoints);
Fmid = zeros(6, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    [shape, shapeDot, shapeDDot] = quinticShape(disc.tMid(midIndex), disc.duration);
    QmidCurve(:, midIndex) = q0 + shape * deltaQ;
    VmidCurve(:, midIndex) = shapeDot * deltaQ;
    AmidCurve(:, midIndex) = shapeDDot * deltaQ;
    Fmid(:, midIndex) = inverseDynamicsCompositeRigidBody( ...
        QmidCurve(:, midIndex), VmidCurve(:, midIndex), AmidCurve(:, midIndex), model);
end

Xnode = [Qnode; Vnode];
Xinternal = Xnode(:, 2:end-1);
z0 = packHSDecisionCompressed(Xinternal, Fnode, Fmid, disc);

initialData = struct();
initialData.Qnode = Qnode;
initialData.Vnode = Vnode;
initialData.Anode = Anode;
initialData.QmidCurve = QmidCurve;
initialData.VmidCurve = VmidCurve;
initialData.AmidCurve = AmidCurve;
initialData.Fnode = Fnode;
initialData.Fmid = Fmid;
initialData.Xnode = Xnode;
end

function [shape, shapeDot, shapeDDot] = quinticShape(timeValue, totalTime)
tau = timeValue / totalTime;
shape = 10*tau^3 - 15*tau^4 + 6*tau^5;
shapeDot = (30*tau^2 - 60*tau^3 + 30*tau^4) / totalTime;
shapeDDot = (60*tau - 180*tau^2 + 120*tau^3) / totalTime^2;
end
