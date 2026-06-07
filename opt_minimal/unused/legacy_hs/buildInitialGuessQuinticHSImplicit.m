function [z0, initialData] = buildInitialGuessQuinticHSImplicit(model, scene, disc)
% buildInitialGuessQuinticHSImplicit - 生成隐式 HS 五次插值初值
%
% 用途：
%   使用五次时间律生成节点状态、节点/中点加速度初值，并通过现有逆动力学为给定
%   q、qd、qdd 估计初始驱动力。端点速度固定为 scene.qd0 与 scene.qdPre；
%   端点加速度只作为初值给出，仍由优化器调整。
%
% 输入：
%   model struct - Stewart 几何、约束和合成刚体动力学参数
%   scene struct - 预对准场景
%   disc  struct - 20 区间 HS 离散
%
% 输出：
%   z0 [720x1] - 隐式 NLP 初值
%   initialData struct - 节点/中点状态、加速度和驱动力诊断
%
% 核心公式：
%   q(t)=q0+s(t)*(qPre-q0)，s=10*tau^3-15*tau^4+6*tau^5。
%   F0 由 inverseDynamicsCompositeRigidBody(q,qd,qdd,model) 反算，仅用于初值。
%
% 优化链路位置：
%   run_01_hs_dynamic_opt 在构建 NLP 前调用本函数提供 x0。

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
Amid = zeros(6, disc.numMidpoints);
Fmid = zeros(6, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    [shape, shapeDot, shapeDDot] = quinticShape(disc.tMid(midIndex), disc.duration);
    QmidCurve(:, midIndex) = q0 + shape * deltaQ;
    VmidCurve(:, midIndex) = shapeDot * deltaQ;
    Amid(:, midIndex) = shapeDDot * deltaQ;
    Fmid(:, midIndex) = inverseDynamicsCompositeRigidBody( ...
        QmidCurve(:, midIndex), VmidCurve(:, midIndex), Amid(:, midIndex), model);
end

Xnode = [Qnode; Vnode];
Xinternal = Xnode(:, 2:end-1);
z0 = packHSDecisionImplicit(Xinternal, Anode, Amid, Fnode, Fmid, disc);

initialData = struct();
initialData.Qnode = Qnode;
initialData.Vnode = Vnode;
initialData.Anode = Anode;
initialData.QmidCurve = QmidCurve;
initialData.VmidCurve = VmidCurve;
initialData.Amid = Amid;
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
