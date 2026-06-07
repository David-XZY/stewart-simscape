function [z0, initialTraj] = buildInitialGuessQuinticHS(model, scene, disc)
% buildInitialGuessQuinticHS - 构造状态/控制 HS 五次多项式初值
%
% 文件用途：
%   使用五次多项式生成从 q0 到 qPre 的状态初值，并通过合成刚体逆动力学
%   生成节点和中点驱动力初值。
%
% 输入参数：
%   model struct - Stewart 模型。
%   scene struct - 预对准场景。
%   disc struct - HS 离散参数。
%
% 输出参数：
%   z0 [738x1] - 打包后的初始决策变量。
%   initialTraj struct - 初值状态/控制轨迹，用于诊断。
%
% 核心公式：
%   s=10tau^3-15tau^4+6tau^5，V=ds/dt*Deltaq，A=d2s/dt2*Deltaq。
%
% 在优化链路中的作用：
%   为完整 NLP 提供端点速度为零且控制力物理一致的初值；若该初值碰撞
%   不可行，只诊断报告，不弱化碰撞硬约束。

q0 = scene.q0;
qf = scene.qPre;
deltaQ = qf - q0;
nodeTimes = linspace(0, disc.duration, disc.numNodes);
midTimes = nodeTimes(1:end-1) + disc.h/2;

Qnode = zeros(6, disc.numNodes);
Vnode = zeros(6, disc.numNodes);
Anode = zeros(6, disc.numNodes);
Unode = zeros(6, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    [shape, shapeDot, shapeDDot] = quinticShape(nodeTimes(nodeIndex), disc.duration);
    Qnode(:, nodeIndex) = q0 + shape * deltaQ;
    Vnode(:, nodeIndex) = shapeDot * deltaQ;
    Anode(:, nodeIndex) = shapeDDot * deltaQ;
    Unode(:, nodeIndex) = inverseDynamicsCompositeRigidBody(Qnode(:, nodeIndex), Vnode(:, nodeIndex), Anode(:, nodeIndex), model);
end

Qmid = zeros(6, disc.numIntervals);
Vmid = zeros(6, disc.numIntervals);
Amid = zeros(6, disc.numIntervals);
Umid = zeros(6, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    [shape, shapeDot, shapeDDot] = quinticShape(midTimes(intervalIndex), disc.duration);
    Qmid(:, intervalIndex) = q0 + shape * deltaQ;
    Vmid(:, intervalIndex) = shapeDot * deltaQ;
    Amid(:, intervalIndex) = shapeDDot * deltaQ;
    Umid(:, intervalIndex) = inverseDynamicsCompositeRigidBody(Qmid(:, intervalIndex), Vmid(:, intervalIndex), Amid(:, intervalIndex), model);
end

Xnode = [Qnode; Vnode];
Xmid = [Qmid; Vmid];
z0 = packHSDecision(Xnode, Xmid, Unode, Umid);

initialTraj = struct();
initialTraj.t = nodeTimes;
initialTraj.tc = midTimes;
initialTraj.Xnode = Xnode;
initialTraj.Xmid = Xmid;
initialTraj.Unode = Unode;
initialTraj.Umid = Umid;
initialTraj.Q = Qnode;
initialTraj.V = Vnode;
initialTraj.Qc = Qmid;
initialTraj.Vc = Vmid;
end

function [shape, shapeDot, shapeDDot] = quinticShape(timeValue, totalTime)
tau = timeValue / totalTime;
shape = 10*tau^3 - 15*tau^4 + 6*tau^5;
shapeDot = (30*tau^2 - 60*tau^3 + 30*tau^4) / totalTime;
shapeDDot = (60*tau - 180*tau^2 + 120*tau^3) / totalTime^2;
end
