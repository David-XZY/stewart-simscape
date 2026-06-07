function [z0, initialData] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc)
% buildInitialGuessTwoPhaseHSImplicit - 生成 IHSID 共用的两阶段五次时间律基础初值
%
% 文件用途：
%   阶段 1 从 q0 到途径点；阶段 2 在长方体局部坐标系内生成圆柱中心直线送入，
%   再转换为平台位姿。节点/中点加速度和驱动力作为 IPOPT 初值。
%
% 输入参数：
%   model struct - Stewart 几何和合成刚体动力学参数。
%   scene struct - 圆柱体-长方体两阶段场景。
%   disc struct  - 两阶段 HS 离散配置。
%
% 输出参数：
%   z0 [] - 保留的基础初值输出；IHSID 决策向量由上层重新打包。
%   initialData struct - 初值轨迹、加速度和驱动力诊断。
%
% 核心公式：
%   s=10*tau^3-15*tau^4+6*tau^5；阶段 2 使用 scene 中定义的圆柱中心高度。
%
% 在优化链路中的作用：
%   buildInitialGuessTwoPhaseIHSImplicit 调用本函数生成基础轨迹。

if ~isfield(disc, 'numStage1CollisionPoints')
    disc.numStage1CollisionPoints = (disc.numIntervalsApproach + 1) + disc.numIntervalsApproach;
end
if ~isfield(disc, 'numAllCollisionPoints')
    disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
end
if ~isfield(disc, 'numCollisionCertificates')
    disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end

Qnode = zeros(6, disc.numNodes);
Vnode = zeros(6, disc.numNodes);
Anode = zeros(6, disc.numNodes);
Fnode = zeros(6, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    [Qnode(:, nodeIndex), Vnode(:, nodeIndex), Anode(:, nodeIndex)] = ...
        sampleTwoPhasePose(disc.tNode(nodeIndex), scene, disc);
    Fnode(:, nodeIndex) = inverseDynamicsCompositeRigidBody( ...
        Qnode(:, nodeIndex), Vnode(:, nodeIndex), Anode(:, nodeIndex), model);
end

QmidCurve = zeros(6, disc.numMidpoints);
VmidCurve = zeros(6, disc.numMidpoints);
Amid = zeros(6, disc.numMidpoints);
Fmid = zeros(6, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    [QmidCurve(:, midIndex), VmidCurve(:, midIndex), Amid(:, midIndex)] = ...
        sampleTwoPhasePose(disc.tMid(midIndex), scene, disc);
    Fmid(:, midIndex) = inverseDynamicsCompositeRigidBody( ...
        QmidCurve(:, midIndex), VmidCurve(:, midIndex), Amid(:, midIndex), model);
end

Xnode = [Qnode; Vnode];
separator = buildSeparatorInitialGuess(Qnode, QmidCurve, scene, disc);
z0 = [];
fNode = [Xnode(7:12, :); Anode];
XmidHS = computeHSMidpointStateCompressed(Xnode, fNode, disc);

initialData = struct();
initialData.Qnode = Qnode;
initialData.Vnode = Vnode;
initialData.Anode = Anode;
initialData.QmidCurve = QmidCurve;
initialData.VmidCurve = VmidCurve;
initialData.Amid = Amid;
initialData.Fnode = Fnode;
initialData.Fmid = Fmid;
initialData.separator = separator;
initialData.Xnode = Xnode;
initialData.XmidHS = XmidHS;
initialData.nominalStage1 = buildStage1NominalReference(Qnode, XmidHS(1:6, :), scene, disc);
end

function nominal = buildStage1NominalReference(Qnode, Qmid, scene, disc)
% buildStage1NominalReference - 直接把第一阶段初值保存为标称参考
nominal = struct();
nominal.Qnode = Qnode(:, 1:(disc.numIntervalsApproach + 1));
nominal.Qmid = Qmid(:, 1:disc.numIntervalsApproach);
nominal.centerNode = zeros(3, disc.numIntervalsApproach + 1);
nominal.centerMid = zeros(3, disc.numIntervalsApproach);
nominal.rotationNode = zeros(3, 3, disc.numIntervalsApproach + 1);
nominal.rotationMid = zeros(3, 3, disc.numIntervalsApproach);
for nodeIndex = 1:(disc.numIntervalsApproach + 1)
    [nominal.centerNode(:, nodeIndex), nominal.rotationNode(:, :, nodeIndex)] = ...
        cylinderCenterAndRotation(Qnode(:, nodeIndex), scene);
end
for midIndex = 1:disc.numIntervalsApproach
    [nominal.centerMid(:, midIndex), nominal.rotationMid(:, :, midIndex)] = ...
        cylinderCenterAndRotation(Qmid(:, midIndex), scene);
end
end

function [centerS, R] = cylinderCenterAndRotation(q, scene)
R = rpy2rotmZYX(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
end

function separator = buildSeparatorInitialGuess(Qnode, Qmid, scene, disc)
separator = zeros(8, disc.numCollisionCertificates);
normalRoof = scene.box.R_S * [0; 0; 1];
normalLeft = scene.box.R_S * [0; 1; 0];
normalRight = scene.box.R_S * [0; -1; 0];
cursor = 0;
for nodeIndex = 1:(disc.numIntervalsApproach + 1)
    cursor = cursor + 1;
    separator(:, cursor) = separatorAtPose(Qnode(:, nodeIndex), normalRoof, scene.hood.roof, scene);
end
for midIndex = 1:disc.numIntervalsApproach
    cursor = cursor + 1;
    separator(:, cursor) = separatorAtPose(Qmid(:, midIndex), normalRoof, scene.hood.roof, scene);
end
for nodeIndex = 1:disc.numNodes
    allPointIndex = nodeIndex;
    separator(:, disc.numStage1CollisionPoints + allPointIndex) = ...
        separatorAtPose(Qnode(:, nodeIndex), normalLeft, scene.hood.leftSkirt, scene);
    separator(:, disc.numStage1CollisionPoints + disc.numAllCollisionPoints + allPointIndex) = ...
        separatorAtPose(Qnode(:, nodeIndex), normalRight, scene.hood.rightSkirt, scene);
end
for midIndex = 1:disc.numMidpoints
    allPointIndex = disc.numNodes + midIndex;
    separator(:, disc.numStage1CollisionPoints + allPointIndex) = ...
        separatorAtPose(Qmid(:, midIndex), normalLeft, scene.hood.leftSkirt, scene);
    separator(:, disc.numStage1CollisionPoints + disc.numAllCollisionPoints + allPointIndex) = ...
        separatorAtPose(Qmid(:, midIndex), normalRight, scene.hood.rightSkirt, scene);
end
end

function value = separatorAtPose(q, normal, obstacle, scene)
R = rpy2rotmZYX(q(4:6));
axisWorld = R * scene.objectCylinder.axis_P;
eta = abs(obstacle.R_S.' * normal);
zeta = abs(normal.' * axisWorld);
rho = sqrt(max(0, 1 - (normal.' * axisWorld)^2) + scene.collision.smoothingEps^2);
value = [normal; eta; zeta; rho];
end

function [q, qd, qdd] = sampleTwoPhasePose(timeValue, scene, disc)
if timeValue <= disc.durationApproach + 100*eps
    [s, sd, sdd] = quinticShape(timeValue, disc.durationApproach);
    dq = scene.qWaypoint - scene.q0;
    q = scene.q0 + s * dq;
    qd = sd * dq;
    qdd = sdd * dq;
else
    localTime = timeValue - disc.durationApproach;
    [s, sd, sdd] = quinticShape(localTime, disc.durationInsertion);
    pC0 = scene.phase.pCylinderWaypoint_B;
    pC1 = scene.phase.pCylinderGoal_B;
    dpC = pC1 - pC0;
    pC_B = pC0 + s * dpC;
    pCd_B = sd * dpC;
    pCdd_B = sdd * dpC;
    q = [scene.box.center_S + scene.box.R_S * (pC_B - scene.objectCylinder.center_P); scene.box.rpy];
    qd = [scene.box.R_S * pCd_B; zeros(3, 1)];
    qdd = [scene.box.R_S * pCdd_B; zeros(3, 1)];
end
end

function [shape, shapeDot, shapeDDot] = quinticShape(timeValue, totalTime)
tau = min(max(timeValue / totalTime, 0), 1);
shape = 10*tau^3 - 15*tau^4 + 6*tau^5;
shapeDot = (30*tau^2 - 60*tau^3 + 30*tau^4) / totalTime;
shapeDDot = (60*tau - 180*tau^2 + 120*tau^3) / totalTime^2;
end
