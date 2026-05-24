function point = evaluatePathConstraintsAtPoint(x, F, model, scene)
% evaluatePathConstraintsAtPoint - 评价单个状态/控制点的路径约束
%
% 文件用途：
%   对一个节点或中点状态 x=[q;qd] 和驱动力 F，计算状态动力学得到 qdd，
%   并评价腿长、腿速、腿加速度、奇异性和碰撞硬约束。
%
% 输入参数：
%   x [12x1] - 状态向量 [q;qd]。
%   F [6x1]  - 支链驱动力控制变量。
%   model struct - Stewart 模型。
%   scene struct - 预对准碰撞场景。
%
% 输出参数：
%   point struct - 包含 cPath、qdd、L、Ld、Ldd、sigmaMin、condJ、
%   collisionDistances、minClearance 等诊断量。
%
% 核心公式：
%   路径约束 c<=0，包括 L 上下限、Ld 上下限、Ldd 上下限、sigmaMin 下限
%   和 29 条 safeDistance-d_j 碰撞约束。condJ 仅报告，不进入 cPath。
%
% 在优化链路中的作用：
%   目标函数、非线性约束、结果分析和 dense 验证共用本函数，保证定义一致。

validateattributes(x, {'double'}, {'real', 'finite', 'vector', 'numel', 12}, mfilename, 'x');
validateattributes(F, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'F');
x = x(:);
F = F(:);

q = x(1:6);
qd = x(7:12);
[xdot, dynAux] = stateDynamicsCompositeRigidBody(x, F, model);
qdd = xdot(7:12);

kin = sgpIK(q, model);
legKin = computeLegKinematics(q, qd, qdd, model);
Jout = sgpJacobian(q, model);
clearance = evaluateCollisionClearance(q, scene);

cLengthUpper = kin.L - model.lmax;
cLengthLower = model.lmin - kin.L;
cVelUpper = legKin.Ld - model.actuator.ldotMax;
cVelLower = -legKin.Ld - model.actuator.ldotMax;
cAccUpper = legKin.Ldd - model.actuator.lddotMax;
cAccLower = -legKin.Ldd - model.actuator.lddotMax;
cSigma = model.singularity.sigmaMinSafe - Jout.sigmaMin;
cCollision = scene.collision.safeDistance - clearance.distances;

point = struct();
point.cPath = [cLengthUpper;
               cLengthLower;
               cVelUpper;
               cVelLower;
               cAccUpper;
               cAccLower;
               cSigma;
               cCollision];
point.q = q;
point.qd = qd;
point.qdd = qdd;
point.xdot = xdot;
point.F = F;
point.L = kin.L;
point.Ld = legKin.Ld;
point.Ldd = legKin.Ldd;
point.sigmaMin = Jout.sigmaMin;
point.condJ = Jout.condJ;
point.collisionDistances = clearance.distances;
point.minClearance = clearance.minDistance;
point.minClearanceIndex = clearance.minIndex;
point.kin = kin;
point.legKin = legKin;
point.Jout = Jout;
point.dynAux = dynAux;
end
