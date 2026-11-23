function common = buildCasadiReducedOCPCommon(model, scene, disc, initialGuess)
% buildCasadiReducedOCPCommon - CHSED 与 DMSED 共用的 CasADi 表达式工具
%
% 输入：
%   model、scene、disc、initialGuess - 当前两阶段优化模型、场景、离散和共同初值。
%
% 输出：
%   common - 函数句柄集合，用于构造 fSolved、路径约束、碰撞证书、送入约束和目标项。
%
% 在实验链路中的作用：
%   避免 CHSED 与 DMSED 各自复制一套物理/约束定义，确保二者只在离散转录方式上不同。
dynFun = buildCasadiImplicitSolvedDynamics(model);
common.fSolved = @(X, F) fSolvedFunctionExpr(dynFun, X, F);
common.point = @(X, F) pointExpressions(X, F, model, scene, dynFun);
common.pointFromAccel = @(X, F, A) pointExpressionsWithAccel(X, F, A, model, scene);
common.separator = @(q, sep, obstacle, requiredDistance) ...
    separatorConstraintsExpr(q, sep, obstacle, requiredDistance, scene);
common.insertion = @(X) insertionConstraintsExpr(X, scene);
common.nodeCost = @(X, F, nodeIndex) nodeCostExpr(X, F, nodeIndex, model, scene, disc, initialGuess, dynFun);
common.midCost = @(X, F, intervalIndex) midCostExpr(X, F, intervalIndex, model, scene, disc, initialGuess, dynFun);
common.nodeCostFromPoint = @(X, F, point, nodeIndex) nodeCostFromPointExpr(X, F, point, nodeIndex, model, scene, disc, initialGuess);
common.midCostFromPoint = @(X, F, point, intervalIndex) midCostFromPointExpr(X, F, point, intervalIndex, model, scene, disc, initialGuess);
common.forceRateCost = @(Fleft, Fmid, Fright) forceRateCostExpr(Fleft, Fmid, Fright, disc.h, model);
common.quadraticForce = @(Fleft, Fmid, Fright, tau) quadraticForceExpr(Fleft, Fmid, Fright, tau);
common.rk4 = @(X0, Fleft, Fmid, Fright, tau0, tau1) rk4Expr(X0, Fleft, Fmid, Fright, tau0, tau1, disc.h, model, dynFun);
end

function [xdot, A] = fSolvedFunctionExpr(dynFun, X, F)
[xdot, A, ~] = dynFun(X, F);
end

function point = pointExpressions(X, F, model, scene, dynFun)
q = X(1:6);
qd = X(7:12);
[~, A] = fSolvedFunctionExpr(dynFun, X, F);
point = pointExpressionsWithAccel(X, F, A, model, scene);
end

function point = pointExpressionsWithAccel(X, F, A, model, scene)
q = X(1:6);
qd = X(7:12);
kin = ikExpr(q, model);
[Jv, Jq] = jacobianExpr(q, model, kin);
Ld = Jq * qd;
Ldd = legAccelExpr(q, qd, A, model, kin, Jq);
phiSing = singularityPenaltyExpr(Jv, model);
cPath = [kin.L - model.lmax;
         model.lmin - kin.L;
         Jq*qd - model.actuator.ldotMax;
         -Jq*qd - model.actuator.ldotMax;
         Ldd - model.actuator.lddotMax;
         -Ldd - model.actuator.lddotMax];
point = struct('A', A, 'Ld', Ld, 'Ldd', Ldd, 'cPath', cPath, 'phiSing', phiSing);
end

function [gSep, cSep] = separatorConstraintsExpr(q, sep, obstacle, requiredDistance, scene)
n = sep(1:3);
eta = sep(4:6);
zeta = sep(7);
rho = sep(8);
[centerS, axisS] = cylinderPoseWorldExpr(q, scene);
PperpN = n - axisS * dot3(axisS, n);
radialNorm = sqrt(dot3(PperpN, PperpN) + scene.collision.smoothingEps^2);
gap = dot3(n, obstacle.center_S - centerS) - obstacle.halfSize.' * eta - ...
    0.5 * scene.objectCylinder.length * zeta - scene.objectCylinder.radius * rho;
gSep = dot3(n, n) - 1;
Robs = obstacle.R_S;
cSep = [dot3(n, Robs(:, 1)) - eta(1);
        -dot3(n, Robs(:, 1)) - eta(1);
        dot3(n, Robs(:, 2)) - eta(2);
        -dot3(n, Robs(:, 2)) - eta(2);
        dot3(n, Robs(:, 3)) - eta(3);
        -dot3(n, Robs(:, 3)) - eta(3);
        dot3(n, axisS) - zeta;
        -dot3(n, axisS) - zeta;
        radialNorm - rho;
        requiredDistance - gap];
end

function [gLine, cMono] = insertionConstraintsExpr(X, scene)
q = X(1:6);
qd = X(7:12);
[pCylinder_B, ~] = cylinderPoseInBoxExpr(q, scene);
lineHeight = insertionLineHeightExpr(pCylinder_B(1), scene);
gLine = [pCylinder_B(2);
         pCylinder_B(3) - lineHeight;
         q(4:6) - scene.box.rpy];
R = rotmZYXExpr(q(4:6));
pCd_S = qd(1:3) + cross3(rpyRateMapExpr(q(4:6))*qd(4:6), R*scene.objectCylinder.center_P);
pCd_B = scene.box.R_S.' * pCd_S;
cMono = -pCd_B(1);
end

function value = nodeCostExpr(X, F, nodeIndex, model, scene, disc, initialGuess, dynFun)
point = pointExpressions(X, F, model, scene, dynFun);
value = nodeCostFromPointExpr(X, F, point, nodeIndex, model, scene, disc, initialGuess);
end

function value = nodeCostFromPointExpr(X, F, point, nodeIndex, model, scene, disc, initialGuess)
nominalTerm = 0;
if nodeIndex <= disc.numIntervalsApproach + 1
    nominal = initialGuess.nominalStage1;
    nominalTerm = nominalDeviationCostExpr(X(1:6), nominal.centerNode(:, nodeIndex), ...
        nominal.rotationNode(:, :, nodeIndex), model, scene);
end
value = model.objective.weightNominalStage1 * nominalTerm + ...
    model.objective.weightLegAccel * sumSquares(point.Ldd ./ model.objective.legAccelScale) + ...
    model.objective.weightSingularity * point.phiSing;
end

function value = midCostExpr(X, F, intervalIndex, model, scene, disc, initialGuess, dynFun)
point = pointExpressions(X, F, model, scene, dynFun);
value = midCostFromPointExpr(X, F, point, intervalIndex, model, scene, disc, initialGuess);
end

function value = midCostFromPointExpr(X, F, point, intervalIndex, model, scene, disc, initialGuess)
nominalTerm = 0;
if intervalIndex <= disc.numIntervalsApproach
    nominal = initialGuess.nominalStage1;
    nominalTerm = nominalDeviationCostExpr(X(1:6), nominal.centerMid(:, intervalIndex), ...
        nominal.rotationMid(:, :, intervalIndex), model, scene);
end
value = model.objective.weightNominalStage1 * nominalTerm + ...
    model.objective.weightLegAccel * sumSquares(point.Ldd ./ model.objective.legAccelScale) + ...
    model.objective.weightSingularity * point.phiSing;
end

function value = forceRateCostExpr(Fleft, Fmid, Fright, h, model)
halfStep = h / 2;
rateLeft = (Fmid - Fleft) ./ halfStep;
rateRight = (Fright - Fmid) ./ halfStep;
value = h/2 * (sumSquares(rateLeft ./ model.objective.forceRateScale) + ...
    sumSquares(rateRight ./ model.objective.forceRateScale)) * model.objective.weightForceRate;
end

function F = quadraticForceExpr(Fleft, Fmid, Fright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
F = L0*Fleft + Lc*Fmid + L1*Fright;
end

function X1 = rk4Expr(X0, Fleft, Fmid, Fright, tau0, tau1, h, model, dynFun)
dt = h * (tau1 - tau0);
f1 = rk4Slope(X0, Fleft, Fmid, Fright, tau0, dynFun);
f2 = rk4Slope(X0 + 0.5*dt*f1, Fleft, Fmid, Fright, 0.5*(tau0+tau1), dynFun);
f3 = rk4Slope(X0 + 0.5*dt*f2, Fleft, Fmid, Fright, 0.5*(tau0+tau1), dynFun);
f4 = rk4Slope(X0 + dt*f3, Fleft, Fmid, Fright, tau1, dynFun);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function slope = rk4Slope(X, Fleft, Fmid, Fright, tau, dynFun)
F = quadraticForceExpr(Fleft, Fmid, Fright, tau);
[slope, ~] = fSolvedFunctionExpr(dynFun, X, F);
end

function value = nominalDeviationCostExpr(q, centerNominal, rotationNominal, model, scene)
R = rotmZYXExpr(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
positionTerm = sumSquares((centerS - centerNominal) ./ model.objective.positionDeviationScale);
attitudeTrace = rotationTraceExpr(rotationNominal, R);
attitudeTerm = model.objective.attitudeDeviationWeight * ...
    (3 - attitudeTrace) / (model.objective.attitudeDeviationScale^2);
value = positionTerm + attitudeTerm;
end

function kin = ikExpr(q, model)
import casadi.*
p = q(1:3);
R = rotmZYXExpr(q(4:6));
Bleg = model.B(:, model.legMap);
rB = R * Bleg;
s = MX.zeros(3, 6);
L = MX.zeros(6, 1);
u = MX.zeros(3, 6);
for legIndex = 1:6
    s(:, legIndex) = p + rB(:, legIndex) - model.A(:, legIndex);
    L(legIndex) = sqrt(dot3(s(:, legIndex), s(:, legIndex)));
    u(:, legIndex) = s(:, legIndex) ./ L(legIndex);
end
kin = struct('R', R, 'rB', rB, 's', s, 'L', L, 'u', u);
end

function [Jv, Jq] = jacobianExpr(q, model, kin)
import casadi.*
Jv = MX.zeros(6, 6);
for legIndex = 1:6
    unitDirection = kin.u(:, legIndex);
    upperJointVector = kin.rB(:, legIndex);
    Jv(legIndex, :) = [unitDirection.', cross3(upperJointVector, unitDirection).'];
end
E = rpyRateMapExpr(q(4:6));
T = [eye(3), zeros(3); zeros(3), E];
Jq = Jv * T;
end

function Ldd = legAccelExpr(q, qd, qdd, model, kin, Jq)
import casadi.*
pd = qd(1:3);
pdd = qdd(1:3);
rpyDot = qd(4:6);
rpyDDot = qdd(4:6);
[E, Edot] = rpyRateMapExpr(q(4:6), rpyDot);
omega = E * rpyDot;
alpha = Edot * rpyDot + E * rpyDDot;
Ld = Jq * qd;
Ldd = MX.zeros(6, 1);
for legIndex = 1:6
    rTop = kin.rB(:, legIndex);
    Vtop = pd + cross3(omega, rTop);
    Atop = pdd + cross3(alpha, rTop) + cross3(omega, cross3(omega, rTop));
    Ldd(legIndex) = dot3(kin.u(:, legIndex), Atop) + ...
        (dot3(Vtop, Vtop) - Ld(legIndex)^2) / kin.L(legIndex);
end
end

function Wreq = wrenchExpr(q, qd, qdd, model)
R = rotmZYXExpr(q(4:6));
[E, Edot] = rpyRateMapExpr(q(4:6), qd(4:6));
omega = E * qd(4:6);
alpha = Edot * qd(4:6) + E * qdd(4:6);
mass = model.dynamics.totalMass;
comWorld = R * model.dynamics.comP;
comAcc = qdd(1:3) + cross3(alpha, comWorld) + cross3(omega, cross3(omega, comWorld));
if model.dynamics.includeGravity
    gravity = model.g;
else
    gravity = zeros(3, 1);
end
force = mass * (comAcc - gravity);
inertiaWorld = R * model.dynamics.inertiaAtCOM_P * R.';
moment = inertiaWorld * alpha + cross3(omega, inertiaWorld * omega) + cross3(comWorld, force);
Wreq = [force; moment];
end

function [centerS, axisS] = cylinderPoseWorldExpr(q, scene)
R = rotmZYXExpr(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
axisS = R * scene.objectCylinder.axis_P;
end

function [pCylinder_B, axis_B] = cylinderPoseInBoxExpr(q, scene)
R = rotmZYXExpr(q(4:6));
pCylinder_S = q(1:3) + R * scene.objectCylinder.center_P;
axis_S = R * scene.objectCylinder.axis_P;
pCylinder_B = scene.box.R_S.' * (pCylinder_S - scene.box.center_S);
axis_B = scene.box.R_S.' * axis_S;
end

function z = insertionLineHeightExpr(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end

function phiSing = singularityPenaltyExpr(Jv, model)
import casadi.*
Lc = model.singularity.characteristicLength;
Dsing = diag([1, 1, 1, 1/Lc, 1/Lc, 1/Lc]);
Jbar = Jv * Dsing;
G = Jbar * Jbar.' + model.objective.singularityEpsilon * eye(6);
Ginv = solve(G, eye(6));
phiSing = model.objective.singularityScale * trace6(Ginv);
end

function R = rotmZYXExpr(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
     sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
     -sp,   cp*sr,            cp*cr];
end

function [E, Edot] = rpyRateMapExpr(rpy, rpyDot)
pitch = rpy(2);
yaw = rpy(3);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
E = [cy*cp, -sy, 0;
     sy*cp,  cy, 0;
     -sp,    0,  1];
if nargout > 1
    thetaDot = rpyDot(2);
    psiDot = rpyDot(3);
    dE_dtheta = [-cy*sp, 0, 0;
                 -sy*sp, 0, 0;
                 -cp,    0, 0];
    dE_dpsi = [-sy*cp, -cy, 0;
                cy*cp, -sy, 0;
                0,      0,  0];
    Edot = dE_dtheta * thetaDot + dE_dpsi * psiDot;
end
end

function c = cross3(a, b)
c = [a(2)*b(3) - a(3)*b(2);
     a(3)*b(1) - a(1)*b(3);
     a(1)*b(2) - a(2)*b(1)];
end

function d = dot3(a, b)
d = a(1)*b(1) + a(2)*b(2) + a(3)*b(3);
end

function value = trace6(A)
value = A(1,1) + A(2,2) + A(3,3) + A(4,4) + A(5,5) + A(6,6);
end

function value = rotationTraceExpr(rotationNominal, R)
value = rotationNominal(1,1)*R(1,1) + rotationNominal(2,1)*R(2,1) + rotationNominal(3,1)*R(3,1) + ...
    rotationNominal(1,2)*R(1,2) + rotationNominal(2,2)*R(2,2) + rotationNominal(3,2)*R(3,2) + ...
    rotationNominal(1,3)*R(1,3) + rotationNominal(2,3)*R(2,3) + rotationNominal(3,3)*R(3,3);
end

function s = sumSquares(v)
import casadi.*
s = MX(0);
for index = 1:numel(v)
    s = s + v(index)^2;
end
end
