function nlpData = buildCasadiImplicitHSNLP(model, scene, disc, initialGuess)
% buildCasadiImplicitHSNLP - 构建两阶段 CasADi MX 隐式 HS 轨迹优化 NLP
%
% 文件用途：
%   构造运动圆柱体-固定长方体两阶段轨迹优化问题，保留 Stewart 隐式动力学、
%   Hermite-Simpson 配点、CasADi/IPOPT/MA27 和三项目标函数。
%
% 输入参数：
%   model struct - Stewart 几何、执行器、动力学和目标函数参数。
%   scene struct - 圆柱体-长方体两阶段场景。
%   disc struct  - 两阶段 HS 离散配置。
%
% 输出参数：
%   nlpData struct - solver、边界、规模、符号诊断函数和变量索引。
%
% 核心公式：
%   r_HS=X_{k+1}-X_k-h/6*(f_k+4f_c+f_{k+1})；
%   r_dyn=Jv(q)'F-Wreq(q,qd,qdd)；
%   d_box=-h_z-(z_C^B+rho_z)。
%
% 在优化链路中的作用：
%   run_01_hs_dynamic_opt 调用本函数获得默认 IPOPT/MA27 求解器。
import casadi.*
if nargin < 4 || ~isfield(initialGuess, 'nominalStage1')
    error('buildCasadiImplicitHSNLP:MissingNominalReference', ...
        '必须传入 buildInitialGuessTwoPhaseHSImplicit 生成的 initialGuess.nominalStage1 作为第一阶段标称轨迹。');
end
nominalStage1 = initialGuess.nominalStage1;

sizes = computeImplicitProblemSizes(disc);
z = MX.sym('z', sizes.numZ, 1);
[Xnode, Anode, Amid, Fnode, Fmid, separator] = unpackSymbolic(z, scene, disc);
fNode = [Xnode(7:12, :); Anode];

gHS = {};
gDyn = {};
gStage2 = {};
cIneq = {};
J = MX(0);

nodeStage = cell(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    point = pointExpressions(Xnode(:, nodeIndex), Fnode(:, nodeIndex), Anode(:, nodeIndex), model, scene);
    gDyn{end+1} = point.rDyn; %#ok<AGROW>
    cIneq{end+1} = point.cPath; %#ok<AGROW>
    if nodeIndex <= disc.numIntervalsApproach + 1
        sepIndex = nodeIndex;
        [gSep, cSep] = separatorConstraintsExpr(Xnode(1:6, nodeIndex), separator(:, sepIndex), scene);
        gDyn{end+1} = gSep; %#ok<AGROW>
        cIneq{end+1} = cSep; %#ok<AGROW>
    end
    if isStage2Node(nodeIndex, disc)
        [gLine, cMono] = insertionConstraintsExpr(Xnode(:, nodeIndex), scene);
        gStage2{end+1} = gLine; %#ok<AGROW>
        cIneq{end+1} = cMono; %#ok<AGROW>
    end
    nominalTerm = nominalNodeCostExpr(Xnode(1:6, nodeIndex), nodeIndex, nominalStage1, model, scene, disc);
    nodeStage{nodeIndex} = runningCostExpr(nominalTerm, Fnode(:, nodeIndex), ...
        point.Ld, point.Ldd, point.phiSing, model);
end

for intervalIndex = 1:disc.numIntervals
    Xc = 0.5*(Xnode(:, intervalIndex) + Xnode(:, intervalIndex+1)) + ...
        disc.h/8*(fNode(:, intervalIndex) - fNode(:, intervalIndex+1));
    fc = [Xc(7:12); Amid(:, intervalIndex)];
    pointMid = pointExpressions(Xc, Fmid(:, intervalIndex), Amid(:, intervalIndex), model, scene);
    gDyn{end+1} = pointMid.rDyn; %#ok<AGROW>
    cIneq{end+1} = pointMid.cPath; %#ok<AGROW>
    if intervalIndex <= disc.numIntervalsApproach
        sepIndex = disc.numIntervalsApproach + 1 + intervalIndex;
        [gSepMid, cSepMid] = separatorConstraintsExpr(Xc(1:6), separator(:, sepIndex), scene);
        gDyn{end+1} = gSepMid; %#ok<AGROW>
        cIneq{end+1} = cSepMid; %#ok<AGROW>
    end
    if isStage2Interval(intervalIndex, disc)
        [gLineMid, cMonoMid] = insertionConstraintsExpr(Xc, scene);
        gStage2{end+1} = gLineMid; %#ok<AGROW>
        cIneq{end+1} = cMonoMid; %#ok<AGROW>
    end
    gHS{end+1} = Xnode(:, intervalIndex+1) - Xnode(:, intervalIndex) - ...
        disc.h/6*(fNode(:, intervalIndex) + 4*fc + fNode(:, intervalIndex+1)); %#ok<AGROW>
    nominalMidTerm = nominalMidCostExpr(Xc(1:6), intervalIndex, nominalStage1, model, scene, disc);
    midpointStage = runningCostExpr(nominalMidTerm, Fmid(:, intervalIndex), ...
        pointMid.Ld, pointMid.Ldd, pointMid.phiSing, model);
    forceRateTerm = forceRateCostExpr(Fnode(:, intervalIndex), Fmid(:, intervalIndex), ...
        Fnode(:, intervalIndex+1), disc.h, model);
    J = J + disc.h/6*(nodeStage{intervalIndex} + 4*midpointStage + nodeStage{intervalIndex+1}) + ...
        model.objective.weightForceRate * forceRateTerm;
end

gWaypoint = Xnode(1:6, disc.waypointNodeIndex) - scene.qWaypoint;
gEq = vertcat(gHS{:}, gDyn{:}, gWaypoint, gStage2{:});
cIneqExpr = vertcat(cIneq{:});
g = [gEq; cIneqExpr];

assert(numel(z) == sizes.numZ, '两阶段隐式 NLP 决策变量数量不匹配。');
assert(numel(gEq) == sizes.numEq, '两阶段隐式 NLP 等式数量不匹配。');
assert(numel(cIneqExpr) == sizes.numIneq, '两阶段隐式 NLP 不等式数量不匹配。');

nlp = struct('x', z, 'f', J, 'g', g);
opts = struct();
opts.print_time = true;
opts.ipopt.linear_solver = 'ma27';
opts.ipopt.max_iter = 300;
opts.ipopt.tol = 1e-6;
opts.ipopt.constr_viol_tol = 1e-6;
opts.ipopt.print_level = 4;
opts.ipopt.hessian_approximation = 'exact';
solver = nlpsol('solver', 'ipopt', nlp, opts);

[lbz, ubz] = buildDecisionBoundsImplicit(disc, model);
lbg = [zeros(sizes.numEq, 1); -inf(sizes.numIneq, 1)];
ubg = [zeros(sizes.numEq, 1); zeros(sizes.numIneq, 1)];

nlpData = struct();
nlpData.z = z;
nlpData.nlp = nlp;
nlpData.solver = solver;
nlpData.lbz = lbz;
nlpData.ubz = ubz;
nlpData.lbg = lbg;
nlpData.ubg = ubg;
nlpData.gEq = gEq;
nlpData.cIneq = cIneqExpr;
nlpData.J = J;
nlpData.eval = Function('implicit_hs_eval', {z}, {J, gEq, cIneqExpr}, ...
    {'z'}, {'J', 'gEq', 'cIneq'});
nlpData.sizes = sizes;
nlpData.opts = opts;
end

function sizes = computeImplicitProblemSizes(disc)
numIntervals = disc.numIntervals;
numNodes = disc.numNodes;
numMidpoints = disc.numMidpoints;
numStage2Nodes = disc.numIntervalsInsertion + 1;
numStage2Midpoints = disc.numIntervalsInsertion;
sizes = struct();
sizes.numZ = 12*(numNodes - 2) + 6*numNodes + 6*numMidpoints + 6*numNodes + 6*numMidpoints;
if isfield(disc, 'numStage1CollisionPoints')
    sizes.numZ = sizes.numZ + 8*disc.numStage1CollisionPoints;
end
sizes.numEq = 12*numIntervals + 6*(numNodes + numMidpoints) + 6 + ...
    5*(numStage2Nodes + numStage2Midpoints);
if isfield(disc, 'numStage1CollisionPoints')
    sizes.numEq = sizes.numEq + disc.numStage1CollisionPoints;
end
sizes.numIneq = 36*(numNodes + numMidpoints) + numStage2Nodes + numStage2Midpoints;
if isfield(disc, 'numStage1CollisionPoints')
    sizes.numIneq = sizes.numIneq + 10*disc.numStage1CollisionPoints;
end
end

function [Xnode, Anode, Amid, Fnode, Fmid, separator] = unpackSymbolic(z, scene, disc)
cursor = 0;
internalCount = 12*(disc.numNodes - 2);
Xinternal = reshape(z(cursor + (1:internalCount)), 12, disc.numNodes - 2);
cursor = cursor + internalCount;

nodeAccelCount = 6*disc.numNodes;
Anode = reshape(z(cursor + (1:nodeAccelCount)), 6, disc.numNodes);
cursor = cursor + nodeAccelCount;

midAccelCount = 6*disc.numMidpoints;
Amid = reshape(z(cursor + (1:midAccelCount)), 6, disc.numMidpoints);
cursor = cursor + midAccelCount;

nodeForceCount = 6*disc.numNodes;
Fnode = reshape(z(cursor + (1:nodeForceCount)), 6, disc.numNodes);
cursor = cursor + nodeForceCount;

midForceCount = 6*disc.numMidpoints;
Fmid = reshape(z(cursor + (1:midForceCount)), 6, disc.numMidpoints);
cursor = cursor + midForceCount;

if isfield(disc, 'numStage1CollisionPoints') && disc.numStage1CollisionPoints > 0
    sepCount = 8*disc.numStage1CollisionPoints;
    separator = reshape(z(cursor + (1:sepCount)), 8, disc.numStage1CollisionPoints);
else
    separator = MX.zeros(8, 0);
end

Xnode = [[scene.q0; scene.qd0], Xinternal, [scene.qGoal; scene.qdGoal]];
end

function [lbz, ubz] = buildDecisionBoundsImplicit(disc, model)
internalCount = 12*(disc.numNodes - 2);
nodeAccelCount = 6*disc.numNodes;
midAccelCount = 6*disc.numMidpoints;
nodeForceCount = 6*disc.numNodes;
midForceCount = 6*disc.numMidpoints;
totalLength = internalCount + nodeAccelCount + midAccelCount + nodeForceCount + midForceCount;
if isfield(disc, 'numStage1CollisionPoints')
    totalLength = totalLength + 8*disc.numStage1CollisionPoints;
end
lbz = -inf(totalLength, 1);
ubz = inf(totalLength, 1);
forceStart = internalCount + nodeAccelCount + midAccelCount + 1;
forceLower = [repmat(model.actuator.forceMin, disc.numNodes, 1); ...
              repmat(model.actuator.forceMin, disc.numMidpoints, 1)];
forceUpper = [repmat(model.actuator.forceMax, disc.numNodes, 1); ...
              repmat(model.actuator.forceMax, disc.numMidpoints, 1)];
if isfield(disc, 'numStage1CollisionPoints') && disc.numStage1CollisionPoints > 0
    forceEnd = forceStart + numel(forceLower) - 1;
    lbz(forceStart:forceEnd) = forceLower;
    ubz(forceStart:forceEnd) = forceUpper;
    lbz(forceEnd+1:end) = -inf;
    ubz(forceEnd+1:end) = inf;
else
    lbz(forceStart:end) = forceLower;
    ubz(forceStart:end) = forceUpper;
end
end

function value = runningCostExpr(nominalTerm, F, Ld, Ldd, phiSing, model)
powerTerm = sumSquares((F .* Ld) ./ model.objective.powerScale);
value = model.objective.weightNominalStage1 * nominalTerm + ...
    model.objective.weightPower * powerTerm + ...
    sumSquares(Ldd ./ model.objective.legAccelScale) * model.objective.weightLegAccel + ...
    model.objective.weightSingularity * phiSing;
end

function value = nominalNodeCostExpr(q, nodeIndex, nominalStage1, model, scene, disc)
if nodeIndex <= disc.numIntervalsApproach + 1
    value = nominalDeviationCostExpr(q, nominalStage1.centerNode(:, nodeIndex), ...
        nominalStage1.rotationNode(:, :, nodeIndex), model, scene);
else
    value = 0;
end
end

function value = nominalMidCostExpr(q, intervalIndex, nominalStage1, model, scene, disc)
if intervalIndex <= disc.numIntervalsApproach
    value = nominalDeviationCostExpr(q, nominalStage1.centerMid(:, intervalIndex), ...
        nominalStage1.rotationMid(:, :, intervalIndex), model, scene);
else
    value = 0;
end
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

function value = forceRateCostExpr(Fleft, Fmid, Fright, h, model)
halfStep = h / 2;
rateLeft = (Fmid - Fleft) ./ halfStep;
rateRight = (Fright - Fmid) ./ halfStep;
value = h/2 * (sumSquares(rateLeft ./ model.objective.forceRateScale) + ...
    sumSquares(rateRight ./ model.objective.forceRateScale));
end

function point = pointExpressions(X, F, A, model, scene)
q = X(1:6);
qd = X(7:12);
kin = ikExpr(q, model);
[Jv, Jq] = jacobianExpr(q, model, kin);
Ld = Jq * qd;
Ldd = legAccelExpr(q, qd, A, model, kin, Jq);
Wreq = wrenchExpr(q, qd, A, model);
rDyn = Jv.'*F - Wreq;
phiSing = singularityPenaltyExpr(Jv, model);
cPath = [kin.L - model.lmax;
         model.lmin - kin.L;
         Jq*qd - model.actuator.ldotMax;
         -Jq*qd - model.actuator.ldotMax;
         Ldd - model.actuator.lddotMax;
         -Ldd - model.actuator.lddotMax];

point = struct('Ld', Ld, 'Ldd', Ldd, 'cPath', cPath, 'rDyn', rDyn, 'phiSing', phiSing);
end

function [gSep, cSep] = separatorConstraintsExpr(q, sep, scene)
n = sep(1:3);
eta = sep(4:6);
zeta = sep(7);
rho = sep(8);
[centerS, axisS] = cylinderPoseWorldExpr(q, scene);
boxAxes = scene.box.R_S;
PperpN = n - axisS * dot3(axisS, n);
radialNorm = sqrt(dot3(PperpN, PperpN) + scene.collision.smoothingEps^2);
gap = dot3(n, scene.box.center_S - centerS) - scene.box.halfSize.' * eta - ...
    0.5 * scene.objectCylinder.length * zeta - scene.objectCylinder.radius * rho;
gSep = dot3(n, n) - 1;
cSep = [dot3(n, boxAxes(:, 1)) - eta(1);
        -dot3(n, boxAxes(:, 1)) - eta(1);
        dot3(n, boxAxes(:, 2)) - eta(2);
        -dot3(n, boxAxes(:, 2)) - eta(2);
        dot3(n, boxAxes(:, 3)) - eta(3);
        -dot3(n, boxAxes(:, 3)) - eta(3);
        dot3(n, axisS) - zeta;
        -dot3(n, axisS) - zeta;
        radialNorm - rho;
        scene.collision.stage1ConstraintDistance - gap];
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

function tf = isStage2Node(nodeIndex, disc)
tf = nodeIndex >= disc.waypointNodeIndex;
end

function tf = isStage2Interval(intervalIndex, disc)
tf = intervalIndex >= disc.numIntervalsApproach + 1;
end

function z = insertionLineHeightExpr(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end

function d = cylinderBoxClearanceExpr(q, scene)
[pCylinder_B, axis_B] = cylinderPoseInBoxExpr(q, scene);
mu = axis_B(3);
epsC = scene.collision.smoothingEps;
rhoZ = 0.5 * scene.objectCylinder.length * sqrt(mu^2 + epsC^2) + ...
    scene.objectCylinder.radius * sqrt(1 - mu^2 + epsC^2);
d = -scene.box.halfSize(3) - (pCylinder_B(3) + rhoZ);
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

function phiSing = singularityPenaltyExpr(Jv, model)
import casadi.*
Lc = model.singularity.characteristicLength;
Dsing = diag([1, 1, 1, 1/Lc, 1/Lc, 1/Lc]);
Jbar = Jv * Dsing;
G = Jbar * Jbar.' + model.objective.singularityEpsilon * eye(6);
Ginv = solve(G, eye(6));
phiSing = model.objective.singularityScale * trace6(Ginv);
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
