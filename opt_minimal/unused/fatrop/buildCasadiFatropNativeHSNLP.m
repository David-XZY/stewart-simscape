function nlpData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, solverOptions)
% buildCasadiFatropNativeHSNLP - 构建 FATROP-native Hermite-Simpson OCP
%
% 本构建器保留 HS 配点思想，但把动力学写成 xdot=f(X,F) 的显式 OCP 形式。
% 每个 stage 状态为 X_k=[q_k;qd_k]，每个区间控制为 U_k=[F_k;F_c;F_{k+1}]。
% 终端 stage 带 1 维固定 dummy 控制，仅用于兼容当前 FATROP manual 接口。
% 碰撞约束使用固定方向 gap，不再引入自由分离轴证书变量。
import casadi.*
if nargin < 4 || ~isfield(initialGuess, 'nominalStage1')
    error('buildCasadiFatropNativeHSNLP:MissingNominalReference', ...
        '必须传入 buildInitialGuessFatropNativeHS 生成的 nominalStage1。');
end
if nargin < 5
    solverOptions = struct();
end

N = disc.numIntervals;
nxVec = 12 * ones(N+1, 1);
nuVec = [18 * ones(N, 1); 1];
z = MX.sym('z', sum(nxVec) + sum(nuVec), 1);
[Xnode, Fleft, Fmid, Fright] = unpackSymbolicNative(z, disc);
dynFun = buildCasadiImplicitSolvedDynamics(model);
nativeOptions = makeNativeOptions(solverOptions);
nativeOptions = attachNativeSideMask(nativeOptions, initialGuess, scene, disc);

gParts = {};
lbgParts = {};
ubgParts = {};
equalityParts = {};
eqParts = {};
ineqParts = {};
ng = zeros(N+1, 1);
J = MX(0);

for intervalIndex = 1:N
    beforeRows = countRows(gParts);

    X0 = Xnode(:, intervalIndex);
    X1 = Xnode(:, intervalIndex+1);
    F0 = Fleft(:, intervalIndex);
    Fc = Fmid(:, intervalIndex);
    F1 = Fright(:, intervalIndex);
    [~, A0] = solvedDynamics(dynFun, X0, F0);
    Xc = rk4IntegrateExpr(dynFun, X0, F0, Fc, F1, 0, 0.5, disc.h);
    X1prop = rk4IntegrateExpr(dynFun, X0, F0, Fc, F1, 0, 1.0, disc.h);
    [~, Ac] = solvedDynamics(dynFun, Xc, Fc);

    hsDefect = X1 - X1prop;
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, hsDefect);

    pointNode = pointExpressionsWithAccel(X0, F0, A0, model);
    pointMid = pointExpressionsWithAccel(Xc, Fc, Ac, model);
    if nativeOptions.includeLocalConstraints
        [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts] = appendLocalPoint( ...
            gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, ...
            X0, pointNode, intervalIndex, true, model, scene, disc, nativeOptions);
        [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts] = appendLocalPoint( ...
            gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, ...
            Xc, pointMid, intervalIndex, false, model, scene, disc, nativeOptions);
    end
    if nativeOptions.includeWaypointConstraint && intervalIndex == disc.waypointNodeIndex
        [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
            gParts, lbgParts, ubgParts, equalityParts, eqParts, X0(1:6) - scene.qWaypoint);
    end

    nodeCost = stageCostExpr(X0, pointNode, intervalIndex, true, initialGuess, model, scene, disc, nativeOptions);
    midCost = stageCostExpr(Xc, pointMid, intervalIndex, false, initialGuess, model, scene, disc, nativeOptions);
    J = J + disc.h/2*(nodeCost + midCost) + ...
        forceRateCostExpr(F0, Fc, F1, disc.h, model);
    ng(intervalIndex) = countRows(gParts) - beforeRows - nxVec(intervalIndex+1);
end

ng(N+1) = 0;

g = vertcat(gParts{:});
lbg = vertcat(lbgParts{:});
ubg = vertcat(ubgParts{:});
equalityMask = vertcat(equalityParts{:});
gEq = vertcat(eqParts{:});
cIneqExpr = vertcat(ineqParts{:});
sizes = struct('numZ', numel(z), 'numEq', numel(gEq), 'numIneq', numel(cIneqExpr));

nlp = struct('x', z, 'f', J, 'g', g);
solverOptions = completeSolverOptions(solverOptions, N, nxVec, nuVec, ng, equalityMask, sizes);
if isfield(solverOptions, 'skipSolver') && solverOptions.skipSolver
    solver = [];
    opts = makeSkipSolverOptions(solverOptions);
    solverBackend = lower(char(string(solverOptions.solverBackend)));
    fatropStructure = char(string(solverOptions.fatropStructure));
else
    [solver, opts, solverBackend, fatropStructure] = createCasadiNlpSolver(nlp, solverOptions);
end
[lbz, ubz] = buildNativeBounds(disc, model, scene);

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
nlpData.eval = Function('fatrop_native_hs_eval', {z}, {J, gEq, cIneqExpr}, ...
    {'z'}, {'J', 'gEq', 'cIneq'});
nlpData.sizes = sizes;
nlpData.opts = opts;
nlpData.method = 'FATROP_NATIVE_HS';
nlpData.solverBackend = solverBackend;
nlpData.fatropStructure = fatropStructure;
nlpData.nativeConstraintProfile = nativeOptions.constraintProfile;
nlpData.nativeInsertionMode = nativeOptions.insertionMode;
nlpData.nativeSideCollisionScope = nativeOptions.sideCollisionScope;
nlpData.nativeSideCollisionMode = nativeOptions.sideCollisionMode;
nlpData.nativeSideCollisionMask = nativeOptions.sideCollisionMask;
nlpData.equalityMask = equalityMask;
nlpData.manualStructure = struct('N', N, 'nx', nxVec, 'nu', nuVec, 'ng', ng, ...
    'numFreeCollisionCertificates', 0, 'controlDescription', '[Fleft; Fmid; Fright]');
end

function nativeOptions = makeNativeOptions(solverOptions)
if isfield(solverOptions, 'constraintProfile') && ~isempty(solverOptions.constraintProfile)
    profile = lower(char(string(solverOptions.constraintProfile)));
else
    profile = 'full';
end
profile = strrep(profile, '-', '_');
nativeOptions = struct();
nativeOptions.constraintProfile = profile;
nativeOptions.includePathConstraints = true;
nativeOptions.includeCollisionConstraints = true;
nativeOptions.includeInsertionConstraints = true;
nativeOptions.includeWaypointConstraint = true;
nativeOptions.insertionMode = 'soft';
nativeOptions.sideCollisionScope = 'all';
nativeOptions.sideCollisionMode = 'hard';
nativeOptions.weightSoftInsertion = readOption(solverOptions, 'weightSoftInsertion', 10);
nativeOptions.weightSoftSideCollision = readOption(solverOptions, 'weightSoftSideCollision', 30);
nativeOptions.collisionSmoothBeta = readOption(solverOptions, 'collisionSmoothBeta', 80);
nativeOptions.insertionLateralScale = readOption(solverOptions, 'insertionLateralScale', 0.02);
nativeOptions.insertionHeightScale = readOption(solverOptions, 'insertionHeightScale', 0.02);
nativeOptions.insertionAttitudeScale = readOption(solverOptions, 'insertionAttitudeScale', deg2rad(2));
nativeOptions.sideCollisionScale = readOption(solverOptions, 'sideCollisionScale', 0.01);
nativeOptions.softPositiveEps = readOption(solverOptions, 'softPositiveEps', 1e-6);
nativeOptions.sideMaskTolerance = readOption(solverOptions, 'sideMaskTolerance', 1e-8);
switch profile
    case 'full'
    case 'full_soft_side'
        nativeOptions.sideCollisionMode = 'soft';
    case 'full_smooth_collision'
        nativeOptions.sideCollisionMode = 'smooth_finite';
    case 'full_stage2_side'
        nativeOptions.sideCollisionScope = 'stage2';
    case 'full_masked_side'
        nativeOptions.sideCollisionScope = 'masked_nominal';
    case {'full_hard', 'hard_insertion'}
        nativeOptions.insertionMode = 'hard';
        nativeOptions.sideCollisionScope = 'all';
    case 'dynamics'
        nativeOptions.includePathConstraints = false;
        nativeOptions.includeCollisionConstraints = false;
        nativeOptions.includeInsertionConstraints = false;
        nativeOptions.includeWaypointConstraint = false;
    case 'actuator'
        nativeOptions.includeCollisionConstraints = false;
        nativeOptions.includeInsertionConstraints = false;
        nativeOptions.includeWaypointConstraint = false;
    case 'actuator_collision'
        nativeOptions.includeInsertionConstraints = false;
        nativeOptions.includeWaypointConstraint = false;
    case 'no_insertion'
        nativeOptions.includeInsertionConstraints = false;
    case 'no_collision'
        nativeOptions.includeCollisionConstraints = false;
    otherwise
        error('buildCasadiFatropNativeHSNLP:UnknownConstraintProfile', ...
            '未知 FATROP-native 约束分层: %s。', profile);
end
if isfield(solverOptions, 'insertionMode') && ~isempty(solverOptions.insertionMode)
    nativeOptions.insertionMode = lower(char(string(solverOptions.insertionMode)));
end
if isfield(solverOptions, 'sideCollisionScope') && ~isempty(solverOptions.sideCollisionScope)
    nativeOptions.sideCollisionScope = lower(char(string(solverOptions.sideCollisionScope)));
end
if isfield(solverOptions, 'sideCollisionMode') && ~isempty(solverOptions.sideCollisionMode)
    nativeOptions.sideCollisionMode = lower(char(string(solverOptions.sideCollisionMode)));
end
if ~ismember(nativeOptions.insertionMode, {'soft', 'hard'})
    error('buildCasadiFatropNativeHSNLP:UnknownInsertionMode', ...
        '未知 FATROP-native 插入约束模式: %s。', nativeOptions.insertionMode);
end
if ~ismember(nativeOptions.sideCollisionScope, {'stage2', 'all', 'masked_nominal'})
    error('buildCasadiFatropNativeHSNLP:UnknownSideCollisionScope', ...
        '未知 FATROP-native 侧壁约束范围: %s。', nativeOptions.sideCollisionScope);
end
if ~ismember(nativeOptions.sideCollisionMode, {'soft', 'hard', 'smooth_finite'})
    error('buildCasadiFatropNativeHSNLP:UnknownSideCollisionMode', ...
        '未知 FATROP-native 侧壁约束模式: %s。', nativeOptions.sideCollisionMode);
end
nativeOptions.includeLocalConstraints = nativeOptions.includePathConstraints || ...
    nativeOptions.includeCollisionConstraints || nativeOptions.includeInsertionConstraints;
end

function nativeOptions = attachNativeSideMask(nativeOptions, initialGuess, scene, disc)
allNode = true(1, disc.numIntervals);
allMid = true(1, disc.numIntervals);
nativeOptions.sideCollisionMask = struct( ...
    'leftNode', allNode, 'leftMid', allMid, ...
    'rightNode', allNode, 'rightMid', allMid);
if ~strcmp(nativeOptions.sideCollisionScope, 'masked_nominal')
    return;
end
tolerance = nativeOptions.sideMaskTolerance;
for intervalIndex = 1:disc.numIntervals
    qNode = initialGuess.Xnode(1:6, intervalIndex);
    if isfield(initialGuess, 'Xmid')
        qMid = initialGuess.Xmid(1:6, intervalIndex);
    else
        qMid = initialGuess.XmidHS(1:6, intervalIndex);
    end
    nativeOptions.sideCollisionMask.leftNode(intervalIndex) = ...
        fixedAxisCollisionIneqNumeric(qNode, scene.box.R_S * [0; 1; 0], ...
        scene.hood.leftSkirt, scene.collision.safeDistance, scene) <= tolerance;
    nativeOptions.sideCollisionMask.rightNode(intervalIndex) = ...
        fixedAxisCollisionIneqNumeric(qNode, scene.box.R_S * [0; -1; 0], ...
        scene.hood.rightSkirt, scene.collision.safeDistance, scene) <= tolerance;
    nativeOptions.sideCollisionMask.leftMid(intervalIndex) = ...
        fixedAxisCollisionIneqNumeric(qMid, scene.box.R_S * [0; 1; 0], ...
        scene.hood.leftSkirt, scene.collision.safeDistance, scene) <= tolerance;
    nativeOptions.sideCollisionMask.rightMid(intervalIndex) = ...
        fixedAxisCollisionIneqNumeric(qMid, scene.box.R_S * [0; -1; 0], ...
        scene.hood.rightSkirt, scene.collision.safeDistance, scene) <= tolerance;
end
end

function value = readOption(options, fieldName, defaultValue)
if isfield(options, fieldName) && ~isempty(options.(fieldName))
    value = options.(fieldName);
else
    value = defaultValue;
end
end

function solverOptions = completeSolverOptions(solverOptions, N, nxVec, nuVec, ng, equalityMask, sizes)
if ~isfield(solverOptions, 'solverBackend') || isempty(solverOptions.solverBackend)
    solverOptions.solverBackend = 'fatrop';
end
if strcmpi(char(string(solverOptions.solverBackend)), 'fatrop')
    solverOptions.fatropStructure = 'manual';
    solverOptions.N = N;
    solverOptions.nx = nxVec;
    solverOptions.nu = nuVec;
    solverOptions.ng = ng;
    solverOptions.equality = equalityMask;
else
    solverOptions.numEq = sizes.numEq;
    solverOptions.numIneq = sizes.numIneq;
end
end

function opts = makeSkipSolverOptions(solverOptions)
opts = struct();
opts.structure_detection = char(string(solverOptions.fatropStructure));
opts.N = solverOptions.N;
opts.nx = num2cell(double(solverOptions.nx(:)).');
opts.nu = num2cell(double(solverOptions.nu(:)).');
opts.ng = num2cell(double(solverOptions.ng(:)).');
opts.equality = num2cell(logical(solverOptions.equality(:)).');
end

function [Xnode, Fleft, Fmid, Fright] = unpackSymbolicNative(z, disc)
import casadi.*
Xnode = MX.zeros(12, disc.numNodes);
Fleft = MX.zeros(6, disc.numIntervals);
Fmid = MX.zeros(6, disc.numIntervals);
Fright = MX.zeros(6, disc.numIntervals);
cursor = 0;
for intervalIndex = 1:disc.numIntervals
    Xnode(:, intervalIndex) = z(cursor + (1:12));
    cursor = cursor + 12;
    uk = z(cursor + (1:18));
    cursor = cursor + 18;
    Fleft(:, intervalIndex) = uk(1:6);
    Fmid(:, intervalIndex) = uk(7:12);
    Fright(:, intervalIndex) = uk(13:18);
end
Xnode(:, end) = z(cursor + (1:12));
cursor = cursor + 12;
terminalDummy = z(cursor + 1); %#ok<NASGU>
end

function [xdot, A] = solvedDynamics(dynFun, X, F)
[xdot, A, ~] = dynFun(X, F);
end

function X1 = rk4IntegrateExpr(dynFun, X0, Fleft, Fmid, Fright, tau0, tau1, h)
dt = h * (tau1 - tau0);
f1 = rk4SlopeExpr(dynFun, X0, Fleft, Fmid, Fright, tau0);
f2 = rk4SlopeExpr(dynFun, X0 + 0.5*dt*f1, Fleft, Fmid, Fright, 0.5*(tau0 + tau1));
f3 = rk4SlopeExpr(dynFun, X0 + 0.5*dt*f2, Fleft, Fmid, Fright, 0.5*(tau0 + tau1));
f4 = rk4SlopeExpr(dynFun, X0 + dt*f3, Fleft, Fmid, Fright, tau1);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function slope = rk4SlopeExpr(dynFun, X, Fleft, Fmid, Fright, tau)
F = quadraticForceExpr(Fleft, Fmid, Fright, tau);
[slope, ~] = solvedDynamics(dynFun, X, F);
end

function F = quadraticForceExpr(Fleft, Fmid, Fright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
F = L0*Fleft + Lc*Fmid + L1*Fright;
end

function [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts] = appendLocalPoint( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, X, point, pointIndex, isNode, model, scene, disc, nativeOptions) %#ok<INUSD>
if nativeOptions.includePathConstraints
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, point.cPath);
end
q = X(1:6);
if nativeOptions.includeCollisionConstraints && isApproachPoint(pointIndex, isNode, disc)
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, ...
        fixedAxisCollisionIneq(q, scene.box.R_S * [0; 0; 1], scene.hood.roof, ...
        scene.collision.stage1ConstraintDistance, scene));
end
if nativeOptions.includeCollisionConstraints && shouldApplySideCollision(pointIndex, isNode, disc, nativeOptions)
    if strcmp(nativeOptions.sideCollisionMode, 'hard')
        if sideMaskAllows(pointIndex, isNode, nativeOptions, 'left')
            [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
                gParts, lbgParts, ubgParts, equalityParts, ineqParts, ...
                fixedAxisCollisionIneq(q, scene.box.R_S * [0; 1; 0], scene.hood.leftSkirt, ...
                scene.collision.safeDistance, scene));
        end
        if sideMaskAllows(pointIndex, isNode, nativeOptions, 'right')
            [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
                gParts, lbgParts, ubgParts, equalityParts, ineqParts, ...
                fixedAxisCollisionIneq(q, scene.box.R_S * [0; -1; 0], scene.hood.rightSkirt, ...
                scene.collision.safeDistance, scene));
        end
    elseif strcmp(nativeOptions.sideCollisionMode, 'smooth_finite')
        [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
            gParts, lbgParts, ubgParts, equalityParts, ineqParts, ...
            smoothFiniteCollisionIneq(q, scene.hood.leftSkirt, ...
            scene.collision.safeDistance, scene, nativeOptions));
        [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
            gParts, lbgParts, ubgParts, equalityParts, ineqParts, ...
            smoothFiniteCollisionIneq(q, scene.hood.rightSkirt, ...
            scene.collision.safeDistance, scene, nativeOptions));
    end
end
if nativeOptions.includeInsertionConstraints && isStage2Point(pointIndex, isNode, disc)
    [gLine, cMono] = insertionConstraintsExpr(X, scene);
    if strcmp(nativeOptions.insertionMode, 'hard')
        [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
            gParts, lbgParts, ubgParts, equalityParts, eqParts, gLine);
    end
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, cMono);
end
end

function tf = sideMaskAllows(pointIndex, isNode, nativeOptions, sideName)
if ~strcmp(nativeOptions.sideCollisionScope, 'masked_nominal')
    tf = true;
    return;
end
if isNode
    key = [sideName, 'Node'];
else
    key = [sideName, 'Mid'];
end
mask = nativeOptions.sideCollisionMask.(key);
tf = pointIndex <= numel(mask) && mask(pointIndex);
end

function tf = shouldApplySideCollision(pointIndex, isNode, disc, nativeOptions)
switch nativeOptions.sideCollisionScope
    case 'all'
        tf = true;
    case 'stage2'
        tf = isStage2Point(pointIndex, isNode, disc);
    case 'masked_nominal'
        tf = true;
    otherwise
        tf = false;
end
end

function tf = isApproachPoint(pointIndex, isNode, disc)
if isNode
    tf = pointIndex <= disc.numIntervalsApproach + 1;
else
    tf = pointIndex <= disc.numIntervalsApproach;
end
end

function tf = isStage2Point(pointIndex, isNode, disc)
if isNode
    tf = pointIndex >= disc.waypointNodeIndex;
else
    tf = pointIndex >= disc.numIntervalsApproach + 1;
end
end

function c = fixedAxisCollisionIneq(q, normalWorld, obstacle, requiredDistance, scene)
[centerS, axisS] = cylinderPoseWorldExpr(q, scene);
normalWorld = normalWorld(:);
eta = abs(obstacle.R_S.' * normalWorld);
mu = dot3(normalWorld, axisS);
zeta = sqrt(mu^2 + scene.collision.smoothingEps^2);
rho = sqrt(1 - mu^2 + scene.collision.smoothingEps^2);
gap = dot3(normalWorld, obstacle.center_S - centerS) - obstacle.halfSize.' * eta - ...
    0.5 * scene.objectCylinder.length * zeta - scene.objectCylinder.radius * rho;
c = requiredDistance - gap;
end

function c = smoothFiniteCollisionIneq(q, obstacle, requiredDistance, scene, nativeOptions)
[centerS, axisS] = cylinderPoseWorldExpr(q, scene);
centerVector = obstacle.center_S - centerS;
centerNormal = centerVector ./ sqrt(dot3(centerVector, centerVector) + nativeOptions.softPositiveEps^2);
candidateGaps = [ ...
    supportGapExpr(obstacle.R_S(:, 1), centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(-obstacle.R_S(:, 1), centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(obstacle.R_S(:, 2), centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(-obstacle.R_S(:, 2), centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(obstacle.R_S(:, 3), centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(-obstacle.R_S(:, 3), centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(axisS, centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(-axisS, centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(centerNormal, centerS, axisS, obstacle, scene, nativeOptions);
    supportGapExpr(-centerNormal, centerS, axisS, obstacle, scene, nativeOptions)];
gap = smoothMaxExpr(candidateGaps, nativeOptions.collisionSmoothBeta);
c = requiredDistance - gap;
end

function gap = supportGapExpr(n, cylinderCenter, cylinderAxis, obstacle, scene, nativeOptions)
n = n(:);
boxMin = dot3(n, obstacle.center_S) - sumSmoothAbs(obstacle.R_S.' * n, obstacle.halfSize, nativeOptions.softPositiveEps);
mu = dot3(n, cylinderAxis);
axial = 0.5 * scene.objectCylinder.length * smoothAbs(mu, nativeOptions.softPositiveEps);
radial = scene.objectCylinder.radius * sqrt(1 - mu^2 + nativeOptions.softPositiveEps^2);
cylinderMax = dot3(n, cylinderCenter) + axial + radial;
gap = boxMin - cylinderMax;
end

function value = smoothMaxExpr(values, beta)
import casadi.*
value = MX(0);
for i = 1:numel(values)
    value = value + exp(beta * values(i));
end
value = log(value) / beta;
end

function value = sumSmoothAbs(v, weights, epsAbs)
import casadi.*
value = MX(0);
for i = 1:numel(v)
    value = value + weights(i) * smoothAbs(v(i), epsAbs);
end
end

function value = smoothAbs(x, epsAbs)
value = sqrt(x^2 + epsAbs^2);
end

function c = fixedAxisCollisionIneqNumeric(q, normalWorld, obstacle, requiredDistance, scene)
R = rpy2rotmZYX(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
axisS = R * scene.objectCylinder.axis_P;
normalWorld = normalWorld(:);
eta = abs(obstacle.R_S.' * normalWorld);
mu = normalWorld.' * axisS;
zeta = sqrt(mu^2 + scene.collision.smoothingEps^2);
rho = sqrt(1 - mu^2 + scene.collision.smoothingEps^2);
gap = normalWorld.' * (obstacle.center_S - centerS) - obstacle.halfSize.' * eta - ...
    0.5 * scene.objectCylinder.length * zeta - scene.objectCylinder.radius * rho;
c = requiredDistance - gap;
end

function [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, expr)
gParts{end+1} = expr; %#ok<AGROW>
lbgParts{end+1} = zeros(numel(expr), 1); %#ok<AGROW>
ubgParts{end+1} = zeros(numel(expr), 1); %#ok<AGROW>
equalityParts{end+1} = true(numel(expr), 1); %#ok<AGROW>
eqParts{end+1} = expr; %#ok<AGROW>
end

function [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, expr)
gParts{end+1} = expr; %#ok<AGROW>
lbgParts{end+1} = -inf(numel(expr), 1); %#ok<AGROW>
ubgParts{end+1} = zeros(numel(expr), 1); %#ok<AGROW>
equalityParts{end+1} = false(numel(expr), 1); %#ok<AGROW>
ineqParts{end+1} = expr; %#ok<AGROW>
end

function n = countRows(parts)
n = 0;
for i = 1:numel(parts)
    n = n + numel(parts{i});
end
end

function [lbz, ubz] = buildNativeBounds(disc, model, scene)
totalLength = 12*disc.numNodes + 18*disc.numIntervals + 1;
lbz = -inf(totalLength, 1);
ubz = inf(totalLength, 1);
cursor = 0;
for intervalIndex = 1:disc.numIntervals
    xRange = cursor + (1:12);
    if intervalIndex == 1
        lbz(xRange) = [scene.q0; scene.qd0];
        ubz(xRange) = [scene.q0; scene.qd0];
    end
    cursor = cursor + 12;
    uRange = cursor + (1:18);
    lbz(uRange) = repmat(model.actuator.forceMin, 3, 1);
    ubz(uRange) = repmat(model.actuator.forceMax, 3, 1);
    cursor = cursor + 18;
end
terminalRange = cursor + (1:12);
lbz(terminalRange) = [scene.qGoal; scene.qdGoal];
ubz(terminalRange) = [scene.qGoal; scene.qdGoal];
cursor = cursor + 12;
lbz(cursor + 1) = 0;
ubz(cursor + 1) = 0;
end

function value = stageCostExpr(X, point, index, isNode, initialGuess, model, scene, disc, nativeOptions)
nominalTerm = 0;
if isNode && index <= disc.numIntervalsApproach + 1
    nominalTerm = nominalDeviationCostExpr(X(1:6), ...
        initialGuess.nominalStage1.centerNode(:, index), ...
        initialGuess.nominalStage1.rotationNode(:, :, index), model, scene);
elseif ~isNode && index <= disc.numIntervalsApproach
    nominalTerm = nominalDeviationCostExpr(X(1:6), ...
        initialGuess.nominalStage1.centerMid(:, index), ...
        initialGuess.nominalStage1.rotationMid(:, :, index), model, scene);
end
value = model.objective.weightNominalStage1 * nominalTerm + ...
    model.objective.weightLegAccel * sumSquares(point.Ldd ./ model.objective.legAccelScale) + ...
    model.objective.weightSingularity * point.phiSing;
if nativeOptions.includeInsertionConstraints && strcmp(nativeOptions.insertionMode, 'soft') && ...
        isStage2Point(index, isNode, disc)
    value = value + softInsertionCostExpr(X, scene, nativeOptions);
end
if nativeOptions.includeCollisionConstraints && strcmp(nativeOptions.sideCollisionMode, 'soft') && ...
        shouldApplySideCollision(index, isNode, disc, nativeOptions)
    value = value + softSideCollisionCostExpr(X, scene, nativeOptions);
end
end

function value = softInsertionCostExpr(X, scene, nativeOptions)
[gLine, ~] = insertionConstraintsExpr(X, scene);
scale = [nativeOptions.insertionLateralScale;
         nativeOptions.insertionHeightScale;
         repmat(nativeOptions.insertionAttitudeScale, 3, 1)];
value = nativeOptions.weightSoftInsertion * sumSquares(gLine ./ scale);
end

function value = softSideCollisionCostExpr(X, scene, nativeOptions)
q = X(1:6);
cLeft = fixedAxisCollisionIneq(q, scene.box.R_S * [0; 1; 0], scene.hood.leftSkirt, ...
    scene.collision.safeDistance, scene);
cRight = fixedAxisCollisionIneq(q, scene.box.R_S * [0; -1; 0], scene.hood.rightSkirt, ...
    scene.collision.safeDistance, scene);
epsSoft = nativeOptions.softPositiveEps;
leftViolation = 0.5 * (cLeft + sqrt(cLeft^2 + epsSoft^2));
rightViolation = 0.5 * (cRight + sqrt(cRight^2 + epsSoft^2));
value = nativeOptions.weightSoftSideCollision * ...
    sumSquares([leftViolation; rightViolation] ./ nativeOptions.sideCollisionScale);
end

function value = forceRateCostExpr(Fleft, Fmid, Fright, h, model)
halfStep = h / 2;
rateLeft = (Fmid - Fleft) ./ halfStep;
rateRight = (Fright - Fmid) ./ halfStep;
value = model.objective.weightForceRate * h/2 * ...
    (sumSquares(rateLeft ./ model.objective.forceRateScale) + ...
     sumSquares(rateRight ./ model.objective.forceRateScale));
end

function point = pointExpressionsWithAccel(X, F, A, model)
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

function value = nominalDeviationCostExpr(q, centerNominal, rotationNominal, model, scene)
R = rotmZYXExpr(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
positionTerm = sumSquares((centerS - centerNominal) ./ model.objective.positionDeviationScale);
attitudeTrace = rotationTraceExpr(rotationNominal, R);
attitudeTerm = model.objective.attitudeDeviationWeight * ...
    (3 - attitudeTrace) / (model.objective.attitudeDeviationScale^2);
value = positionTerm + attitudeTerm;
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

function z = insertionLineHeightExpr(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
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

function [Jv, Jq] = jacobianExpr(q, ~, kin)
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

function Ldd = legAccelExpr(q, qd, qdd, ~, kin, Jq)
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

function s = sumSquares(v)
import casadi.*
s = MX(0);
for index = 1:numel(v)
    s = s + v(index)^2;
end
end

function value = rotationTraceExpr(rotationNominal, R)
value = rotationNominal(1,1)*R(1,1) + rotationNominal(2,1)*R(2,1) + rotationNominal(3,1)*R(3,1) + ...
    rotationNominal(1,2)*R(1,2) + rotationNominal(2,2)*R(2,2) + rotationNominal(3,2)*R(3,2) + ...
    rotationNominal(1,3)*R(1,3) + rotationNominal(2,3)*R(2,3) + rotationNominal(3,3)*R(3,3);
end
