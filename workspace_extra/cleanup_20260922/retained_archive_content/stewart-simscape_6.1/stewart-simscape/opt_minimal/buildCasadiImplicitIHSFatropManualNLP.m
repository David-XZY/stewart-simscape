function nlpData = buildCasadiImplicitIHSFatropManualNLP(model, scene, disc, initialGuess, solverOptions)
% buildCasadiImplicitIHSFatropManualNLP - 构建 FATROP manual 阶段结构的 IHSID NLP
%
% 本文件不改变 IHSID 物理模型、目标函数和约束公式，只调整变量与约束顺序：
% z=[Y_0;U_0;...;Y_N;U_N]，每个区间先放 24 维 identity gap，再放本阶段 local constraints。
import casadi.*
if nargin < 4 || ~isfield(initialGuess, 'nominalStage1')
    error('buildCasadiImplicitIHSFatropManualNLP:MissingNominalReference', ...
        '必须传入 buildInitialGuessTwoPhaseIHSImplicit 生成的 initialGuess.nominalStage1。');
end
if nargin < 5
    solverOptions = struct();
end
nominalStage1 = initialGuess.nominalStage1;

N = disc.numIntervals;
nx = 24;
nuInterval = 96;
nuTerminal = terminalNu(disc);
nuVec = [nuInterval * ones(N, 1); nuTerminal];
z = MX.sym('z', (N+1)*nx + N*nuInterval + nuTerminal, 1);
[Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xright, Aright, Fright, separator] = unpackManualSymbolic(z, disc);
fNode = [Xnode(7:12, :); Anode];

gParts = {};
lbgParts = {};
ubgParts = {};
equalityParts = {};
eqParts = {};
ineqParts = {};
ng = zeros(N+1, 1);
J = MX(0);

nodeStage = cell(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    pointNode = pointExpressions(Xnode(:, nodeIndex), Fnode(:, nodeIndex), Anode(:, nodeIndex), model, scene);
    nominalTerm = nominalNodeCostExpr(Xnode(1:6, nodeIndex), nodeIndex, nominalStage1, model, scene, disc);
    nodeStage{nodeIndex} = runningCostExpr(nominalTerm, pointNode.Ldd, pointNode.phiSing, model);
end

for intervalIndex = 1:N
    Xc = Xmid(:, intervalIndex);
    fc = [Xc(7:12); Amid(:, intervalIndex)];
    fRight = [Xright(7:12, intervalIndex); Aright(:, intervalIndex)];
    gap = [Xnode(:, intervalIndex+1); Anode(:, intervalIndex+1); Fnode(:, intervalIndex+1)] - ...
        [Xright(:, intervalIndex); Aright(:, intervalIndex); Fright(:, intervalIndex)];
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, gap);

    beforeStageRows = countRows(gParts);
    midConsistency = Xc - 0.5*(Xnode(:, intervalIndex) + Xright(:, intervalIndex)) - ...
        disc.h/8*(fNode(:, intervalIndex) - fRight);
    hsDefect = Xright(:, intervalIndex) - Xnode(:, intervalIndex) - ...
        disc.h/6*(fNode(:, intervalIndex) + 4*fc + fRight);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, midConsistency);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, hsDefect);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts] = appendNodeLocal( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, ...
        Xnode(:, intervalIndex), Anode(:, intervalIndex), Fnode(:, intervalIndex), ...
        separator, intervalIndex, model, scene, disc);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, pointMid] = appendMidLocal( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, ...
        Xc, Amid(:, intervalIndex), Fmid(:, intervalIndex), separator, intervalIndex, model, scene, disc);
    if intervalIndex == disc.waypointNodeIndex
        [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
            gParts, lbgParts, ubgParts, equalityParts, eqParts, Xnode(1:6, intervalIndex) - scene.qWaypoint);
    end
    ng(intervalIndex) = countRows(gParts) - beforeStageRows;

    nominalMidTerm = nominalMidCostExpr(Xc(1:6), intervalIndex, nominalStage1, model, scene, disc);
    pointRight = pointExpressions(Xright(:, intervalIndex), Fright(:, intervalIndex), ...
        Aright(:, intervalIndex), model, scene);
    nominalRightTerm = nominalNodeCostExpr(Xright(1:6, intervalIndex), intervalIndex+1, nominalStage1, model, scene, disc);
    rightStage = runningCostExpr(nominalRightTerm, pointRight.Ldd, pointRight.phiSing, model);
    midpointStage = runningCostExpr(nominalMidTerm, pointMid.Ldd, pointMid.phiSing, model);
    forceRateTerm = forceRateCostExpr(Fnode(:, intervalIndex), Fmid(:, intervalIndex), ...
        Fright(:, intervalIndex), disc.h, model);
    J = J + disc.h/6*(nodeStage{intervalIndex} + 4*midpointStage + rightStage) + ...
        model.objective.weightForceRate * forceRateTerm;
end

beforeTerminalRows = countRows(gParts);
[gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts] = appendNodeLocal( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, ...
    Xnode(:, N+1), Anode(:, N+1), Fnode(:, N+1), separator, N+1, model, scene, disc);
if N+1 == disc.waypointNodeIndex
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, Xnode(1:6, N+1) - scene.qWaypoint);
end
ng(N+1) = countRows(gParts) - beforeTerminalRows;

g = vertcat(gParts{:});
lbg = vertcat(lbgParts{:});
ubg = vertcat(ubgParts{:});
equalityMask = vertcat(equalityParts{:});
gEq = vertcat(eqParts{:});
cIneqExpr = vertcat(ineqParts{:});

sizes = struct();
sizes.numZ = numel(z);
sizes.numEq = numel(gEq);
sizes.numIneq = numel(cIneqExpr);
assert(numel(equalityMask) == numel(g), 'FATROP manual equality mask 长度不匹配。');

nlp = struct('x', z, 'f', J, 'g', g);
if ~isfield(solverOptions, 'solverBackend') || isempty(solverOptions.solverBackend)
    solverOptions.solverBackend = 'fatrop';
end
if strcmpi(char(string(solverOptions.solverBackend)), 'fatrop')
    solverOptions.fatropStructure = 'manual';
    solverOptions.N = N;
    solverOptions.nx = nx * ones(N+1, 1);
    solverOptions.nu = nuVec;
    solverOptions.ng = ng;
    solverOptions.equality = equalityMask;
end
if isfield(solverOptions, 'skipSolver') && solverOptions.skipSolver
    solver = [];
    solverBackend = lower(char(string(solverOptions.solverBackend)));
    fatropStructure = '';
    if strcmp(solverBackend, 'fatrop')
        opts = struct();
        opts.structure_detection = 'manual';
        opts.N = N;
        opts.nx = num2cell(nx * ones(N+1, 1));
        opts.nu = num2cell(nuVec);
        opts.ng = num2cell(ng);
        opts.equality = num2cell(logical(equalityMask(:)));
        opts.hessian_approximation = 'limited-memory';
        opts.lbfgs_memory = 10;
        fatropStructure = 'manual';
    else
        opts = makeCommonIpoptOptions(solverOptions);
    end
else
    [solver, opts, solverBackend, fatropStructure] = createCasadiNlpSolver(nlp, solverOptions);
end
[lbz, ubz] = buildManualDecisionBounds(disc, model, scene);

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
nlpData.gMidConsistency = MX.zeros(12, N);
nlpData.J = J;
nlpData.eval = Function('implicit_ihs_fatrop_manual_eval', {z}, {J, gEq, cIneqExpr}, ...
    {'z'}, {'J', 'gEq', 'cIneq'});
nlpData.sizes = sizes;
nlpData.opts = opts;
nlpData.method = 'IHSID';
nlpData.solverBackend = solverBackend;
nlpData.fatropStructure = fatropStructure;
nlpData.equalityMask = equalityMask;
nlpData.manualStructure = struct('N', N, 'nx', nx, 'nu', nuVec, 'ng', ng, ...
    'nuInterval', nuInterval, 'nuTerminal', nuTerminal);
end

function [Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xright, Aright, Fright, separator] = unpackManualSymbolic(z, disc)
import casadi.*
N = disc.numIntervals;
nuTerminal = terminalNu(disc);
Xnode = MX.zeros(12, N+1);
Anode = MX.zeros(6, N+1);
Fnode = MX.zeros(6, N+1);
Xmid = MX.zeros(12, N);
Amid = MX.zeros(6, N);
Fmid = MX.zeros(6, N);
Xright = MX.zeros(12, N);
Aright = MX.zeros(6, N);
Fright = MX.zeros(6, N);
separator = MX.zeros(8, disc.numCollisionCertificates);
cursor = 0;
for nodeIndex = 1:N
    yk = z(cursor + (1:24));
    cursor = cursor + 24;
    uk = z(cursor + (1:96));
    cursor = cursor + 96;
    Xnode(:, nodeIndex) = yk(1:12);
    Anode(:, nodeIndex) = yk(13:18);
    Fnode(:, nodeIndex) = yk(19:24);
    Xmid(:, nodeIndex) = uk(1:12);
    Amid(:, nodeIndex) = uk(13:18);
    Fmid(:, nodeIndex) = uk(19:24);
    Xright(:, nodeIndex) = uk(73:84);
    Aright(:, nodeIndex) = uk(85:90);
    Fright(:, nodeIndex) = uk(91:96);
    separator = setSeparatorIfUsed(separator, roofNodeSepIndex(nodeIndex, disc), uk(25:32));
    separator = setSeparatorIfUsed(separator, roofMidSepIndex(nodeIndex, disc), uk(33:40));
    separator = setSeparatorIfUsed(separator, leftNodeSepIndex(nodeIndex, disc), uk(41:48));
    separator = setSeparatorIfUsed(separator, leftMidSepIndex(nodeIndex, disc), uk(49:56));
    separator = setSeparatorIfUsed(separator, rightMidSepIndex(nodeIndex, disc), uk(57:64));
    separator = setSeparatorIfUsed(separator, rightNodeSepIndex(nodeIndex, disc), uk(65:72));
end
terminalNode = N + 1;
yk = z(cursor + (1:24));
cursor = cursor + 24;
ukTerminal = z(cursor + (1:nuTerminal));
Xnode(:, terminalNode) = yk(1:12);
Anode(:, terminalNode) = yk(13:18);
Fnode(:, terminalNode) = yk(19:24);
separator = unpackTerminalSeparatorSymbolic(separator, ukTerminal, terminalNode, disc);
end

function separator = unpackTerminalSeparatorSymbolic(separator, ukTerminal, nodeIndex, disc)
cursor = 0;
roofIndex = roofNodeSepIndex(nodeIndex, disc);
if roofIndex > 0
    separator = setSeparatorIfUsed(separator, roofIndex, ukTerminal(cursor + (1:8)));
    cursor = cursor + 8;
end
separator = setSeparatorIfUsed(separator, leftNodeSepIndex(nodeIndex, disc), ukTerminal(cursor + (1:8)));
cursor = cursor + 8;
separator = setSeparatorIfUsed(separator, rightNodeSepIndex(nodeIndex, disc), ukTerminal(cursor + (1:8)));
end

function separator = setSeparatorIfUsed(separator, index, value)
if index > 0
    separator(:, index) = value;
end
end

function [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts] = appendNodeLocal( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, X, A, F, separator, nodeIndex, model, scene, disc)
point = pointExpressions(X, F, A, model, scene);
[gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, point.rDyn);
[gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, point.cPath);
roofIndex = roofNodeSepIndex(nodeIndex, disc);
if roofIndex > 0
    [gSep, cSep] = separatorConstraintsExpr(X(1:6), separator(:, roofIndex), ...
        scene.hood.roof, scene.collision.stage1ConstraintDistance, scene);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, gSep);
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, cSep);
end
[gLeft, cLeft] = separatorConstraintsExpr(X(1:6), separator(:, leftNodeSepIndex(nodeIndex, disc)), ...
    scene.hood.leftSkirt, scene.collision.safeDistance, scene);
[gRight, cRight] = separatorConstraintsExpr(X(1:6), separator(:, rightNodeSepIndex(nodeIndex, disc)), ...
    scene.hood.rightSkirt, scene.collision.safeDistance, scene);
[gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, gLeft);
[gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, gRight);
[gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, cLeft);
[gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, cRight);
if isStage2Node(nodeIndex, disc)
    [gLine, cMono] = insertionConstraintsExpr(X, scene);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, gLine);
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, cMono);
end
end

function [gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, pointMid] = appendMidLocal( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, ineqParts, Xc, Amid, Fmid, separator, intervalIndex, model, scene, disc)
pointMid = pointExpressions(Xc, Fmid, Amid, model, scene);
[gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, pointMid.rDyn);
[gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, pointMid.cPath);
roofIndex = roofMidSepIndex(intervalIndex, disc);
if roofIndex > 0
    [gSepMid, cSepMid] = separatorConstraintsExpr(Xc(1:6), separator(:, roofIndex), ...
        scene.hood.roof, scene.collision.stage1ConstraintDistance, scene);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, gSepMid);
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, cSepMid);
end
[gLeftMid, cLeftMid] = separatorConstraintsExpr(Xc(1:6), separator(:, leftMidSepIndex(intervalIndex, disc)), ...
    scene.hood.leftSkirt, scene.collision.safeDistance, scene);
[gRightMid, cRightMid] = separatorConstraintsExpr(Xc(1:6), separator(:, rightMidSepIndex(intervalIndex, disc)), ...
    scene.hood.rightSkirt, scene.collision.safeDistance, scene);
[gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, gLeftMid);
[gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
    gParts, lbgParts, ubgParts, equalityParts, eqParts, gRightMid);
[gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, cLeftMid);
[gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
    gParts, lbgParts, ubgParts, equalityParts, ineqParts, cRightMid);
if isStage2Interval(intervalIndex, disc)
    [gLineMid, cMonoMid] = insertionConstraintsExpr(Xc, scene);
    [gParts, lbgParts, ubgParts, equalityParts, eqParts] = appendEq( ...
        gParts, lbgParts, ubgParts, equalityParts, eqParts, gLineMid);
    [gParts, lbgParts, ubgParts, equalityParts, ineqParts] = appendIneq( ...
        gParts, lbgParts, ubgParts, equalityParts, ineqParts, cMonoMid);
end
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

function [lbz, ubz] = buildManualDecisionBounds(disc, model, scene)
N = disc.numIntervals;
stageDim = 120;
nuTerminal = terminalNu(disc);
totalDim = stageDim*N + 24 + nuTerminal;
lbz = -inf(totalDim, 1);
ubz = inf(totalDim, 1);
for nodeIndex = 1:N
    stageStart = (nodeIndex-1)*stageDim;
    yStart = stageStart + 1;
    uStart = stageStart + 24 + 1;
    if nodeIndex == 1
        lbz(yStart:yStart+11) = [scene.q0; scene.qd0];
        ubz(yStart:yStart+11) = [scene.q0; scene.qd0];
    end
    lbz(yStart+18:yStart+23) = model.actuator.forceMin;
    ubz(yStart+18:yStart+23) = model.actuator.forceMax;

    lbz(uStart:uStart+95) = 0;
    ubz(uStart:uStart+95) = 0;
    lbz(uStart:uStart+17) = -inf;
    ubz(uStart:uStart+17) = inf;
    lbz(uStart+18:uStart+23) = model.actuator.forceMin;
    ubz(uStart+18:uStart+23) = model.actuator.forceMax;
    lbz(uStart+72:uStart+89) = -inf;
    ubz(uStart+72:uStart+89) = inf;
    lbz(uStart+90:uStart+95) = model.actuator.forceMin;
    ubz(uStart+90:uStart+95) = model.actuator.forceMax;
    releaseSlot(uStart, 25, 32, roofNodeSepIndex(nodeIndex, disc) > 0);
    releaseSlot(uStart, 33, 40, roofMidSepIndex(nodeIndex, disc) > 0);
    releaseSlot(uStart, 41, 48, true);
    releaseSlot(uStart, 49, 56, true);
    releaseSlot(uStart, 57, 64, true);
    releaseSlot(uStart, 65, 72, true);
end

terminalYStart = stageDim*N + 1;
lbz(terminalYStart:terminalYStart+11) = [scene.qGoal; scene.qdGoal];
ubz(terminalYStart:terminalYStart+11) = [scene.qGoal; scene.qdGoal];
lbz(terminalYStart+18:terminalYStart+23) = model.actuator.forceMin;
ubz(terminalYStart+18:terminalYStart+23) = model.actuator.forceMax;
terminalUStart = terminalYStart + 24;
lbz(terminalUStart+1:end) = -inf;
ubz(terminalUStart+1:end) = inf;

    function releaseSlot(base, first, last, doRelease)
        if doRelease
            lbz(base+first-1:base+last-1) = -inf;
            ubz(base+first-1:base+last-1) = inf;
        end
    end
end

function n = terminalNu(disc)
n = 16;
if roofNodeSepIndex(disc.numIntervals + 1, disc) > 0
    n = n + 8;
end
end

function index = roofNodeSepIndex(nodeIndex, disc)
if nodeIndex <= disc.numIntervalsApproach + 1
    index = nodeIndex;
else
    index = 0;
end
end

function index = roofMidSepIndex(intervalIndex, disc)
if intervalIndex <= disc.numIntervalsApproach
    index = disc.numIntervalsApproach + 1 + intervalIndex;
else
    index = 0;
end
end

function index = leftNodeSepIndex(nodeIndex, disc)
index = disc.numStage1CollisionPoints + nodeIndex;
end

function index = rightNodeSepIndex(nodeIndex, disc)
index = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + nodeIndex;
end

function index = leftMidSepIndex(intervalIndex, disc)
index = disc.numStage1CollisionPoints + disc.numNodes + intervalIndex;
end

function index = rightMidSepIndex(intervalIndex, disc)
index = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + disc.numNodes + intervalIndex;
end

function value = runningCostExpr(nominalTerm, Ldd, phiSing, model)
value = model.objective.weightNominalStage1 * nominalTerm + ...
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

function [gSep, cSep] = separatorConstraintsExpr(q, sep, obstacle, requiredDistance, scene)
n = sep(1:3);
eta = sep(4:6);
zeta = sep(7);
rho = sep(8);
[centerS, axisS] = cylinderPoseWorldExpr(q, scene);
boxAxes = obstacle.R_S;
PperpN = n - axisS * dot3(axisS, n);
radialNorm = sqrt(dot3(PperpN, PperpN) + scene.collision.smoothingEps^2);
gap = dot3(n, obstacle.center_S - centerS) - obstacle.halfSize.' * eta - ...
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
