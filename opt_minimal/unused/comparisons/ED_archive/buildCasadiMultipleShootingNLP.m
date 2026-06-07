function nlpData = buildCasadiMultipleShootingNLP(model, scene, disc, initialGuess, solverOptions)
% buildCasadiMultipleShootingNLP - 构建消去加速度的直接多重射击 NLP
%
% 输入：
%   model、scene、disc、initialGuess - 与 CHSED 共用的模型、场景、离散和初值。
%
% 输出：
%   nlpData - CasADi 求解器、边界、规模和 eval 函数。
%
% 公式：
%   每个区间用二次力插值 F(tau)，先 RK4 积分到 tau=0.5 得到 XmidDMS，
%   再 RK4 积分到 tau=1 得到 XendProp，射击约束为 Xnode(k+1)-XendProp=0。
import casadi.*
common = buildCasadiReducedOCPCommon(model, scene, disc, initialGuess);
if nargin < 5
    solverOptions = struct();
end
sizes = computeDMSSizes(disc);
z = MX.sym('z', sizes.numZ, 1);
[Xnode, Fnode, Fmid, separator] = unpackSymbolicReduced(z, scene, disc);

gShoot = {};
gStage = {};
cIneq = {};
J = MX(0);
nodeCost = cell(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    point = common.point(Xnode(:, nodeIndex), Fnode(:, nodeIndex));
    cIneq{end+1} = point.cPath; %#ok<AGROW>
    appendCollisionAtPoint(nodeIndex, Xnode(1:6, nodeIndex), nodeIndex <= disc.numIntervalsApproach + 1);
    if nodeIndex >= disc.waypointNodeIndex
        [gLine, cMono] = common.insertion(Xnode(:, nodeIndex));
        gStage{end+1} = gLine; %#ok<AGROW>
        cIneq{end+1} = cMono; %#ok<AGROW>
    end
    nodeCost{nodeIndex} = common.nodeCostFromPoint(Xnode(:, nodeIndex), Fnode(:, nodeIndex), point, nodeIndex);
end

for intervalIndex = 1:disc.numIntervals
    XmidDMS = common.rk4(Xnode(:, intervalIndex), Fnode(:, intervalIndex), ...
        Fmid(:, intervalIndex), Fnode(:, intervalIndex+1), 0, 0.5);
    XendProp = common.rk4(XmidDMS, Fnode(:, intervalIndex), ...
        Fmid(:, intervalIndex), Fnode(:, intervalIndex+1), 0.5, 1.0);
    pointMid = common.point(XmidDMS, Fmid(:, intervalIndex));
    cIneq{end+1} = pointMid.cPath; %#ok<AGROW>
    appendCollisionAtPoint(disc.numNodes + intervalIndex, XmidDMS(1:6), intervalIndex <= disc.numIntervalsApproach);
    if intervalIndex >= disc.numIntervalsApproach + 1
        [gLineMid, cMonoMid] = common.insertion(XmidDMS);
        gStage{end+1} = gLineMid; %#ok<AGROW>
        cIneq{end+1} = cMonoMid; %#ok<AGROW>
    end
    gShoot{end+1} = Xnode(:, intervalIndex+1) - XendProp; %#ok<AGROW>
    midpointCost = common.midCostFromPoint(XmidDMS, Fmid(:, intervalIndex), pointMid, intervalIndex);
    J = J + disc.h/6*(nodeCost{intervalIndex} + 4*midpointCost + nodeCost{intervalIndex+1}) + ...
        common.forceRateCost(Fnode(:, intervalIndex), Fmid(:, intervalIndex), Fnode(:, intervalIndex+1));
end

gWaypoint = Xnode(1:6, disc.waypointNodeIndex) - scene.qWaypoint;
gEq = vertcat(gShoot{:}, gWaypoint, gStage{:});
cIneqExpr = vertcat(cIneq{:});
assert(numel(gEq) == sizes.numEq, 'DMSED 等式数量不匹配。');
assert(numel(cIneqExpr) == sizes.numIneq, 'DMSED 不等式数量不匹配。');

nlp = struct('x', z, 'f', J, 'g', [gEq; cIneqExpr]);
opts = makeCommonIpoptOptions(solverOptions);
solver = nlpsol('solver', 'ipopt', nlp, opts);
[lbz, ubz] = buildReducedBounds(disc, model);
nlpData = struct('z', z, 'nlp', nlp, 'solver', solver, 'lbz', lbz, 'ubz', ubz, ...
    'lbg', [zeros(sizes.numEq,1); -inf(sizes.numIneq,1)], ...
    'ubg', [zeros(sizes.numEq,1); zeros(sizes.numIneq,1)], ...
    'gEq', gEq, 'cIneq', cIneqExpr, 'J', J, 'sizes', sizes, 'opts', opts, ...
    'method', 'DMSED', 'dynamicsEvalCountEstimate', disc.numIntervals*8);
nlpData.eval = Function('dmse_eval', {z}, {J, gEq, cIneqExpr}, {'z'}, {'J', 'gEq', 'cIneq'});

    function appendCollisionAtPoint(allPointIndex, qPoint, useRoof)
        if useRoof
            if allPointIndex <= disc.numNodes
                roofIndex = allPointIndex;
            else
                roofIndex = disc.numIntervalsApproach + 1 + (allPointIndex - disc.numNodes);
            end
            [gRoof, cRoof] = common.separator(qPoint, separator(:, roofIndex), ...
                scene.hood.roof, scene.collision.stage1ConstraintDistance);
            gStage{end+1} = gRoof; %#ok<AGROW>
            cIneq{end+1} = cRoof; %#ok<AGROW>
        end
        leftIndex = disc.numStage1CollisionPoints + allPointIndex;
        rightIndex = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + allPointIndex;
        [gLeft, cLeft] = common.separator(qPoint, separator(:, leftIndex), ...
            scene.hood.leftSkirt, scene.collision.safeDistance);
        [gRight, cRight] = common.separator(qPoint, separator(:, rightIndex), ...
            scene.hood.rightSkirt, scene.collision.safeDistance);
        gStage{end+1} = gLeft; %#ok<AGROW>
        gStage{end+1} = gRight; %#ok<AGROW>
        cIneq{end+1} = cLeft; %#ok<AGROW>
        cIneq{end+1} = cRight; %#ok<AGROW>
    end
end

function sizes = computeDMSSizes(disc)
numStage2Points = disc.numIntervalsInsertion + 1 + disc.numIntervalsInsertion;
sizes = struct();
sizes.numZ = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates;
sizes.numEq = 12*disc.numIntervals + 6 + 5*numStage2Points + disc.numCollisionCertificates;
sizes.numIneq = 36*(disc.numNodes + disc.numMidpoints) + numStage2Points + 10*disc.numCollisionCertificates;
end

function [Xnode, Fnode, Fmid, separator] = unpackSymbolicReduced(z, scene, disc)
cursor = 0;
internalCount = 12*(disc.numNodes - 2);
Xinternal = reshape(z(cursor + (1:internalCount)), 12, disc.numNodes - 2);
cursor = cursor + internalCount;
nodeForceCount = 6*disc.numNodes;
Fnode = reshape(z(cursor + (1:nodeForceCount)), 6, disc.numNodes);
cursor = cursor + nodeForceCount;
midForceCount = 6*disc.numMidpoints;
Fmid = reshape(z(cursor + (1:midForceCount)), 6, disc.numMidpoints);
cursor = cursor + midForceCount;
sepCount = 8*disc.numCollisionCertificates;
separator = reshape(z(cursor + (1:sepCount)), 8, disc.numCollisionCertificates);
Xnode = [[scene.q0; scene.qd0], Xinternal, [scene.qGoal; scene.qdGoal]];
end

function [lbz, ubz] = buildReducedBounds(disc, model)
totalLength = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates;
lbz = -inf(totalLength, 1);
ubz = inf(totalLength, 1);
forceStart = 12*(disc.numNodes-2) + 1;
forceCount = 6*disc.numNodes + 6*disc.numMidpoints;
forceLower = [repmat(model.actuator.forceMin, disc.numNodes, 1); ...
              repmat(model.actuator.forceMin, disc.numMidpoints, 1)];
forceUpper = [repmat(model.actuator.forceMax, disc.numNodes, 1); ...
              repmat(model.actuator.forceMax, disc.numMidpoints, 1)];
lbz(forceStart:forceStart+forceCount-1) = forceLower;
ubz(forceStart:forceStart+forceCount-1) = forceUpper;
end
