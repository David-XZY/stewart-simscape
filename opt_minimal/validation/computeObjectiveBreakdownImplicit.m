function breakdown = computeObjectiveBreakdownImplicit(trajOrData, model, scene, disc, initialGuess)
% computeObjectiveBreakdownImplicit - 计算隐式 HS 新目标函数四项分解
%
% 新目标函数包含第一阶段标称轨迹偏离、全程驱动力变化率、全程支链加速度和平滑奇异性软惩罚。
% 本函数与 buildCasadiImplicitIHSNLP 使用相同的 Simpson/区间差分积分规则。
if nargin < 5 || ~isfield(initialGuess, 'nominalStage1')
    error('computeObjectiveBreakdownImplicit:MissingNominalReference', ...
        '必须传入 initialGuess.nominalStage1 才能计算第一阶段标称偏离代价。');
end

[Xnode, Xmid, Fnode, Fmid, nodePoints, midPoints] = normalizeInput(trajOrData);
nominalStage1 = initialGuess.nominalStage1;

nodeNominal = zeros(1, disc.numNodes);
nodeAccel = zeros(1, disc.numNodes);
nodeSing = zeros(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    if nodeIndex <= disc.numIntervalsApproach + 1
        nodeNominal(nodeIndex) = nominalDeviationCostNumeric( ...
            Xnode(1:6, nodeIndex), nominalStage1.centerNode(:, nodeIndex), ...
            nominalStage1.rotationNode(:, :, nodeIndex), model, scene);
    end
    nodeAccel(nodeIndex) = sum((nodePoints{nodeIndex}.Ldd ./ model.objective.legAccelScale).^2);
    nodeSing(nodeIndex) = singularityPenaltyNumeric(Xnode(1:6, nodeIndex), model);
end

midNominal = zeros(1, disc.numMidpoints);
midAccel = zeros(1, disc.numMidpoints);
midSing = zeros(1, disc.numMidpoints);
for midIndex = 1:disc.numMidpoints
    if midIndex <= disc.numIntervalsApproach
        midNominal(midIndex) = nominalDeviationCostNumeric( ...
            Xmid(1:6, midIndex), nominalStage1.centerMid(:, midIndex), ...
            nominalStage1.rotationMid(:, :, midIndex), model, scene);
    end
    midAccel(midIndex) = sum((midPoints{midIndex}.Ldd ./ model.objective.legAccelScale).^2);
    midSing(midIndex) = singularityPenaltyNumeric(Xmid(1:6, midIndex), model);
end

rawNominal = 0;
rawForceRate = 0;
rawLegAccel = 0;
rawSingularity = 0;
for intervalIndex = 1:disc.numIntervals
    rawNominal = rawNominal + disc.h/6 * ...
        (nodeNominal(intervalIndex) + 4*midNominal(intervalIndex) + nodeNominal(intervalIndex+1));
    rawLegAccel = rawLegAccel + disc.h/6 * ...
        (nodeAccel(intervalIndex) + 4*midAccel(intervalIndex) + nodeAccel(intervalIndex+1));
    rawSingularity = rawSingularity + disc.h/6 * ...
        (nodeSing(intervalIndex) + 4*midSing(intervalIndex) + nodeSing(intervalIndex+1));
    rawForceRate = rawForceRate + forceRateCostNumeric( ...
        Fnode(:, intervalIndex), Fmid(:, intervalIndex), Fnode(:, intervalIndex+1), disc.h, model);
end

breakdown = struct();
breakdown.nominalStage1 = model.objective.weightNominalStage1 * rawNominal;
breakdown.forceRate = model.objective.weightForceRate * rawForceRate;
breakdown.legAccel = model.objective.weightLegAccel * rawLegAccel;
breakdown.singularity = model.objective.weightSingularity * rawSingularity;
breakdown.total = breakdown.nominalStage1 + breakdown.forceRate + breakdown.legAccel + breakdown.singularity;
breakdown.rawNominalStage1 = rawNominal;
breakdown.rawForceRate = rawForceRate;
breakdown.rawLegAccel = rawLegAccel;
breakdown.rawSingularity = rawSingularity;
breakdown.percent = percentageBreakdown(breakdown);
breakdown.weights = struct( ...
    'nominalStage1', model.objective.weightNominalStage1, ...
    'forceRate', model.objective.weightForceRate, ...
    'legAccel', model.objective.weightLegAccel, ...
    'singularity', model.objective.weightSingularity);
breakdown.scales = struct( ...
    'positionDeviation', model.objective.positionDeviationScale, ...
    'attitudeDeviation', model.objective.attitudeDeviationScale, ...
    'forceRate', model.objective.forceRateScale, ...
    'legAccel', model.objective.legAccelScale);
breakdown.singularityEpsilon = model.objective.singularityEpsilon;
breakdown.singularityScale = model.objective.singularityScale;
breakdown.nodePhiSing = nodeSing;
breakdown.midPhiSing = midSing;
end

function [Xnode, Xmid, Fnode, Fmid, nodePoints, midPoints] = normalizeInput(inputData)
if isfield(inputData, 'Unode')
    Xnode = inputData.Xnode;
    Xmid = inputData.Xmid;
    Fnode = inputData.Unode;
    Fmid = inputData.Umid;
    nodePoints = inputData.nodePoints;
    midPoints = inputData.midPoints;
else
    Xnode = inputData.Xnode;
    Xmid = inputData.Xmid;
    Fnode = inputData.Fnode;
    Fmid = inputData.Fmid;
    nodePoints = inputData.nodePoint;
    midPoints = inputData.midPoint;
end
end

function value = nominalDeviationCostNumeric(q, centerNominal, rotationNominal, model, scene)
R = rpy2rotmZYX(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
positionTerm = sum(((centerS - centerNominal) ./ model.objective.positionDeviationScale).^2);
attitudeTerm = model.objective.attitudeDeviationWeight * ...
    (3 - trace(rotationNominal.' * R)) / (model.objective.attitudeDeviationScale^2);
value = positionTerm + attitudeTerm;
end

function value = forceRateCostNumeric(Fleft, Fmid, Fright, h, model)
halfStep = h / 2;
rateLeft = (Fmid - Fleft) ./ halfStep;
rateRight = (Fright - Fmid) ./ halfStep;
value = h/2 * (sum((rateLeft ./ model.objective.forceRateScale).^2) + ...
    sum((rateRight ./ model.objective.forceRateScale).^2));
end

function phiSing = singularityPenaltyNumeric(q, model)
Jout = sgpJacobian(q, model);
G = Jout.Jbar * Jout.Jbar.' + model.objective.singularityEpsilon * eye(6);
phiSing = model.objective.singularityScale * trace(G \ eye(6));
end

function percent = percentageBreakdown(breakdown)
denominator = max(abs(breakdown.total), eps);
percent = struct();
percent.nominalStage1 = 100 * breakdown.nominalStage1 / denominator;
percent.forceRate = 100 * breakdown.forceRate / denominator;
percent.legAccel = 100 * breakdown.legAccel / denominator;
percent.singularity = 100 * breakdown.singularity / denominator;
end
