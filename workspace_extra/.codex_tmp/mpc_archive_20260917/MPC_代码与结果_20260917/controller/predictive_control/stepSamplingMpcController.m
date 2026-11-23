function [Fcmd, diagnostic] = stepSamplingMpcController(q, qd, refs, refIndex, Fprev, model, config, Fnom)
% stepSamplingMpcController - 用候选力序列的滚动预测代价选择当前控制力
arguments
    q double
    qd double
    refs struct
    refIndex double
    Fprev double
    model struct
    config struct
    Fnom double = []
end

ticHandle = tic;
q = q(:);
qd = qd(:);
Fprev = Fprev(:);
if isempty(Fnom)
    Fnom = nominalForceAt(refs, refIndex, q, qd, model, config);
end
Fnom = Fnom(:);

qRef = refs.q(:, refIndex);
qdRef = refs.qd(:, refIndex);
qddRef = refs.qdd(:, refIndex);
feedbackForce = computeFeedbackForce(q, qd, qRef, qdRef, qddRef, model, config);

candidateCount = numel(config.candidateScales);
candidateRows = zeros(candidateCount, 6);
predictionCell = cell(candidateCount, 1);
for candidateIndex = 1:candidateCount
    scale = config.candidateScales(candidateIndex);
    targetForce = Fnom + scale * feedbackForce;
    candidateForce = enforceForceLimits(targetForce, Fprev, config);
    [totalCost, trackingCost, forceCost, safetyCost, predictedEquivalentError] = ...
        evaluateCandidate(q, qd, Fnom, candidateForce, Fprev, refs, refIndex, model, config);
    candidateRows(candidateIndex, :) = [scale, totalCost, trackingCost, forceCost, ...
        safetyCost, predictedEquivalentError(end)];
    predictionCell{candidateIndex} = predictedEquivalentError;
end

[bestCost, selectedIndex] = min(candidateRows(:, 2));
Fcmd = enforceForceLimits(Fnom + config.candidateScales(selectedIndex) * feedbackForce, ...
    Fprev, config);
diagnostic = struct();
diagnostic.selectedIndex = selectedIndex;
diagnostic.selectedScale = config.candidateScales(selectedIndex);
diagnostic.bestCost = bestCost;
diagnostic.feedbackForce = feedbackForce;
diagnostic.nominalForce = Fnom;
diagnostic.horizonSteps = config.horizonSteps;
diagnostic.solveTime = toc(ticHandle);
diagnostic.predictedEquivalentError = predictionCell{selectedIndex};
diagnostic.candidateTable = array2table(candidateRows, 'VariableNames', { ...
    'scale', 'totalCost', 'trackingCost', 'forceCost', 'safetyCost', ...
    'terminalEquivalentError'});
end

function Fnom = nominalForceAt(refs, refIndex, q, qd, model, config)
if isfield(refs, 'Fcomputed')
    Fnom = refs.Fcomputed(:, refIndex);
elseif isfield(refs, 'Fleg')
    Fnom = refs.Fleg(:, refIndex);
else
    Fnom = computeComputedTorqueForce(refs.q(:, refIndex), refs.qd(:, refIndex), ...
        refs.qdd(:, refIndex), q, qd, model, config);
end
Fnom = min(max(Fnom(:), config.forceMin), config.forceMax);
end

function feedbackForce = computeFeedbackForce(q, qd, qRef, qdRef, qddRef, model, config)
desiredCorrection = -config.feedback.positionGain * (q - qRef) ...
    - config.feedback.velocityGain * (qd - qdRef);
desiredAcceleration = qddRef + desiredCorrection;
wrench = equivalentMassMatrix(model, config) * desiredAcceleration;
jacobian = sgpJacobian(q, model);
matrixToSolve = jacobian.Jv.';
if rcond(matrixToSolve) < model.num.rcondMin
    targetForce = pinv(matrixToSolve) * wrench;
else
    targetForce = matrixToSolve \ wrench;
end
feedforward = computeComputedTorqueForce(qRef, qdRef, qddRef, q, qd, model, config);
feedbackForce = targetForce - feedforward;
feedbackForce = min(max(feedbackForce, -config.feedback.feedbackForceLimit), ...
    config.feedback.feedbackForceLimit);
end

function [totalCost, trackingCost, forceCost, safetyCost, predictedEquivalentError] = ...
        evaluateCandidate(q, qd, Fnom, Fcandidate, Fprev, refs, refIndex, model, config)
trackingCost = 0;
safetyCost = 0;
predictedEquivalentError = zeros(config.horizonSteps, 1);
qPred = q;
qdPred = qd;
for horizonIndex = 1:config.horizonSteps
    sampleIndex = min(refIndex + horizonIndex - 1, numel(refs.t));
    qRef = refs.q(:, sampleIndex);
    qdRef = refs.qd(:, sampleIndex);
    qddRef = refs.qdd(:, sampleIndex);
    qddPred = qddRef + forceToAcceleration(qPred, Fcandidate - Fnom, model, config);
    qdPred = qdPred + qddPred * config.sampleTime;
    qPred = qPred + qdPred * config.sampleTime;

    errorPose = qPred - qRef;
    errorVelocity = qdPred - qdRef;
    equivalentError = sqrt(sum(errorPose(1:3).^2) ...
        + config.characteristicLength^2 * sum(errorPose(4:6).^2));
    predictedEquivalentError(horizonIndex) = equivalentError;
    trackingCost = trackingCost + config.weights.position * sum(errorPose(1:3).^2) ...
        + config.weights.rotation * sum(errorPose(4:6).^2) ...
        + config.weights.velocity * sum(errorVelocity.^2);
    safetyCost = safetyCost + safetyPenalty(qPred, model, config);
end
trackingCost = trackingCost + config.weights.terminal * predictedEquivalentError(end)^2;
forceRate = (Fcandidate - Fprev) / config.sampleTime;
forceCost = config.weights.force * sum(Fcandidate.^2) ...
    + config.weights.forceRate * sum(forceRate.^2);
totalCost = trackingCost + forceCost + safetyCost;
end

function penalty = safetyPenalty(q, model, config)
kin = sgpIK(q, model);
jacobian = sgpJacobian(q, model);
lengthMargin = min([kin.L - config.lengthMin; config.lengthMax - kin.L]);
lengthViolation = max(config.lengthMargin - lengthMargin, 0);
sigmaViolation = max(config.sigmaSafe - jacobian.sigmaMin, 0);
penalty = config.weights.safety * (lengthViolation^2 + sigmaViolation^2);
end

function qdd = forceToAcceleration(q, deltaForce, model, config)
jacobian = sgpJacobian(q, model);
wrench = jacobian.Jv.' * deltaForce(:);
qdd = equivalentMassMatrix(model, config) \ wrench;
end

function massMatrix = equivalentMassMatrix(model, config)
rotationalMass = max(diag(model.dynamics.inertiaAtCOM_P)) ...
    / max(config.characteristicLength, eps)^2;
massMatrix = diag([model.dynamics.totalMass * ones(1, 3), ...
    rotationalMass * ones(1, 3)]);
end

function Fcmd = enforceForceLimits(Fcmd, Fprev, config)
rateStep = config.forceRateLimit(:) * config.sampleTime;
Fcmd = min(max(Fcmd(:), Fprev(:) - rateStep), Fprev(:) + rateStep);
Fcmd = min(max(Fcmd, config.forceMin), config.forceMax);
end
