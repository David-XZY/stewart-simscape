function report = evaluateSimscapeStrictClfCbfQp(simout, setup)
% evaluateSimscapeStrictClfCbfQp - Evaluate Simscape and strict-QP evidence.
arguments
    simout struct
    setup struct
end

baselineInput = alignMeasuredForceForLegacyEvaluator(simout);
baseline = evaluateSimscapePoseForceControl(baselineInput, setup.refs, setup.model, ...
    setup.design, setup.baselineConfig);
[diagnosticTime, diagnosticMatrix] = readSignal(simout.strictQpDiagnostic);
[poseTime, absolutePose] = readSignal(simout.qQp);
[velocityTime, generalizedVelocity] = readSignal(simout.qdQp);
[nominalTime, nominalForce] = readSignal(simout.FnomQp);
layout = setup.strictRuntime.diagnosticLayout;
decoded = decodeDiagnostics(diagnosticMatrix, layout);

referencePose = interp1(setup.refs.t(:), setup.refs.q.', poseTime, 'linear');
poseError = referencePose-absolutePose;
referenceVelocity = interp1(setup.refs.t(:), setup.refs.qd.', velocityTime, 'linear');
velocityError = referenceVelocity-generalizedVelocity;

sampleCount = numel(diagnosticTime);
poseAtDiagnostic = interp1(poseTime, absolutePose, diagnosticTime, 'linear');
velocityAtDiagnostic = interp1(velocityTime, generalizedVelocity, diagnosticTime, 'linear');
commandAtDiagnostic = interp1(baseline.forceTime, baseline.controlForce, ...
    diagnosticTime, 'previous', 'extrap');
legLength = zeros(sampleCount, 6);
legSpeed = zeros(sampleCount, 6);
legAcceleration = zeros(sampleCount, 6);
sigmaActual = zeros(sampleCount, 1);
collisionDistance = zeros(sampleCount, 3);
collisionControllerThreshold = zeros(sampleCount, 3);
collisionRequiredThreshold = zeros(sampleCount, 3);
roofThresholdRateDifference = zeros(sampleCount, 1);
requiredConfig = setup.strictConfig;
requiredConfig.collision.roofStage1Distance = ...
    setup.scene.collision.stage1ConstraintDistance;
requiredConfig.collision.roofFinalDistance = setup.scene.collision.finalGap;
requiredConfig.collision.sideDistance = setup.scene.collision.safeDistance;
for index = 1:sampleCount
    q = poseAtDiagnostic(index, :).';
    qd = velocityAtDiagnostic(index, :).';
    kinematics = sgpIK(q, setup.model);
    jacobian = sgpJacobian(q, setup.model);
    evaluation = evaluateStrictClfCbfAdModel( ...
        q, qd, setup.model, setup.strictScene, setup.strictConfig);
    legLength(index, :) = kinematics.L.';
    legSpeed(index, :) = (jacobian.Jq*qd).';
    legAcceleration(index, :) = (evaluation.legAccelerationDrift+ ...
        evaluation.legAccelerationMap*commandAtDiagnostic(index, :).').';
    sigmaActual(index) = jacobian.sigmaMin;
    distance = evaluateCylinderBoxDistanceNumeric(q, setup.plantScene);
    collisionDistance(index, :) = distance.distances;
    roof = evaluateStrictRoofThreshold(diagnosticTime(index), ...
        setup.strictScene, setup.strictConfig);
    requiredRoof = evaluateStrictRoofThreshold(diagnosticTime(index), ...
        setup.scene, requiredConfig);
    collisionControllerThreshold(index, :) = [roof.value, ...
        setup.strictScene.collision.safeDistance, ...
        setup.strictScene.collision.safeDistance];
    collisionRequiredThreshold(index, :) = [requiredRoof.value, ...
        setup.scene.collision.safeDistance, setup.scene.collision.safeDistance];
    roofThresholdRateDifference(index) = roof.rate-requiredRoof.rate;
end

dt = setup.strictConfig.dt;
forceDifference = diff(commandAtDiagnostic, 1, 1)/dt;
forceRatePeak = max(abs(forceDifference), [], 1);
solveTimes = decoded.solveTime(isfinite(decoded.solveTime));
if isempty(solveTimes)
    solveStatistics = [nan, nan, nan];
else
    solveStatistics = [mean(solveTimes), percentile(solveTimes, 95), max(solveTimes)];
end
stepTimes = decoded.controllerStepTime(isfinite(decoded.controllerStepTime));
if isempty(stepTimes)
    stepStatistics = [nan, nan, nan];
else
    stepStatistics = [mean(stepTimes), percentile(stepTimes, 95), max(stepTimes)];
end
controllerGeometryMargin = collisionDistance-collisionControllerThreshold;
geometryMargin = collisionDistance-collisionRequiredThreshold;
thresholdBuffer = collisionControllerThreshold-collisionRequiredThreshold;
requiredStateMargins = decoded.stateMargins;
requiredPsi1 = decoded.psi1;
collisionStateColumns = find(ismember(decoded.stateNames, ...
    ["collision_roof", "collision_left", "collision_right"]));
collisionPsiColumns = find(ismember(decoded.psi1Names, ...
    ["collision_roof", "collision_left", "collision_right"]));
requiredStateMargins(:, collisionStateColumns) = ...
    requiredStateMargins(:, collisionStateColumns)+thresholdBuffer;
psiCorrection = setup.strictConfig.cbf.collisionAlpha1*thresholdBuffer;
psiCorrection(:, 1) = psiCorrection(:, 1)+roofThresholdRateDifference;
requiredPsi1(:, collisionPsiColumns) = ...
    requiredPsi1(:, collisionPsiColumns)+psiCorrection;
decoded.requiredStateMargins = requiredStateMargins;
decoded.requiredPsi1 = requiredPsi1;
sigmaLower = sqrt(max(decoded.sigmaLowerSquared, 0));
tolerance = 10*setup.strictConfig.solver.constraintTolerance;

qpAcceptance = struct();
qpAcceptance.allStepsFeasible = all(decoded.feasible > 0.5);
qpAcceptance.noFallback = ~any(decoded.usedFallback > 0.5);
qpAcceptance.noSolverError = ~any(decoded.solverError > 0.5);
qpAcceptance.exactDerivativeBackend = ~any(decoded.usesFiniteDifferences > 0.5);
qpAcceptance.initialStateMarginPassed = ...
    setup.strictRuntime.initialConditionAudit.minimumStateMargin >= -tolerance;
qpAcceptance.initialPsi1Passed = ...
    setup.strictRuntime.initialConditionAudit.minimumPsi1 >= -tolerance;
qpAcceptance.hardResidualPassed = min(decoded.minimumCbfResidual) >= -tolerance;
qpAcceptance.stateMarginPassed = min(requiredStateMargins, [], 'all') >= -tolerance;
qpAcceptance.psi1Passed = min(requiredPsi1, [], 'all') >= -tolerance;
qpAcceptance.lengthPassed = all(legLength >= setup.strictConfig.lengthMin.'-tolerance, 'all') && ...
    all(legLength <= setup.strictConfig.lengthMax.'+tolerance, 'all');
qpAcceptance.speedPassed = all(abs(legSpeed) <= ...
    setup.strictConfig.legSpeedLimit.'+tolerance, 'all');
qpAcceptance.modelAccelerationPassed = all(abs(legAcceleration) <= ...
    setup.strictConfig.legAccelerationLimit.'+1e-6, 'all');
qpAcceptance.measuredAccelerationPassed = ...
    baseline.metrics.maxAbsLegAcceleration <= ...
    max(setup.strictConfig.legAccelerationLimit)+1e-2;
qpAcceptance.forcePassed = all(commandAtDiagnostic >= ...
    setup.strictConfig.forceMin.'-tolerance, 'all') && ...
    all(commandAtDiagnostic <= setup.strictConfig.forceMax.'+tolerance, 'all');
qpAcceptance.forceRatePassed = isempty(forceDifference) || all(forceRatePeak <= ...
    setup.strictConfig.forceRateLimit.'+1e-6);
qpAcceptance.singularityPassed = min(sigmaActual) >= setup.strictConfig.sigmaSafe-tolerance;
qpAcceptance.singularityLowerBoundValid = all(sigmaLower <= sigmaActual+1e-9);
qpAcceptance.collisionPassed = min(geometryMargin, [], 'all') >= -1e-7;
qpAcceptance.controllerStepTime95Passed = stepStatistics(2) <= dt;
qpAcceptance.trackingPassed = baseline.acceptance.trackingPassed;

% The same numerical safety/performance checks are required for both paths.
% Nonideal execution may pass these checks, but it remains engineering
% evidence rather than an unconditional forward-invariance proof.
qpPassed = all(structfun(@(value) logical(value), qpAcceptance));

report = struct();
report.validationClass = setup.validationClass;
report.plantPerturbation = setup.plantPerturbation;
report.baseline = baseline;
report.diagnosticTime = diagnosticTime;
report.poseTime = poseTime;
report.velocityTime = velocityTime;
report.nominalTime = nominalTime;
report.absolutePose = absolutePose;
report.generalizedVelocity = generalizedVelocity;
report.nominalForce = nominalForce;
report.poseError = poseError;
report.velocityError = velocityError;
report.diagnostics = decoded;
report.legLength = legLength;
report.legSpeed = legSpeed;
report.modelLegAcceleration = legAcceleration;
report.sigmaActual = sigmaActual;
report.sigmaLower = sigmaLower;
report.collisionDistance = collisionDistance;
report.collisionControllerThreshold = collisionControllerThreshold;
report.collisionRequiredThreshold = collisionRequiredThreshold;
report.collisionThresholdBuffer = thresholdBuffer;
report.collisionControllerMargin = controllerGeometryMargin;
report.collisionMargin = geometryMargin;
report.metrics = struct( ...
    'translationRms', sqrt(mean(poseError(:, 1:3).^2, 'all')), ...
    'translationPeak', max(abs(poseError(:, 1:3)), [], 'all'), ...
    'rotationRms', sqrt(mean(poseError(:, 4:6).^2, 'all')), ...
    'rotationPeak', max(abs(poseError(:, 4:6)), [], 'all'), ...
    'velocityRms', sqrt(mean(velocityError.^2, 'all')), ...
    'maxAbsCommandForce', max(abs(commandAtDiagnostic), [], 'all'), ...
    'maxAbsForceRate', max(forceRatePeak, [], 'all'), ...
    'controlEffortIntegral', trapz(diagnosticTime, sum(commandAtDiagnostic.^2, 2)), ...
    'maxAbsLegSpeed', max(abs(legSpeed), [], 'all'), ...
    'maxAbsModelLegAcceleration', max(abs(legAcceleration), [], 'all'), ...
    'maxAbsMeasuredLegAcceleration', baseline.metrics.maxAbsLegAcceleration, ...
    'actualForceTrackingRms', baseline.metrics.targetActualForceRms, ...
    'actualForceTrackingPeak', baseline.metrics.targetActualForcePeak, ...
    'minimumSigmaActual', min(sigmaActual), ...
    'minimumSigmaLower', min(sigmaLower), ...
    'minimumCollisionMargin', min(geometryMargin, [], 'all'), ...
    'minimumCbfResidual', min(decoded.minimumCbfResidual), ...
    'minimumStateMargin', min(requiredStateMargins, [], 'all'), ...
    'minimumPsi1', min(requiredPsi1, [], 'all'), ...
    'minimumTightenedStateMargin', min(decoded.minimumStateMargin), ...
    'minimumTightenedPsi1', min(decoded.minimumPsi1), ...
    'maximumClfSlack', max(decoded.clfSlack), ...
    'fallbackCount', nnz(decoded.usedFallback > 0.5), ...
    'infeasibleCount', nnz(decoded.feasible <= 0.5), ...
    'solverErrorCount', nnz(decoded.solverError > 0.5), ...
    'coldInitializationWallTime', ...
        setup.strictRuntime.initialConditionAudit.coldInitializationWallTime, ...
    'coldQpSolveTime', setup.strictRuntime.initialConditionAudit.coldQpSolveTime, ...
    'coldControllerStepTime', ...
        setup.strictRuntime.initialConditionAudit.coldControllerStepTime, ...
    'simulinkInitializationTime', max(decoded.simulinkInitializationTime), ...
    'solveTimeMean', solveStatistics(1), ...
    'solveTime95', solveStatistics(2), ...
    'solveTimeMax', solveStatistics(3), ...
    'controllerStepTimeMean', stepStatistics(1), ...
    'controllerStepTime95', stepStatistics(2), ...
    'controllerStepTimeMax', stepStatistics(3));
report.acceptance = qpAcceptance;
report.strictGuaranteeApplicable = setup.actuatorMode == "ideal-force";
report.forceSignCheck = makeForceSignCheck(baseline);
report.passed = qpPassed;
end

function aligned = alignMeasuredForceForLegacyEvaluator(simout)
% The legacy evaluator assumes commanded and measured force share a grid.
% A discrete QP intentionally breaks that assumption, so align a local
% copy without altering the raw evidence carried by simout.
aligned = simout;
if ~isfield(simout, 'y') || ~isfield(simout.y, 'Taum') || ~isfield(simout, 'u')
    return;
end
[commandTime, ~] = readSignal(simout.u);
[actualTime, actualForce] = readSignal(simout.y.Taum);
if numel(actualTime) == numel(commandTime) && ...
        max(abs(actualTime-commandTime)) <= 1e-12
    return;
end
actualAligned = interp1(actualTime, actualForce, commandTime, 'linear', 'extrap');
aligned.y.Taum = timeseries(actualAligned, commandTime);
end

function decoded = decodeDiagnostics(matrix, layout)
h = layout.header;
headerNames = fieldnames(h);
decoded = struct();
for index = 1:numel(headerNames)
    name = headerNames{index};
    decoded.(name) = matrix(:, h.(name));
end
decoded.stateNames = layout.stateNames;
decoded.stateMargins = matrix(:, layout.stateMargins);
decoded.psi1Names = layout.psi1Names;
decoded.psi1 = matrix(:, layout.psi1);
decoded.cbfConstraintNames = layout.cbfConstraintNames;
decoded.cbfResidual = matrix(:, layout.cbfResidual);
decoded.activeConstraintNames = layout.activeConstraintNames;
decoded.activeMask = matrix(:, layout.activeMask) > 0.5;
end

function check = makeForceSignCheck(baseline)
check = struct('available', false, 'sameSignRms', nan, ...
    'oppositeSignRms', nan, 'passed', false);
if isempty(baseline.actualForce) || ~any(isfinite(baseline.actualForce), 'all')
    return;
end
actual = interp1(baseline.actualForceTime, baseline.actualForce, ...
    baseline.forceTime, 'linear', 'extrap');
finite = isfinite(actual) & isfinite(baseline.controlForce);
if ~any(finite, 'all')
    return;
end
same = baseline.controlForce(finite)-actual(finite);
opposite = baseline.controlForce(finite)+actual(finite);
check.available = true;
check.sameSignRms = sqrt(mean(same.^2));
check.oppositeSignRms = sqrt(mean(opposite.^2));
check.passed = check.sameSignRms <= check.oppositeSignRms;
end

function value = percentile(samples, percentage)
samples = sort(samples(:));
position = 1+(numel(samples)-1)*percentage/100;
lower = floor(position);
upper = ceil(position);
if lower == upper
    value = samples(lower);
else
    value = samples(lower)+(position-lower)*(samples(upper)-samples(lower));
end
end

function [time, data] = readSignal(value)
if ~isa(value, 'timeseries')
    error('evaluateSimscapeStrictClfCbfQp:InvalidSignal', ...
        'Expected a logged timeseries.');
end
time = value.Time(:);
data = squeeze(value.Data);
if size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
if size(data, 1) ~= numel(time)
    error('evaluateSimscapeStrictClfCbfQp:InvalidSignalSize', ...
        'Cannot align signal size %s with %d time samples.', ...
        mat2str(size(data)), numel(time));
end
[time, uniqueIndex] = unique(time, 'stable');
data = data(uniqueIndex, :);
end
