function run = simulateStrictExactModelScenario(reference, nominalModel, scene, ...
        poseConfig, discreteLqi, strictConfig, scenario, settings)
% simulateStrictExactModelScenario - Integrate one exact nonlinear closed loop.
%
% Controller updates occur on reference.time.  Between updates, the rigid
% body is integrated with fixed command force (ideal input) or together
% with a first-order force state (lag experiment).  No artificial error
% decay or replayed plant output is used.
arguments
    reference struct
    nominalModel struct
    scene struct
    poseConfig struct
    discreteLqi
    strictConfig struct
    scenario struct
    settings struct
end

time = reference.time(:).';
sampleCount = numel(time);
dt = median(diff(time));
substepCount = settings.integrationSubsteps;
denseCount = (sampleCount-1)*substepCount+1;
denseTime = linspace(time(1), time(end), denseCount);

[controllerA, controllerB, controllerC, controllerD] = ssdata(discreteLqi);
controllerState = zeros(size(controllerA, 1), 1);
antiWindup = zeros(6, 1);

state = zeros(12, sampleCount);
state(:, 1) = [reference.q(:, 1)+settings.initialPoseOffset; ...
    reference.qd(:, 1)+settings.initialVelocityOffset];
feedbackForce = zeros(6, sampleCount);
rawFeedbackForce = zeros(6, sampleCount);
nominalForce = zeros(6, sampleCount);
commandForce = zeros(6, sampleCount);
appliedForce = zeros(6, sampleCount);
plantAcceleration = zeros(6, sampleCount);

clfValue = nan(1, sampleCount);
clfSlack = nan(1, sampleCount);
clfResidual = nan(1, sampleCount);
minimumStateMargin = nan(1, sampleCount);
minimumPsi1 = nan(1, sampleCount);
minimumCommandCbfResidual = nan(1, sampleCount);
minimumRealizedCbfResidual = nan(1, sampleCount);
sigmaLowerSquared = nan(1, sampleCount);
collisionCertificate = nan(3, sampleCount);
collisionCertificateMargin = nan(3, sampleCount);
qpSolveTime = nan(1, sampleCount);
qpFeasible = true(1, sampleCount);
qpFallback = false(1, sampleCount);
qpStatus = strings(1, sampleCount);
activeConstraints = cell(1, sampleCount);
activeConstraintCount = zeros(1, sampleCount);

denseState = zeros(12, denseCount);
denseAppliedForce = zeros(6, denseCount);
denseState(:, 1) = state(:, 1);

previousCommand = reference.computedForce(:, 1);
forceState = previousCommand;
config = strictConfig;
config.dt = dt;

% Build CasADi and warm the numerical QP before timing the study loop.
config.time = time(1);
initialEvaluation = evaluateStrictClfCbfAdModel( ...
    state(1:6, 1), state(7:12, 1), nominalModel, scene, config);
[initialClf, initialCbf] = buildStrictClfCbfConstraints( ...
    state(1:6, 1), state(7:12, 1), reference.q(:, 1), ...
    reference.qd(:, 1), reference.qdd(:, 1), nominalModel, scene, ...
    config, initialEvaluation);
emptyCbf = makeEmptyCbf();
if scenario.controllerMode == "clf-only"
    for warmupIndex = 1:4
        solveStrictClfCbfQp(previousCommand, previousCommand, ...
            initialClf, emptyCbf, config);
    end
elseif scenario.controllerMode == "full"
    for warmupIndex = 1:4
        solveStrictClfCbfQp(previousCommand, previousCommand, ...
            initialClf, initialCbf, config);
    end
end

terminatedEarly = false;
terminationReason = "";
terminationTime = NaN;
lastControlSample = 0;
lastDenseSample = 1;

for sampleIndex = 1:sampleCount
    q = state(1:6, sampleIndex);
    qd = state(7:12, sampleIndex);
    trackingInput = [reference.q(:, sampleIndex)-q; antiWindup];
    rawFeedback = controllerC*controllerState + controllerD*trackingInput;
    saturatedFeedback = min(max(rawFeedback, ...
        -poseConfig.lqiFeedbackForceLimit), poseConfig.lqiFeedbackForceLimit);
    antiWindup = saturatedFeedback-rawFeedback;
    controllerState = controllerA*controllerState + ...
        controllerB*[reference.q(:, sampleIndex)-q; antiWindup];

    rawFeedbackForce(:, sampleIndex) = rawFeedback;
    feedbackForce(:, sampleIndex) = saturatedFeedback;
    nominalForce(:, sampleIndex) = reference.computedForce(:, sampleIndex) + ...
        saturatedFeedback;
    config.time = time(sampleIndex);

    if scenario.controllerMode == "full"
        [command, diagnostic] = stepStrictClfCbfQp( ...
            q, qd, reference.q(:, sampleIndex), ...
            reference.qd(:, sampleIndex), reference.qdd(:, sampleIndex), ...
            nominalForce(:, sampleIndex), previousCommand, nominalModel, ...
            scene, config);
        evaluation = diagnostic.cbf.evaluation;
        monitorCbf = diagnostic.cbf;
        clf = diagnostic.clf;
        clfSlack(sampleIndex) = diagnostic.clfSlack;
        clfResidual(sampleIndex) = diagnostic.clfResidual;
        minimumCommandCbfResidual(sampleIndex) = ...
            diagnostic.minimumCbfResidual;
        qpSolveTime(sampleIndex) = diagnostic.solveTime;
        qpFeasible(sampleIndex) = diagnostic.feasible;
        qpFallback(sampleIndex) = diagnostic.usedFallback;
        qpStatus(sampleIndex) = diagnostic.status;
        names = string(diagnostic.activeConstraints);
        if diagnostic.clfResidual <= config.solver.activeTolerance
            names(end+1) = "clf"; %#ok<AGROW>
        end
    else
        evaluation = evaluateStrictClfCbfAdModel(q, qd, ...
            nominalModel, scene, config);
        [clf, monitorCbf] = buildStrictClfCbfConstraints( ...
            q, qd, reference.q(:, sampleIndex), ...
            reference.qd(:, sampleIndex), reference.qdd(:, sampleIndex), ...
            nominalModel, scene, config, evaluation);
        if scenario.controllerMode == "clf-only"
            [command, diagnostic] = solveStrictClfCbfQp( ...
                nominalForce(:, sampleIndex), previousCommand, ...
                clf, emptyCbf, config);
            clfSlack(sampleIndex) = diagnostic.clfSlack;
            clfResidual(sampleIndex) = diagnostic.clfResidual;
            qpSolveTime(sampleIndex) = diagnostic.solveTime;
            qpFeasible(sampleIndex) = diagnostic.feasible;
            qpFallback(sampleIndex) = diagnostic.usedFallback;
            qpStatus(sampleIndex) = diagnostic.status;
            names = string(diagnostic.activeConstraints);
            if diagnostic.clfResidual <= config.solver.activeTolerance
                names(end+1) = "clf"; %#ok<AGROW>
            end
        else
            command = min(max(nominalForce(:, sampleIndex), ...
                nominalModel.actuator.forceMin), ...
                nominalModel.actuator.forceMax);
            qpStatus(sampleIndex) = "not_applicable";
            names = strings(0, 1);
        end
        minimumCommandCbfResidual(sampleIndex) = min( ...
            monitorCbf.b-monitorCbf.A*command);
    end

    commandForce(:, sampleIndex) = command;
    clfValue(sampleIndex) = clf.V;
    minimumStateMargin(sampleIndex) = monitorCbf.minimumStateMargin;
    minimumPsi1(sampleIndex) = monitorCbf.minimumPsi1;
    sigmaLowerSquared(sampleIndex) = evaluation.sigmaLowerSquared;
    collisionCertificate(:, sampleIndex) = evaluation.collisionGap;
    threshold = collisionThreshold(time(sampleIndex), scene, config);
    collisionCertificateMargin(:, sampleIndex) = ...
        evaluation.collisionGap-threshold;
    activeConstraints{sampleIndex} = names(:).';
    activeConstraintCount(sampleIndex) = numel(names);

    if scenario.actuatorLag > 0
        appliedForce(:, sampleIndex) = forceState;
    else
        forceState = command;
        appliedForce(:, sampleIndex) = command;
    end
    try
        [~, plantAux] = stateDynamicsCompositeRigidBody( ...
            state(:, sampleIndex), appliedForce(:, sampleIndex), ...
            scenario.plantModel);
    catch exception
        terminatedEarly = true;
        terminationReason = string(exception.identifier);
        terminationTime = time(sampleIndex);
        lastControlSample = max(1, sampleIndex-1);
        lastDenseSample = max(1, (sampleIndex-1)*substepCount);
        break;
    end
    plantAcceleration(:, sampleIndex) = plantAux.qdd;
    previousCommand = command;

    denseStart = (sampleIndex-1)*substepCount+1;
    denseState(:, denseStart) = state(:, sampleIndex);
    denseAppliedForce(:, denseStart) = appliedForce(:, sampleIndex);
    lastControlSample = sampleIndex;
    lastDenseSample = denseStart;
    if sampleIndex < sampleCount
        try
            [nextState, nextForce, internalState, internalForce] = ...
                integrateControlInterval(state(:, sampleIndex), forceState, ...
                command, dt, substepCount, scenario.plantModel, ...
                scenario.actuatorLag);
        catch exception
            terminatedEarly = true;
            terminationReason = string(exception.identifier);
            terminationTime = time(sampleIndex);
            break;
        end
        denseIndices = denseStart+(1:substepCount);
        denseState(:, denseIndices) = internalState;
        denseAppliedForce(:, denseIndices) = internalForce;
        lastDenseSample = denseIndices(end);
        state(:, sampleIndex+1) = nextState;
        forceState = nextForce;
    end
end

if lastControlSample < 1
    error('simulateStrictExactModelScenario:NoValidPlantSample', ...
        'The disturbed plant failed before one valid sample was obtained.');
end
time = time(1:lastControlSample);
state = state(:, 1:lastControlSample);
feedbackForce = feedbackForce(:, 1:lastControlSample);
rawFeedbackForce = rawFeedbackForce(:, 1:lastControlSample);
nominalForce = nominalForce(:, 1:lastControlSample);
commandForce = commandForce(:, 1:lastControlSample);
appliedForce = appliedForce(:, 1:lastControlSample);
plantAcceleration = plantAcceleration(:, 1:lastControlSample);
clfValue = clfValue(1:lastControlSample);
clfSlack = clfSlack(1:lastControlSample);
clfResidual = clfResidual(1:lastControlSample);
minimumStateMargin = minimumStateMargin(1:lastControlSample);
minimumPsi1 = minimumPsi1(1:lastControlSample);
minimumCommandCbfResidual = ...
    minimumCommandCbfResidual(1:lastControlSample);
minimumRealizedCbfResidual = ...
    minimumRealizedCbfResidual(1:lastControlSample);
sigmaLowerSquared = sigmaLowerSquared(1:lastControlSample);
collisionCertificate = collisionCertificate(:, 1:lastControlSample);
collisionCertificateMargin = ...
    collisionCertificateMargin(:, 1:lastControlSample);
qpSolveTime = qpSolveTime(1:lastControlSample);
qpFeasible = qpFeasible(1:lastControlSample);
qpFallback = qpFallback(1:lastControlSample);
qpStatus = qpStatus(1:lastControlSample);
activeConstraints = activeConstraints(1:lastControlSample);
activeConstraintCount = activeConstraintCount(1:lastControlSample);
denseTime = denseTime(1:lastDenseSample);
denseState = denseState(:, 1:lastDenseSample);
denseAppliedForce = denseAppliedForce(:, 1:lastDenseSample);
sampleCount = lastControlSample;

% For perturbed dynamics and actuator lag, evaluate the CBF differential
% inequality with the actual plant model and the actual applied force.  It
% is deliberately kept separate from the command-side nominal-QP residual.
if scenario.controllerMode == "full" && ...
        (~scenario.guaranteeApplicable || scenario.actuatorLag > 0)
    for sampleIndex = 1:sampleCount
        config.time = time(sampleIndex);
        q = state(1:6, sampleIndex);
        qd = state(7:12, sampleIndex);
        actualEvaluation = evaluateStrictClfCbfAdModel( ...
            q, qd, scenario.plantModel, scene, config);
        [~, actualCbf] = buildStrictClfCbfConstraints( ...
            q, qd, reference.q(:, sampleIndex), ...
            reference.qd(:, sampleIndex), reference.qdd(:, sampleIndex), ...
            scenario.plantModel, scene, config, actualEvaluation);
        minimumRealizedCbfResidual(sampleIndex) = min( ...
            actualCbf.b-actualCbf.A*appliedForce(:, sampleIndex));
    end
else
    minimumRealizedCbfResidual = minimumCommandCbfResidual;
end

denseTelemetry = evaluateDenseTelemetry(denseTime, denseState, ...
    denseAppliedForce, scenario.plantModel, nominalModel, scene, config);

diagnostics = struct();
diagnostics.clfValue = clfValue;
diagnostics.clfSlack = clfSlack;
diagnostics.clfResidual = clfResidual;
diagnostics.minimumStateMargin = minimumStateMargin;
diagnostics.minimumPsi1 = minimumPsi1;
diagnostics.minimumCommandCbfResidual = minimumCommandCbfResidual;
diagnostics.minimumRealizedCbfResidual = minimumRealizedCbfResidual;
diagnostics.sigmaLowerSquared = sigmaLowerSquared;
diagnostics.collisionCertificate = collisionCertificate;
diagnostics.collisionCertificateMargin = collisionCertificateMargin;
diagnostics.qpSolveTime = qpSolveTime;
diagnostics.qpFeasible = qpFeasible;
diagnostics.qpFallback = qpFallback;
diagnostics.qpStatus = qpStatus;
diagnostics.activeConstraints = activeConstraints;
diagnostics.activeConstraintCount = activeConstraintCount;

control = struct();
control.time = time;
control.state = state;
control.poseError = state(1:6, :)-reference.q(:, 1:sampleCount);
control.velocityError = state(7:12, :)-reference.qd(:, 1:sampleCount);
control.plantAcceleration = plantAcceleration;
control.rawFeedbackForce = rawFeedbackForce;
control.feedbackForce = feedbackForce;
control.nominalForce = nominalForce;
control.commandForce = commandForce;
control.appliedForce = appliedForce;

runScenario = rmfield(scenario, 'plantModel');
run = struct();
run.scenario = runScenario;
run.plantDynamics = scenario.plantModel.dynamics;
run.control = control;
run.dense = denseTelemetry;
run.diagnostics = diagnostics;
run.termination = struct('terminatedEarly', terminatedEarly, ...
    'time', terminationTime, 'reason', terminationReason, ...
    'completedControlSamples', lastControlSample, ...
    'requestedControlSamples', numel(reference.time));
run.metrics = calculateMetrics(run, nominalModel, strictConfig, settings);
end

function emptyCbf = makeEmptyCbf()
emptyCbf = struct();
emptyCbf.A = zeros(0, 6);
emptyCbf.b = zeros(0, 1);
emptyCbf.names = strings(0, 1);
end

function threshold = collisionThreshold(time, scene, config)
roof = evaluateStrictRoofThreshold(time, scene, config);
threshold = [roof.value; scene.collision.safeDistance; ...
    scene.collision.safeDistance];
end

function [nextState, nextForce, internalState, internalForce] = ...
        integrateControlInterval(initialState, initialForce, commandForce, ...
        intervalDuration, substepCount, plantModel, actuatorLag)
stepSize = intervalDuration/substepCount;
internalState = zeros(12, substepCount);
internalForce = zeros(6, substepCount);
if actuatorLag > 0
    extendedState = [initialState; initialForce];
    derivative = @(value) extendedDerivative( ...
        value, commandForce, plantModel, actuatorLag);
    for index = 1:substepCount
        extendedState = rk4(extendedState, stepSize, derivative);
        internalState(:, index) = extendedState(1:12);
        internalForce(:, index) = extendedState(13:18);
    end
    nextState = extendedState(1:12);
    nextForce = extendedState(13:18);
else
    value = initialState;
    derivative = @(state) rigidBodyDerivative(state, commandForce, plantModel);
    for index = 1:substepCount
        value = rk4(value, stepSize, derivative);
        internalState(:, index) = value;
        internalForce(:, index) = commandForce;
    end
    nextState = value;
    nextForce = commandForce;
end
end

function derivative = extendedDerivative(value, commandForce, plantModel, tau)
state = value(1:12);
force = value(13:18);
derivative = [rigidBodyDerivative(state, force, plantModel); ...
    (commandForce-force)/tau];
end

function derivative = rigidBodyDerivative(state, force, model)
[derivative, ~] = stateDynamicsCompositeRigidBody(state, force, model);
end

function next = rk4(value, stepSize, derivative)
k1 = derivative(value);
k2 = derivative(value+0.5*stepSize*k1);
k3 = derivative(value+0.5*stepSize*k2);
k4 = derivative(value+stepSize*k3);
next = value+stepSize*(k1+2*k2+2*k3+k4)/6;
end

function telemetry = evaluateDenseTelemetry(time, state, appliedForce, ...
        plantModel, nominalModel, scene, config)
sampleCount = numel(time);
legLength = zeros(6, sampleCount);
legSpeed = zeros(6, sampleCount);
legAcceleration = zeros(6, sampleCount);
generalizedAcceleration = zeros(6, sampleCount);
sigmaMin = zeros(1, sampleCount);
sigmaLower = zeros(1, sampleCount);
collisionDistance = zeros(3, sampleCount);
collisionMargin = zeros(3, sampleCount);
roofThreshold = zeros(1, sampleCount);
distanceSeed = zeros(6, 3);
for index = 1:sampleCount
    [~, aux] = stateDynamicsCompositeRigidBody( ...
        state(:, index), appliedForce(:, index), plantModel);
    q = state(1:6, index);
    qd = state(7:12, index);
    leg = computeLegKinematics(q, qd, aux.qdd, nominalModel);
    jacobian = sgpJacobian(q, nominalModel);
    characteristicLength = nominalModel.singularity.characteristicLength;
    normalizedJacobian = jacobian.Jv * diag([1, 1, 1, ...
        1/characteristicLength, 1/characteristicLength, ...
        1/characteristicLength]);
    gram = normalizedJacobian*normalizedJacobian.';
    [exactDistance, distanceSeed] = ...
        evaluateCylinderObbDistancesExact(q, scene, distanceSeed);
    threshold = collisionThreshold(time(index), scene, config);
    generalizedAcceleration(:, index) = aux.qdd;
    legLength(:, index) = leg.L;
    legSpeed(:, index) = leg.Ld;
    legAcceleration(:, index) = leg.Ldd;
    sigmaMin(index) = jacobian.sigmaMin;
    sigmaLower(index) = sqrt(max(0, 1/trace(gram\eye(6))));
    collisionDistance(:, index) = exactDistance;
    collisionMargin(:, index) = exactDistance-threshold;
    roofThreshold(index) = threshold(1);
end

if sampleCount > 1
    appliedForceRate = [zeros(6, 1), diff(appliedForce, 1, 2) ./ ...
        diff(time)];
else
    appliedForceRate = zeros(6, 1);
end
telemetry = struct();
telemetry.time = time;
telemetry.state = state;
telemetry.appliedForce = appliedForce;
telemetry.appliedForceRate = appliedForceRate;
telemetry.generalizedAcceleration = generalizedAcceleration;
telemetry.legLength = legLength;
telemetry.legSpeed = legSpeed;
telemetry.legAcceleration = legAcceleration;
telemetry.sigmaMin = sigmaMin;
telemetry.sigmaLower = sigmaLower;
telemetry.collisionDistance = collisionDistance;
telemetry.collisionMargin = collisionMargin;
telemetry.roofThreshold = roofThreshold;
end

function metrics = calculateMetrics(run, model, config, settings)
control = run.control;
dense = run.dense;
diagnostics = run.diagnostics;
positionNorm = vecnorm(control.poseError(1:3, :), 2, 1);
rotationNorm = vecnorm(control.poseError(4:6, :), 2, 1);

commandRate = [ ...
    (control.commandForce(:, 1)-control.nominalForce(:, 1))/config.dt, ...
    diff(control.commandForce, 1, 2)/config.dt];
% The first rate is measured against the pre-run feedforward equilibrium,
% not against the nominal command after feedback.  Replace it below with
% the exact stored initial equilibrium for an unambiguous physical metric.
initialEquilibrium = control.nominalForce(:, 1)-control.feedbackForce(:, 1);
commandRate(:, 1) = ...
    (control.commandForce(:, 1)-initialEquilibrium)/config.dt;

lowerLengthMargin = dense.legLength-model.lmin(:);
upperLengthMargin = model.lmax(:)-dense.legLength;
speedMargin = model.actuator.ldotMax(:)-abs(dense.legSpeed);
accelerationMargin = model.actuator.lddotMax(:)-abs(dense.legAcceleration);
forceLowerMargin = control.appliedForce-model.actuator.forceMin(:);
forceUpperMargin = model.actuator.forceMax(:)-control.appliedForce;
commandRateMargin = config.forceRateLimit(:)-abs(commandRate);

validQpTime = diagnostics.qpSolveTime(isfinite(diagnostics.qpSolveTime));
validSlack = diagnostics.clfSlack(isfinite(diagnostics.clfSlack));
activeNames = strings(0, 1);
for index = 1:numel(diagnostics.activeConstraints)
    activeNames = [activeNames; ...
        string(diagnostics.activeConstraints{index}(:))]; %#ok<AGROW>
end
if isempty(activeNames)
    mostActiveConstraint = "";
    mostActiveCount = 0;
else
    [uniqueNames, ~, membership] = unique(activeNames);
    counts = accumarray(membership, 1);
    [mostActiveCount, maximumIndex] = max(counts);
    mostActiveConstraint = uniqueNames(maximumIndex);
end

metrics = struct();
metrics.translationRmse = sqrt(mean(positionNorm.^2));
metrics.rotationRmse = sqrt(mean(rotationNorm.^2));
metrics.translationPeak = max(positionNorm);
metrics.rotationPeak = max(rotationNorm);
metrics.minLegLength = min(dense.legLength, [], 'all');
metrics.maxLegLength = max(dense.legLength, [], 'all');
metrics.minLegLengthMargin = min(lowerLengthMargin, [], 'all');
metrics.maxLegLengthMargin = min(upperLengthMargin, [], 'all');
metrics.maxLegSpeed = max(abs(dense.legSpeed), [], 'all');
metrics.minLegSpeedMargin = min(speedMargin, [], 'all');
metrics.maxLegAcceleration = max(abs(dense.legAcceleration), [], 'all');
metrics.minLegAccelerationMargin = min(accelerationMargin, [], 'all');
metrics.minActualCollisionDistance = min(dense.collisionDistance, [], 'all');
metrics.minActualCollisionMargin = min(dense.collisionMargin, [], 'all');
metrics.minRoofDistance = min(dense.collisionDistance(1, :));
metrics.minLeftDistance = min(dense.collisionDistance(2, :));
metrics.minRightDistance = min(dense.collisionDistance(3, :));
metrics.minRoofMargin = min(dense.collisionMargin(1, :));
metrics.minLeftMargin = min(dense.collisionMargin(2, :));
metrics.minRightMargin = min(dense.collisionMargin(3, :));
metrics.minCollisionCertificateMargin = min( ...
    diagnostics.collisionCertificateMargin, [], 'all');
metrics.minSigma = min(dense.sigmaMin);
metrics.minSigmaLower = min(dense.sigmaLower);
metrics.maxSigmaLowerBoundExcess = max( ...
    dense.sigmaLower-dense.sigmaMin);
denseControlIndex = round((control.time-dense.time(1)) ./ ...
    median(diff(dense.time)))+1;
denseControlIndex = min(max(denseControlIndex, 1), numel(dense.time));
metrics.maxCollisionCertificateExcess = max( ...
    diagnostics.collisionCertificate- ...
    dense.collisionDistance(:, denseControlIndex), [], 'all');
metrics.peakCommandForce = max(abs(control.commandForce), [], 'all');
metrics.peakAppliedForce = max(abs(control.appliedForce), [], 'all');
metrics.minAppliedForceMargin = min( ...
    [forceLowerMargin; forceUpperMargin], [], 'all');
metrics.peakCommandForceRate = max(abs(commandRate), [], 'all');
if run.scenario.actuatorLag > 0
    metrics.peakAppliedForceRate = max(abs(dense.appliedForceRate), [], 'all');
else
    % An ideal input changes at the sample boundary.  Report the standard
    % zero-order-hold command-rate metric rather than a substep-dependent
    % approximation of that discontinuity.
    metrics.peakAppliedForceRate = metrics.peakCommandForceRate;
end
metrics.minCommandForceRateMargin = min(commandRateMargin, [], 'all');
metrics.controlEnergy = trapz(dense.time, ...
    sum(dense.appliedForce.^2, 1));
metrics.maxClfSlack = finiteMaximum(validSlack);
metrics.meanClfSlack = finiteMean(validSlack);
metrics.rmsClfSlack = finiteRms(validSlack);
metrics.minStateMargin = min(diagnostics.minimumStateMargin);
metrics.minPsi1 = min(diagnostics.minimumPsi1);
metrics.minCommandCbfResidual = min( ...
    diagnostics.minimumCommandCbfResidual);
metrics.minRealizedCbfResidual = min( ...
    diagnostics.minimumRealizedCbfResidual);
metrics.commandCbfViolationCount = sum( ...
    diagnostics.minimumCommandCbfResidual < ...
    -settings.hardConstraintTolerance);
metrics.realizedCbfViolationCount = sum( ...
    diagnostics.minimumRealizedCbfResidual < ...
    -settings.hardConstraintTolerance);
metrics.infeasibleCount = sum(~diagnostics.qpFeasible);
metrics.fallbackCount = sum(diagnostics.qpFallback);
metrics.qpTimeMean = finiteMean(validQpTime);
metrics.qpTimeP95 = percentile(validQpTime, 95);
metrics.qpTimeMax = finiteMaximum(validQpTime);
metrics.activeSampleCount = sum(diagnostics.activeConstraintCount > 0);
metrics.activeSampleFraction = metrics.activeSampleCount / ...
    numel(diagnostics.activeConstraintCount);
metrics.mostActiveConstraint = mostActiveConstraint;
metrics.mostActiveConstraintCount = mostActiveCount;
metrics.lengthViolationCount = sum(any( ...
    lowerLengthMargin < -settings.stateConstraintTolerance | ...
    upperLengthMargin < -settings.stateConstraintTolerance, 1));
metrics.speedViolationCount = sum(any( ...
    speedMargin < -settings.stateConstraintTolerance, 1));
metrics.accelerationViolationCount = sum(any( ...
    accelerationMargin < -settings.stateConstraintTolerance, 1));
metrics.collisionViolationCount = sum(any( ...
    dense.collisionMargin < -settings.stateConstraintTolerance, 1));
metrics.singularityViolationCount = sum( ...
    dense.sigmaMin < config.sigmaSafe-settings.stateConstraintTolerance);
metrics.forceViolationCount = sum(any( ...
    forceLowerMargin < -settings.hardConstraintTolerance | ...
    forceUpperMargin < -settings.hardConstraintTolerance, 1));
metrics.forceRateViolationCount = sum(any( ...
    commandRateMargin < -settings.hardConstraintTolerance, 1));
metrics.initialMinimumStateMargin = diagnostics.minimumStateMargin(1);
metrics.initialMinimumPsi1 = diagnostics.minimumPsi1(1);
metrics.terminatedEarly = run.termination.terminatedEarly;
metrics.terminationTime = run.termination.time;
metrics.fullTrajectoryCompleted = ~run.termination.terminatedEarly;

metrics.trackingPassed = ...
    metrics.translationPeak <= settings.trackingTranslationPeakLimit && ...
    metrics.rotationPeak <= settings.trackingRotationPeakLimit;
metrics.physicalLimitsPassed = ...
    metrics.lengthViolationCount == 0 && ...
    metrics.speedViolationCount == 0 && ...
    metrics.accelerationViolationCount == 0 && ...
    metrics.collisionViolationCount == 0 && ...
    metrics.singularityViolationCount == 0 && ...
    metrics.forceViolationCount == 0 && ~metrics.terminatedEarly;
if run.scenario.controllerMode == "baseline"
    metrics.qpExecutionPassed = true;
else
    metrics.qpExecutionPassed = metrics.infeasibleCount == 0 && ...
        metrics.fallbackCount == 0 && ...
        metrics.qpTimeP95 <= 0.010 && ...
        metrics.forceRateViolationCount == 0;
end
metrics.engineeringPassed = metrics.trackingPassed && ...
    metrics.physicalLimitsPassed && metrics.qpExecutionPassed;
metrics.strictMatchedPassed = run.scenario.guaranteeApplicable && ...
    metrics.engineeringPassed && ...
    metrics.initialMinimumStateMargin >= -settings.stateConstraintTolerance && ...
    metrics.initialMinimumPsi1 >= -settings.stateConstraintTolerance && ...
    metrics.minStateMargin >= -settings.stateConstraintTolerance && ...
    metrics.minPsi1 >= -settings.stateConstraintTolerance && ...
    metrics.minCollisionCertificateMargin >= ...
        -settings.stateConstraintTolerance && ...
    metrics.minCommandCbfResidual >= ...
        -settings.hardConstraintTolerance && ...
    metrics.minRealizedCbfResidual >= ...
        -settings.hardConstraintTolerance;
end

function value = percentile(data, probability)
if isempty(data)
    value = NaN;
    return;
end
data = sort(data(:));
location = 1+(numel(data)-1)*probability/100;
low = floor(location);
high = ceil(location);
if low == high
    value = data(low);
else
    value = data(low)+(location-low)*(data(high)-data(low));
end
end

function value = finiteMean(data)
if isempty(data)
    value = NaN;
else
    value = mean(data);
end
end

function value = finiteMaximum(data)
if isempty(data)
    value = NaN;
else
    value = max(data);
end
end

function value = finiteRms(data)
if isempty(data)
    value = NaN;
else
    value = sqrt(mean(data.^2));
end
end
