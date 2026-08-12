function run = simulateDisturbanceControlScenario(reference, nominalModel, scene, ...
        poseConfig, strictConfig, experimentCase, controller, config)
% simulateDisturbanceControlScenario - Exact nonlinear, matched-case comparison.
arguments
    reference struct
    nominalModel struct
    scene struct
    poseConfig struct
    strictConfig struct
    experimentCase struct
    controller struct
    config struct
end

time = reference.time(:).';
sampleCount = numel(time);
dt = median(diff(time));
substeps = config.integrationSubsteps;
denseCount = (sampleCount-1)*substeps+1;
denseTime = linspace(time(1), time(end), denseCount);

state = zeros(12, sampleCount);
state(:, 1) = [reference.q(:, 1)+config.initialPoseOffset(:); ...
    reference.qd(:, 1)+config.initialVelocityOffset(:)];
denseState = zeros(12, denseCount);
denseState(:, 1) = state(:, 1);
feedbackForce = zeros(6, sampleCount);
dobForce = zeros(6, sampleCount);
nominalForce = zeros(6, sampleCount);
commandForce = zeros(6, sampleCount);
appliedForce = zeros(6, sampleCount);
plantAcceleration = zeros(6, sampleCount);
estimatedWrench = zeros(6, sampleCount);
actualWrench = evaluatePlatformWrench(time, experimentCase, config);
controllerStepTime = nan(1, sampleCount);
qpSolveTime = nan(1, sampleCount);
qpFeasible = true(1, sampleCount);
qpFallback = false(1, sampleCount);
minimumCbfResidual = nan(1, sampleCount);

lqiState = zeros(size(controller.schedule.samples(1).controllerA, 1), 1);
antiWindup = zeros(6, 1);
dobState = initializeDisturbanceObserver();
previousCommand = reference.computedForce(:, 1);
observerAppliedForce = previousCommand;
strict = strictConfig;
strict.dt = dt;

terminatedEarly = false;
terminationReason = "";
terminationTime = NaN;
lastSample = 0;
lastDense = 1;
for sampleIndex = 1:sampleCount
    q = state(1:6, sampleIndex);
    qd = state(7:12, sampleIndex);
    try
        [~, measuredAux] = dynamicsWithWrench( ...
            state(:, sampleIndex), observerAppliedForce, ...
            actualWrench(:, sampleIndex), nominalModel);
    catch exception
        terminatedEarly = true;
        terminationReason = string(exception.identifier);
        terminationTime = time(sampleIndex);
        break;
    end

    timer = tic;
    scheduleIndex = controller.schedule.referenceIndex(sampleIndex);
    scheduleSample = controller.schedule.samples(scheduleIndex);
    [lqiState, antiWindup, ~, feedback] = stepDiscreteLqiController( ...
        scheduleSample, lqiState, reference.q(:, sampleIndex)-q, ...
        antiWindup, poseConfig.lqiFeedbackForceLimit);
    compensation = zeros(6, 1);
    if controller.useDob
        [compensation, dobState, dobDiagnostic] = stepDisturbanceObserver( ...
            q, qd, measuredAux.qdd, observerAppliedForce, nominalModel, ...
            controller.dobConfig, dobState);
        estimatedWrench(:, sampleIndex) = dobDiagnostic.wrenchEstimate;
    end
    nominal = reference.computedForce(:, sampleIndex)+feedback+compensation;
    if controller.useStrictQp
        strict.time = time(sampleIndex);
        [command, qpDiagnostic] = stepStrictClfCbfQp( ...
            q, qd, reference.q(:, sampleIndex), reference.qd(:, sampleIndex), ...
            reference.qdd(:, sampleIndex), nominal, previousCommand, ...
            nominalModel, scene, strict);
        qpSolveTime(sampleIndex) = qpDiagnostic.solveTime;
        qpFeasible(sampleIndex) = qpDiagnostic.feasible;
        qpFallback(sampleIndex) = qpDiagnostic.usedFallback;
        minimumCbfResidual(sampleIndex) = qpDiagnostic.minimumCbfResidual;
    else
        command = min(max(nominal, nominalModel.actuator.forceMin), ...
            nominalModel.actuator.forceMax);
    end
    controllerStepTime(sampleIndex) = toc(timer);

    feedbackForce(:, sampleIndex) = feedback;
    dobForce(:, sampleIndex) = compensation;
    nominalForce(:, sampleIndex) = nominal;
    commandForce(:, sampleIndex) = command;
    appliedForce(:, sampleIndex) = command;
    try
        [~, plantAux] = dynamicsWithWrench(state(:, sampleIndex), command, ...
            actualWrench(:, sampleIndex), nominalModel);
    catch exception
        terminatedEarly = true;
        terminationReason = string(exception.identifier);
        terminationTime = time(sampleIndex);
        break;
    end
    plantAcceleration(:, sampleIndex) = plantAux.qdd;
    previousCommand = command;
    observerAppliedForce = command;
    denseStart = (sampleIndex-1)*substeps+1;
    denseState(:, denseStart) = state(:, sampleIndex);
    lastSample = sampleIndex;
    lastDense = denseStart;
    if sampleIndex < sampleCount
        try
            [nextState, internalState] = integrateInterval( ...
                state(:, sampleIndex), command, time(sampleIndex), dt, ...
                substeps, nominalModel, experimentCase, config);
        catch exception
            terminatedEarly = true;
            terminationReason = string(exception.identifier);
            terminationTime = time(sampleIndex);
            break;
        end
        denseIndices = denseStart+(1:substeps);
        denseState(:, denseIndices) = internalState;
        lastDense = denseIndices(end);
        state(:, sampleIndex+1) = nextState;
    end
end

if lastSample < 1
    error('simulateDisturbanceControlScenario:NoValidSample', ...
        'No valid plant sample was generated.');
end
time = time(1:lastSample);
state = state(:, 1:lastSample);
feedbackForce = feedbackForce(:, 1:lastSample);
dobForce = dobForce(:, 1:lastSample);
nominalForce = nominalForce(:, 1:lastSample);
commandForce = commandForce(:, 1:lastSample);
appliedForce = appliedForce(:, 1:lastSample);
plantAcceleration = plantAcceleration(:, 1:lastSample);
estimatedWrench = estimatedWrench(:, 1:lastSample);
actualWrench = actualWrench(:, 1:lastSample);
controllerStepTime = controllerStepTime(1:lastSample);
qpSolveTime = qpSolveTime(1:lastSample);
qpFeasible = qpFeasible(1:lastSample);
qpFallback = qpFallback(1:lastSample);
minimumCbfResidual = minimumCbfResidual(1:lastSample);
denseTime = denseTime(1:lastDense);
denseState = denseState(:, 1:lastDense);

control = struct('time', time, 'state', state, ...
    'poseError', state(1:6, :)-reference.q(:, 1:lastSample), ...
    'velocityError', state(7:12, :)-reference.qd(:, 1:lastSample), ...
    'feedbackForce', feedbackForce, 'dobForce', dobForce, ...
    'nominalForce', nominalForce, 'commandForce', commandForce, ...
    'appliedForce', appliedForce, 'plantAcceleration', plantAcceleration, ...
    'actualWrench', actualWrench, 'estimatedWrench', estimatedWrench);
diagnostics = struct('controllerStepTime', controllerStepTime, ...
    'qpSolveTime', qpSolveTime, 'qpFeasible', qpFeasible, ...
    'qpFallback', qpFallback, 'minimumCbfResidual', minimumCbfResidual);
termination = struct('terminatedEarly', terminatedEarly, ...
    'time', terminationTime, 'reason', terminationReason, ...
    'completedControlSamples', lastSample, ...
    'requestedControlSamples', sampleCount);
dense = buildTelemetry(denseTime, denseState, commandForce, time, ...
    nominalModel, scene, experimentCase, config);

run = struct();
run.controller = rmfield(controller, {'schedule', 'dobConfig'});
run.experimentCase = experimentCase;
run.referencePolicy = reference.feedforwardPolicy;
run.control = control;
run.diagnostics = diagnostics;
run.dense = dense;
run.termination = termination;
run.metrics = calculateMetrics(run, nominalModel, strict, scene, config);
end

function [xdot, aux] = dynamicsWithWrench(state, force, wrench, model)
[xdotNominal, aux] = stateDynamicsCompositeRigidBody(state, force, model);
qdd = aux.H\(aux.Wact+wrench(:)-aux.Wbias);
xdot = [state(7:12); qdd];
aux.qdd = qdd;
aux.externalWrench = wrench(:);
aux.nominalQdd = xdotNominal(7:12);
end

function [nextState, internalState] = integrateInterval(initialState, force, ...
        startTime, duration, substeps, model, experimentCase, config)
stepSize = duration/substeps;
value = initialState;
internalState = zeros(12, substeps);
for index = 1:substeps
    stepStart = startTime+(index-1)*stepSize;
    derivative = @(state, localTime) dynamicsWithWrench(state, force, ...
        evaluatePlatformWrench(localTime, experimentCase, config), model);
    k1 = derivative(value, stepStart);
    k2 = derivative(value+0.5*stepSize*k1, stepStart+0.5*stepSize);
    k3 = derivative(value+0.5*stepSize*k2, stepStart+0.5*stepSize);
    k4 = derivative(value+stepSize*k3, stepStart+stepSize);
    value = value+stepSize*(k1+2*k2+2*k3+k4)/6;
    internalState(:, index) = value;
end
nextState = value;
end

function telemetry = buildTelemetry(time, state, commandForce, controlTime, ...
        model, scene, experimentCase, config)
sampleCount = numel(time);
legLength = zeros(6, sampleCount);
legSpeed = zeros(6, sampleCount);
legAcceleration = zeros(6, sampleCount);
sigmaMin = zeros(1, sampleCount);
collisionDistance = zeros(3, sampleCount);
appliedForce = interp1(controlTime(:), commandForce.', time(:), 'previous', 'extrap').';
for index = 1:sampleCount
    wrench = evaluatePlatformWrench(time(index), experimentCase, config);
    [~, aux] = dynamicsWithWrench(state(:, index), appliedForce(:, index), wrench, model);
    q = state(1:6, index);
    qd = state(7:12, index);
    leg = computeLegKinematics(q, qd, aux.qdd, model);
    jacobian = sgpJacobian(q, model);
    legLength(:, index) = leg.L;
    legSpeed(:, index) = leg.Ld;
    legAcceleration(:, index) = leg.Ldd;
    sigmaMin(index) = jacobian.sigmaMin;
end
if config.screeningSkipCollision
    collisionDistance(:) = inf;
else
    auditIndex = unique([1:10:sampleCount, sampleCount]);
    auditDistance = zeros(3, numel(auditIndex));
    for index = 1:numel(auditIndex)
        distance = evaluateCylinderBoxDistanceNumeric( ...
            state(1:6, auditIndex(index)), scene);
        auditDistance(:, index) = distance.distances(:);
    end
    for obstacle = 1:3
        collisionDistance(obstacle, :) = interp1(time(auditIndex), ...
            auditDistance(obstacle, :), time, 'linear');
    end
end
forceRate = [zeros(6, 1), diff(appliedForce, 1, 2)./diff(time)];
telemetry = struct('time', time, 'state', state, ...
    'appliedForce', appliedForce, 'appliedForceRate', forceRate, ...
    'legLength', legLength, 'legSpeed', legSpeed, ...
    'legAcceleration', legAcceleration, 'sigmaMin', sigmaMin, ...
    'collisionDistance', collisionDistance);
end

function metrics = calculateMetrics(run, model, strictConfig, scene, config)
error = run.control.poseError;
equivalent = sqrt(sum(error(1:3, :).^2, 1)+ ...
    (0.5*vecnorm(error(4:6, :), 2, 1)).^2);
commandRate = [zeros(6, 1), diff(run.control.commandForce, 1, 2)/strictConfig.dt];
lowerLength = run.dense.legLength-model.lmin(:);
upperLength = model.lmax(:)-run.dense.legLength;
speedMargin = model.actuator.ldotMax(:)-abs(run.dense.legSpeed);
accelerationMargin = model.actuator.lddotMax(:)-abs(run.dense.legAcceleration);
forceMargin = [run.control.appliedForce-model.actuator.forceMin(:); ...
    model.actuator.forceMax(:)-run.control.appliedForce];
forceRateMargin = strictConfig.forceRateLimit(:)-abs(commandRate);
collisionThreshold = [scene.collision.finalGap; ...
    scene.collision.safeDistance; scene.collision.safeDistance];
collisionMargin = run.dense.collisionDistance-collisionThreshold;
validQp = run.diagnostics.qpSolveTime(isfinite(run.diagnostics.qpSolveTime));
validStep = run.diagnostics.controllerStepTime(isfinite(run.diagnostics.controllerStepTime));

metrics = struct();
metrics.equivalentPoseRms = sqrt(mean(equivalent.^2));
metrics.equivalentPosePeak = max(equivalent);
metrics.translationRms = sqrt(mean(error(1:3, :).^2, 2)).';
metrics.translationPeak = max(abs(error(1:3, :)), [], 2).';
metrics.rotationRms = sqrt(mean(error(4:6, :).^2, 2)).';
metrics.rotationPeak = max(abs(error(4:6, :)), [], 2).';
metrics.recoveryTime = calculateRecoveryTime(run.control.time, equivalent, ...
    run.experimentCase, config);
metrics.minLegLength = min(run.dense.legLength, [], 'all');
metrics.maxLegLength = max(run.dense.legLength, [], 'all');
metrics.maxLegSpeed = max(abs(run.dense.legSpeed), [], 'all');
metrics.maxLegAcceleration = max(abs(run.dense.legAcceleration), [], 'all');
metrics.peakForce = max(abs(run.control.appliedForce), [], 'all');
metrics.peakForceRate = max(abs(commandRate), [], 'all');
metrics.controlEnergy = trapz(run.dense.time, sum(run.dense.appliedForce.^2, 1));
metrics.controllerTimeP95 = percentile(validStep, 95);
metrics.qpTimeP95 = percentile(validQp, 95);
metrics.dobTimeP95 = metrics.controllerTimeP95;
metrics.infeasibleCount = sum(~run.diagnostics.qpFeasible);
metrics.fallbackCount = sum(run.diagnostics.qpFallback);
metrics.minSigma = min(run.dense.sigmaMin);
metrics.minCollisionMargin = min(collisionMargin, [], 'all');
metrics.minCbfResidual = finiteMinimum(run.diagnostics.minimumCbfResidual);
metrics.lengthViolationCount = sum(any(lowerLength < -config.stateConstraintTolerance | ...
    upperLength < -config.stateConstraintTolerance, 1));
metrics.speedViolationCount = sum(any(speedMargin < -config.stateConstraintTolerance, 1));
metrics.accelerationViolationCount = sum(any(accelerationMargin < -config.stateConstraintTolerance, 1));
metrics.forceViolationCount = sum(any(forceMargin < -config.hardConstraintTolerance, 1));
metrics.forceRateViolationCount = sum(any( ...
    forceRateMargin < -config.hardConstraintTolerance, 1));
metrics.collisionViolationCount = sum(any(collisionMargin < -config.stateConstraintTolerance, 1));
metrics.singularityViolationCount = sum(run.dense.sigmaMin < ...
    strictConfig.sigmaSafe-config.stateConstraintTolerance);
metrics.nonfiniteCount = sum(~isfinite([run.control.state(:); ...
    run.control.commandForce(:)]));
metrics.fullTrajectoryCompleted = ~run.termination.terminatedEarly && ...
    run.termination.completedControlSamples == run.termination.requestedControlSamples;
metrics.hardConstraintsPassed = metrics.lengthViolationCount == 0 && ...
    metrics.speedViolationCount == 0 && metrics.accelerationViolationCount == 0 && ...
    metrics.forceViolationCount == 0 && metrics.forceRateViolationCount == 0 && ...
    metrics.collisionViolationCount == 0 && ...
    metrics.singularityViolationCount == 0;
metrics.eligible = metrics.hardConstraintsPassed && ...
    metrics.fullTrajectoryCompleted && metrics.infeasibleCount == 0 && ...
    metrics.fallbackCount == 0 && metrics.nonfiniteCount == 0 && ...
    metrics.controllerTimeP95 <= config.onlineP95Limit;
end

function recovery = calculateRecoveryTime(time, equivalent, experimentCase, config)
if experimentCase.disturbanceType == "nominal"
    recovery = 0;
    return;
end
endTime = 0;
if experimentCase.hasWrench, endTime = max(endTime, config.wrenchWindow(2)); end
if experimentCase.hasSmoothBump, endTime = max(endTime, config.bumpWindow(2)); end
if experimentCase.hasTargetNoise, endTime = max(endTime, config.noiseWindow(2)); end
pre = time < max(0.5, min(endTime-0.1, 1.0));
threshold = max(1e-7, config.recoveryBand*sqrt(mean(equivalent(pre).^2)));
holdSamples = max(1, ceil(config.recoveryHoldTime/median(diff(time))));
start = find(time >= endTime, 1, 'first');
recovery = Inf;
for index = start:max(start, numel(time)-holdSamples+1)
    last = min(numel(time), index+holdSamples-1);
    if last-index+1 == holdSamples && all(equivalent(index:last) <= threshold)
        recovery = time(index)-endTime;
        return;
    end
end
end

function value = percentile(data, probability)
if isempty(data), value = NaN; return; end
data = sort(data(:));
position = 1+(numel(data)-1)*probability/100;
lower = floor(position); upper = ceil(position);
if lower == upper
    value = data(lower);
else
    value = data(lower)+(position-lower)*(data(upper)-data(lower));
end
end

function value = finiteMinimum(data)
data = data(isfinite(data));
if isempty(data), value = NaN; else, value = min(data); end
end
