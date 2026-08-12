function run = simulateLqiScheduleScenario(reference, schedule, poseConfig, scenario, options)
% simulateLqiScheduleScenario - Integrate one exact fixed/scheduled LQI run.
arguments
    reference struct
    schedule struct
    poseConfig struct
    scenario struct
    options.integrationSubsteps (1, 1) double {mustBeInteger, mustBePositive} = 2
    options.initialPoseOffset double = [0.0004; -0.0003; 0.0002; 0.0002; -0.00015; 0.0001]
    options.initialVelocityOffset double = zeros(6, 1)
end

time = reference.time(:).';
sampleCount = numel(time);
if schedule.sampleCount ~= sampleCount
    error('simulateLqiScheduleScenario:ScheduleLengthMismatch', ...
        'The controller schedule and reference must have the same length.');
end
dt = median(diff(time));
substepCount = options.integrationSubsteps;
denseCount = (sampleCount-1)*substepCount+1;
denseTime = linspace(time(1), time(end), denseCount);

controllerState = zeros(size(schedule.samples(1).controllerA, 1), 1);
antiWindup = zeros(6, 1);
state = zeros(12, sampleCount);
state(:, 1) = [reference.q(:, 1)+options.initialPoseOffset(:); ...
    reference.qd(:, 1)+options.initialVelocityOffset(:)];
rawFeedbackForce = zeros(6, sampleCount);
feedbackForce = zeros(6, sampleCount);
nominalForce = zeros(6, sampleCount);
commandForce = zeros(6, sampleCount);
appliedForce = zeros(6, sampleCount);

denseState = zeros(12, denseCount);
denseAppliedForce = zeros(6, denseCount);
denseState(:, 1) = state(:, 1);
forceState = reference.computedForce(:, 1);
terminatedEarly = false;
termination = struct('time', NaN, 'reason', "", 'completedControlSamples', 0, ...
    'requestedControlSamples', sampleCount);
lastDenseIndex = 1;

for sampleIndex = 1:sampleCount
    item = schedule.samples(schedule.referenceIndex(sampleIndex));
    q = state(1:6, sampleIndex);
    [controllerState, antiWindup, rawFeedback, saturatedFeedback] = ...
        stepDiscreteLqiController(item, controllerState, ...
        reference.q(:, sampleIndex)-q, antiWindup, ...
        poseConfig.lqiFeedbackForceLimit);

    rawFeedbackForce(:, sampleIndex) = rawFeedback;
    feedbackForce(:, sampleIndex) = saturatedFeedback;
    nominalForce(:, sampleIndex) = reference.computedForce(:, sampleIndex) + ...
        saturatedFeedback;
    command = min(max(nominalForce(:, sampleIndex), ...
        scenario.plantModel.actuator.forceMin), scenario.plantModel.actuator.forceMax);
    commandForce(:, sampleIndex) = command;
    if scenario.actuatorLag > 0
        appliedForce(:, sampleIndex) = forceState;
    else
        forceState = command;
        appliedForce(:, sampleIndex) = command;
    end

    denseStart = (sampleIndex-1)*substepCount+1;
    denseState(:, denseStart) = state(:, sampleIndex);
    denseAppliedForce(:, denseStart) = appliedForce(:, sampleIndex);
    lastDenseIndex = denseStart;
    termination.completedControlSamples = sampleIndex;
    if sampleIndex == sampleCount
        break;
    end
    try
        [nextState, nextForce, internalState, internalForce] = integrateInterval( ...
            state(:, sampleIndex), forceState, command, dt, substepCount, ...
            scenario.plantModel, scenario.actuatorLag);
    catch exception
        terminatedEarly = true;
        termination.time = time(sampleIndex);
        termination.reason = string(exception.identifier);
        termination.completedControlSamples = sampleIndex;
        break;
    end
    denseIndices = denseStart+(1:substepCount);
    denseState(:, denseIndices) = internalState;
    denseAppliedForce(:, denseIndices) = internalForce;
    lastDenseIndex = denseIndices(end);
    state(:, sampleIndex+1) = nextState;
    forceState = nextForce;
end

lastSample = termination.completedControlSamples;
time = time(1:lastSample);
state = state(:, 1:lastSample);
rawFeedbackForce = rawFeedbackForce(:, 1:lastSample);
feedbackForce = feedbackForce(:, 1:lastSample);
nominalForce = nominalForce(:, 1:lastSample);
commandForce = commandForce(:, 1:lastSample);
appliedForce = appliedForce(:, 1:lastSample);
denseTime = denseTime(1:lastDenseIndex);
denseState = denseState(:, 1:lastDenseIndex);
denseAppliedForce = denseAppliedForce(:, 1:lastDenseIndex);
termination.terminatedEarly = terminatedEarly;

control = struct();
control.time = time;
control.state = state;
control.poseError = state(1:6, :)-reference.q(:, 1:lastSample);
control.velocityError = state(7:12, :)-reference.qd(:, 1:lastSample);
control.rawFeedbackForce = rawFeedbackForce;
control.feedbackForce = feedbackForce;
control.nominalForce = nominalForce;
control.commandForce = commandForce;
control.appliedForce = appliedForce;
dense = buildDenseTelemetry(denseTime, denseState, denseAppliedForce, scenario.plantModel);

run = struct();
run.scenario = rmfield(scenario, 'plantModel');
run.control = control;
run.dense = dense;
run.termination = termination;
run.metrics = calculateMetrics(control, dense, scenario.plantModel, termination);
end

function [nextState, nextForce, internalState, internalForce] = integrateInterval( ...
        initialState, initialForce, commandForce, duration, substepCount, model, actuatorLag)
stepSize = duration/substepCount;
internalState = zeros(12, substepCount);
internalForce = zeros(6, substepCount);
if actuatorLag > 0
    value = [initialState; initialForce];
    derivative = @(sample) [rigidDerivative(sample(1:12), sample(13:18), model); ...
        (commandForce-sample(13:18))/actuatorLag];
    for index = 1:substepCount
        value = rk4(value, stepSize, derivative);
        internalState(:, index) = value(1:12);
        internalForce(:, index) = value(13:18);
    end
    nextState = value(1:12);
    nextForce = value(13:18);
else
    value = initialState;
    derivative = @(sample) rigidDerivative(sample, commandForce, model);
    for index = 1:substepCount
        value = rk4(value, stepSize, derivative);
        internalState(:, index) = value;
        internalForce(:, index) = commandForce;
    end
    nextState = value;
    nextForce = commandForce;
end
end

function derivative = rigidDerivative(state, force, model)
derivative = stateDynamicsCompositeRigidBody(state, force, model);
end

function next = rk4(value, stepSize, derivative)
k1 = derivative(value);
k2 = derivative(value+0.5*stepSize*k1);
k3 = derivative(value+0.5*stepSize*k2);
k4 = derivative(value+stepSize*k3);
next = value+stepSize*(k1+2*k2+2*k3+k4)/6;
end

function dense = buildDenseTelemetry(time, state, appliedForce, model)
sampleCount = numel(time);
legLength = zeros(6, sampleCount);
legSpeed = zeros(6, sampleCount);
legAcceleration = zeros(6, sampleCount);
sigmaMin = zeros(1, sampleCount);
for index = 1:sampleCount
    [~, auxiliary] = stateDynamicsCompositeRigidBody( ...
        state(:, index), appliedForce(:, index), model);
    q = state(1:6, index);
    qd = state(7:12, index);
    leg = computeLegKinematics(q, qd, auxiliary.qdd, model);
    jacobian = sgpJacobian(q, model);
    legLength(:, index) = leg.L;
    legSpeed(:, index) = leg.Ld;
    legAcceleration(:, index) = leg.Ldd;
    sigmaMin(index) = jacobian.sigmaMin;
end
if sampleCount > 1
    forceRate = [zeros(6, 1), diff(appliedForce, 1, 2) ./ diff(time)];
else
    forceRate = zeros(6, 1);
end
dense = struct('time', time, 'state', state, 'appliedForce', appliedForce, ...
    'appliedForceRate', forceRate, 'legLength', legLength, 'legSpeed', legSpeed, ...
    'legAcceleration', legAcceleration, 'sigmaMin', sigmaMin);
end

function metrics = calculateMetrics(control, dense, model, termination)
translationError = vecnorm(control.poseError(1:3, :), 2, 1);
rotationError = vecnorm(control.poseError(4:6, :), 2, 1);
lowerLengthMargin = dense.legLength-model.lmin(:);
upperLengthMargin = model.lmax(:)-dense.legLength;
speedMargin = model.actuator.ldotMax(:)-abs(dense.legSpeed);
accelerationMargin = model.actuator.lddotMax(:)-abs(dense.legAcceleration);
forceLowerMargin = control.appliedForce-model.actuator.forceMin(:);
forceUpperMargin = model.actuator.forceMax(:)-control.appliedForce;
commandRate = [zeros(6, 1), diff(control.commandForce, 1, 2) ./ diff(control.time)];

metrics = struct();
metrics.translationRmse = sqrt(mean(translationError.^2));
metrics.rotationRmse = sqrt(mean(rotationError.^2));
metrics.translationPeak = max(translationError);
metrics.rotationPeak = max(rotationError);
metrics.peakLqiFeedbackForce = max(abs(control.feedbackForce), [], 'all');
metrics.peakRawLqiFeedbackForce = max(abs(control.rawFeedbackForce), [], 'all');
metrics.peakCommandForce = max(abs(control.commandForce), [], 'all');
metrics.peakAppliedForce = max(abs(control.appliedForce), [], 'all');
metrics.peakCommandForceRate = max(abs(commandRate), [], 'all');
metrics.peakAppliedForceRate = max(abs(dense.appliedForceRate), [], 'all');
metrics.controlEnergy = trapz(dense.time, sum(dense.appliedForce.^2, 1));
metrics.minSigma = min(dense.sigmaMin);
metrics.lengthViolationCount = sum(any(lowerLengthMargin < -1e-6 | ...
    upperLengthMargin < -1e-6, 1));
metrics.speedViolationCount = sum(any(speedMargin < -1e-6, 1));
metrics.accelerationViolationCount = sum(any(accelerationMargin < -1e-6, 1));
metrics.forceViolationCount = sum(any(forceLowerMargin < -1e-7 | ...
    forceUpperMargin < -1e-7, 1));
metrics.fullTrajectoryCompleted = ~termination.terminatedEarly;
metrics.terminatedEarly = termination.terminatedEarly;
end
