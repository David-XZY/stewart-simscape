function run = evaluateSimscapeDisturbanceRun(simout, setup, reference, ...
        experimentCase, controller, strictConfig, scene, config)
% evaluateSimscapeDisturbanceRun - Convert one actual Simscape run to common metrics.
arguments
    simout struct
    setup struct
    reference struct
    experimentCase struct
    controller struct
    strictConfig struct
    scene struct
    config struct
end

if controller.id == "strict_qp"
    strictReport = evaluateSimscapeStrictClfCbfQp(simout, setup);
    baseline = strictReport.baseline;
    diagnostic = strictReport.diagnostics;
    controllerTime = diagnostic.controllerStepTime;
    qpTime = diagnostic.solveTime;
    feasible = diagnostic.feasible > 0.5;
    fallback = diagnostic.usedFallback > 0.5;
    minimumCbf = diagnostic.minimumCbfResidual;
else
    baselineInput = alignMeasuredForce(simout);
    baseline = evaluateSimscapePoseForceControl(baselineInput, setup.refs, ...
        setup.model, setup.design, setup.config);
    controllerTime = zeros(numel(baseline.forceTime), 1);
    qpTime = nan(size(controllerTime));
    feasible = true(size(controllerTime));
    fallback = false(size(controllerTime));
    minimumCbf = nan(size(controllerTime));
    if isfield(simout, 'candidateDiagnostic')
        [~, matrix] = readSignal(simout.candidateDiagnostic);
        layout = setup.candidateRuntime.diagnosticLayout;
        controllerTime = matrix(:, layout.controllerStepTime);
        qpTime = matrix(:, layout.qpSolveTime);
        feasible = matrix(:, layout.feasible) > 0.5;
        fallback = matrix(:, layout.usedFallback) > 0.5;
        minimumCbf = matrix(:, layout.minimumCbfResidual);
    end
end

poseTime = baseline.poseTime(:);
poseError = baseline.poseError;
equivalent = sqrt(sum(poseError(:, 1:3).^2, 2)+ ...
    (0.5*vecnorm(poseError(:, 4:6), 2, 2)).^2);
absolutePose = baseline.actualRelativePose+setup.refs.q0(:).';
sampleCount = numel(poseTime);
sigma = zeros(sampleCount, 1);
collisionDistance = zeros(sampleCount, 3);
for index = 1:sampleCount
    q = absolutePose(index, :).';
    sigma(index) = sgpJacobian(q, setup.model).sigmaMin;
    distance = evaluateCylinderBoxDistanceNumeric(q, scene);
    collisionDistance(index, :) = distance.distances;
end
threshold = [scene.collision.finalGap, scene.collision.safeDistance, ...
    scene.collision.safeDistance];
collisionMargin = collisionDistance-threshold;

force = baseline.controlForce;
forceTime = baseline.forceTime(:);
forceRate = [zeros(1, 6); diff(force, 1, 1)./diff(forceTime)];
legLength = baseline.actualAbsoluteLength;
legSpeed = baseline.legSpeed;
legAcceleration = baseline.legAcceleration;
lowerLength = legLength-setup.model.lmin(:).';
upperLength = setup.model.lmax(:).'-legLength;
speedMargin = setup.model.actuator.ldotMax(:).'-abs(legSpeed);
accelerationMargin = setup.model.actuator.lddotMax(:).'-abs(legAcceleration);
forceMargin = [force-setup.model.actuator.forceMin(:).'; ...
    setup.model.actuator.forceMax(:).'-force];
forceRateMargin = strictConfig.forceRateLimit(:).'-abs(forceRate);

metrics = struct();
metrics.equivalentPoseRms = sqrt(mean(equivalent.^2));
metrics.equivalentPosePeak = max(equivalent);
metrics.translationRms = sqrt(mean(poseError(:, 1:3).^2, 1));
metrics.translationPeak = max(abs(poseError(:, 1:3)), [], 1);
metrics.rotationRms = sqrt(mean(poseError(:, 4:6).^2, 1));
metrics.rotationPeak = max(abs(poseError(:, 4:6)), [], 1);
metrics.recoveryTime = recoveryTime(poseTime, equivalent, experimentCase, config);
metrics.minLegLength = min(legLength, [], 'all');
metrics.maxLegLength = max(legLength, [], 'all');
metrics.maxLegSpeed = max(abs(legSpeed), [], 'all');
metrics.maxLegAcceleration = max(abs(legAcceleration), [], 'all');
metrics.peakForce = max(abs(force), [], 'all');
metrics.peakForceRate = max(abs(forceRate), [], 'all');
metrics.controlEnergy = trapz(forceTime, sum(force.^2, 2));
metrics.controllerTimeP95 = percentile(controllerTime(isfinite(controllerTime)), 95);
metrics.qpTimeP95 = percentile(qpTime(isfinite(qpTime)), 95);
if controller.useDob
    metrics.dobTimeP95 = metrics.controllerTimeP95;
else
    metrics.dobTimeP95 = NaN;
end
metrics.infeasibleCount = sum(~feasible);
metrics.fallbackCount = sum(fallback);
metrics.minSigma = min(sigma);
metrics.minCollisionMargin = min(collisionMargin, [], 'all');
metrics.minCbfResidual = finiteMinimum(minimumCbf);
metrics.lengthViolationCount = sum(any(lowerLength < -config.stateConstraintTolerance | ...
    upperLength < -config.stateConstraintTolerance, 2));
metrics.speedViolationCount = sum(any(speedMargin < -config.stateConstraintTolerance, 2));
metrics.accelerationViolationCount = sum(any( ...
    accelerationMargin < -config.stateConstraintTolerance, 2));
metrics.forceViolationCount = sum(any(forceMargin < -config.hardConstraintTolerance, 2));
metrics.forceRateViolationCount = sum(any( ...
    forceRateMargin < -config.hardConstraintTolerance, 2));
metrics.collisionViolationCount = sum(any( ...
    collisionMargin < -config.stateConstraintTolerance, 2));
metrics.singularityViolationCount = sum(sigma < ...
    strictConfig.sigmaSafe-config.stateConstraintTolerance);
metrics.nonfiniteCount = sum(~isfinite([poseError(:); force(:); legLength(:)]));
metrics.fullTrajectoryCompleted = abs(poseTime(end)-reference.time(end)) <= config.sampleTime;
metrics.hardConstraintsPassed = metrics.lengthViolationCount == 0 && ...
    metrics.speedViolationCount == 0 && metrics.accelerationViolationCount == 0 && ...
    metrics.forceViolationCount == 0 && metrics.forceRateViolationCount == 0 && ...
    metrics.collisionViolationCount == 0 && ...
    metrics.singularityViolationCount == 0;
metrics.eligible = metrics.hardConstraintsPassed && metrics.fullTrajectoryCompleted && ...
    metrics.infeasibleCount == 0 && metrics.fallbackCount == 0 && ...
    metrics.nonfiniteCount == 0 && metrics.controllerTimeP95 <= config.onlineP95Limit;

run = struct();
run.controller = rmfield(controller, {'schedule', 'dobConfig'});
run.experimentCase = experimentCase;
run.referencePolicy = reference.feedforwardPolicy;
run.control = struct('time', poseTime.', 'poseError', poseError.', ...
    'state', absolutePose.', 'commandForce', force.', 'appliedForce', force.');
run.diagnostics = struct('controllerStepTime', controllerTime, ...
    'qpSolveTime', qpTime, 'qpFeasible', feasible, ...
    'qpFallback', fallback, 'minimumCbfResidual', minimumCbf);
run.dense = struct('time', poseTime.', 'legLength', legLength.', ...
    'legSpeed', legSpeed.', 'legAcceleration', legAcceleration.', ...
    'sigmaMin', sigma.', 'collisionDistance', collisionDistance.');
run.termination = struct('terminatedEarly', ~metrics.fullTrajectoryCompleted, ...
    'time', poseTime(end), 'reason', "", ...
    'completedControlSamples', numel(poseTime), ...
    'requestedControlSamples', numel(reference.time));
run.metrics = metrics;
end

function aligned = alignMeasuredForce(simout)
aligned = simout;
if ~isfield(simout, 'y') || ~isfield(simout.y, 'Taum') || ~isfield(simout, 'u')
    return;
end
[commandTime, ~] = readSignal(simout.u);
[actualTime, actualForce] = readSignal(simout.y.Taum);
aligned.y.Taum = timeseries(interp1(actualTime, actualForce, commandTime, ...
    'linear', 'extrap'), commandTime);
end

function recovery = recoveryTime(time, error, experimentCase, config)
if experimentCase.disturbanceType == "nominal", recovery = 0; return; end
endTime = 0;
if experimentCase.hasWrench, endTime = max(endTime, config.wrenchWindow(2)); end
if experimentCase.hasSmoothBump, endTime = max(endTime, config.bumpWindow(2)); end
if experimentCase.hasTargetNoise, endTime = max(endTime, config.noiseWindow(2)); end
pre = time < max(0.5, min(endTime-0.1, 1.0));
threshold = max(1e-7, config.recoveryBand*sqrt(mean(error(pre).^2)));
hold = max(1, ceil(config.recoveryHoldTime/median(diff(time))));
start = find(time >= endTime, 1, 'first');
recovery = Inf;
for index = start:max(start, numel(time)-hold+1)
    last = min(numel(time), index+hold-1);
    if last-index+1 == hold && all(error(index:last) <= threshold)
        recovery = time(index)-endTime;
        return;
    end
end
end

function [time, data] = readSignal(value)
time = value.Time(:);
data = squeeze(value.Data);
if size(data, 1) ~= numel(time) && size(data, 2) == numel(time), data = data.'; end
[time, uniqueIndex] = unique(time, 'stable');
data = data(uniqueIndex, :);
end

function value = percentile(data, probability)
if isempty(data), value = NaN; return; end
data = sort(data(:));
position = 1+(numel(data)-1)*probability/100;
lower = floor(position); upper = ceil(position);
if lower == upper, value = data(lower); else
    value = data(lower)+(position-lower)*(data(upper)-data(lower));
end
end

function value = finiteMinimum(data)
data = data(isfinite(data));
if isempty(data), value = NaN; else, value = min(data); end
end
