function candidateDisturbanceControllerSfun(block)
% candidateDisturbanceControllerSfun - Runtime scheduled-LQI/DOB/QP bridge.
% Inputs: relative q, spatial velocity, absolute qref, qdref, qddref, Fff.
% Outputs: command, absolute q, generalized qd, nominal command, diagnostic.
setup(block);
end

function setup(block)
runtime = block.DialogPrm(1).Data;
block.NumDialogPrms = 1;
block.NumInputPorts = 6;
block.NumOutputPorts = 5;
for index = 1:6
    block.InputPort(index).Dimensions = 6;
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
end
widths = [6, 6, 6, 6, runtime.diagnosticLayout.width];
for index = 1:5
    block.OutputPort(index).Dimensions = widths(index);
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = 'Real';
end
block.SampleTimes = [runtime.sampleTime, 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @postPropagationSetup);
block.RegBlockMethod('InitializeConditions', @initializeConditions);
block.RegBlockMethod('Outputs', @outputs);
end

function postPropagationSetup(block)
runtime = block.DialogPrm(1).Data;
controllerOrder = size(runtime.schedule.samples(1).controllerA, 1);
widths = [6, 6, controllerOrder, 6, 6, 6, 6, 1, 1, 6, 6, 6, ...
    runtime.diagnosticLayout.width, 3, 3];
names = {'PreviousForce', 'ForceCommand', 'LqiState', 'AntiWindup', ...
    'DobWrench', 'DobCompensation', 'PreviousQd', 'DobInitialized', ...
    'LastTime', 'AbsolutePose', 'GeneralizedVelocity', 'NominalForce', ...
    'Diagnostic', 'GovernorPerturbation', 'GovernorRate'};
block.NumDworks = numel(widths);
for index = 1:numel(widths)
    block.Dwork(index).Name = names{index};
    block.Dwork(index).Dimensions = widths(index);
    block.Dwork(index).DatatypeID = 0;
    block.Dwork(index).Complexity = 'Real';
    block.Dwork(index).UsedAsDiscState = true;
end
end

function initializeConditions(block)
runtime = block.DialogPrm(1).Data;
initialForce = runtime.initialForce(:);
block.Dwork(1).Data = initialForce;
block.Dwork(2).Data = initialForce;
block.Dwork(3).Data = zeros(block.Dwork(3).Dimensions, 1);
block.Dwork(4).Data = zeros(6, 1);
block.Dwork(5).Data = zeros(6, 1);
block.Dwork(6).Data = zeros(6, 1);
block.Dwork(7).Data = runtime.initialQd(:);
block.Dwork(8).Data = 0;
block.Dwork(9).Data = -inf;
block.Dwork(10).Data = runtime.q0(:);
block.Dwork(11).Data = runtime.initialQd(:);
block.Dwork(12).Data = initialForce;
diagnostic = nan(runtime.diagnosticLayout.width, 1);
diagnostic(runtime.diagnosticLayout.feasible) = 1;
diagnostic(runtime.diagnosticLayout.usedFallback) = 0;
block.Dwork(13).Data = diagnostic;
block.Dwork(14).Data = zeros(3, 1);
block.Dwork(15).Data = zeros(3, 1);
end

function outputs(block)
runtime = block.DialogPrm(1).Data;
currentTime = block.CurrentTime;
lastTime = block.Dwork(9).Data;
tolerance = max(1e-12, 32*eps(max(1, abs(currentTime))));
if ~isfinite(lastTime) || abs(currentTime-lastTime) > tolerance
    relativePose = reshape(block.InputPort(1).Data, 6, 1);
    spatialVelocity = reshape(block.InputPort(2).Data, 6, 1);
    qRefRaw = reshape(block.InputPort(3).Data, 6, 1);
    qdRefRaw = reshape(block.InputPort(4).Data, 6, 1);
    qddRefRaw = reshape(block.InputPort(5).Data, 6, 1);
    feedforwardRaw = reshape(block.InputPort(6).Data, 6, 1);
    q = runtime.q0(:)+relativePose;
    rateMap = rpyRateMapZYX(q(4:6));
    if rcond(rateMap) < runtime.rpyRateRcondMin
        rpyRate = pinv(rateMap)*spatialVelocity(4:6);
    else
        rpyRate = rateMap\spatialVelocity(4:6);
    end
    qd = [spatialVelocity(1:3); rpyRate];
    timer = tic;
    sampleIndex = min(runtime.schedule.sampleCount, ...
        max(1, round((currentTime-runtime.schedule.time(1))/runtime.sampleTime)+1));
    qRef = qRefRaw;
    qdRef = qdRefRaw;
    qddRef = qddRefRaw;
    feedforward = feedforwardRaw;
    governed = struct('perturbation', zeros(3, 1), ...
        'rate', zeros(3, 1), 'acceleration', zeros(3, 1));
    if runtime.useCommandGovernor
        baseQ = runtime.governorBaseReference.q(:, sampleIndex);
        rawNoise = qRefRaw(4:6)-baseQ(4:6);
        governorState = struct('perturbation', block.Dwork(14).Data, ...
            'rate', block.Dwork(15).Data);
        [governed, governorState] = stepAttitudeCommandGovernor( ...
            rawNoise, governorState, runtime.commandGovernorConfig);
        qRef = baseQ;
        qRef(4:6) = qRef(4:6)+governed.perturbation;
        qdRef = runtime.governorBaseReference.qd(:, sampleIndex);
        qdRef(4:6) = qdRef(4:6)+governed.rate;
        qddRef = runtime.governorBaseReference.qdd(:, sampleIndex);
        qddRef(4:6) = qddRef(4:6)+governed.acceleration;
        feedforward = computeComputedTorqueForce(qRef, qdRef, qddRef, ...
            q, qd, runtime.model, runtime.poseConfig);
        block.Dwork(14).Data = governorState.perturbation;
        block.Dwork(15).Data = governorState.rate;
    end
    item = runtime.schedule.samples(runtime.schedule.referenceIndex(sampleIndex));
    lqiState = block.Dwork(3).Data;
    antiWindup = block.Dwork(4).Data;
    [nextLqi, nextAntiWindup, rawFeedback, feedback] = ...
        stepDiscreteLqiController( ...
        item, lqiState, qRef-q, antiWindup, runtime.feedbackForceLimit);

    dobCompensation = zeros(6, 1);
    dobEstimate = zeros(6, 1);
    if runtime.useDob
        dobState = struct('wrenchEstimate', block.Dwork(5).Data, ...
            'legCompensation', block.Dwork(6).Data, ...
            'initialized', logical(block.Dwork(8).Data));
        if block.Dwork(8).Data > 0
            measuredAcceleration = (qd-block.Dwork(7).Data)/runtime.sampleTime;
        else
            measuredAcceleration = zeros(6, 1);
        end
        [dobCompensation, dobState] = stepDisturbanceObserver(q, qd, ...
            measuredAcceleration, block.Dwork(1).Data, runtime.model, ...
            runtime.dobConfig, dobState);
        dobEstimate = dobState.wrenchEstimate;
        block.Dwork(5).Data = dobState.wrenchEstimate;
        block.Dwork(6).Data = dobState.legCompensation;
        block.Dwork(8).Data = double(dobState.initialized);
    end
    regulatedNominal = feedforward+feedback;
    nominal = regulatedNominal+dobCompensation;
    qpSolve = nan;
    feasible = true;
    fallback = false;
    minimumCbf = nan;
    if runtime.useStrictQp
        strict = runtime.strictConfig;
        strict.time = currentTime;
        try
            [command, qp] = stepStrictClfCbfQp(q, qd, qRef, qdRef, qddRef, ...
                regulatedNominal, block.Dwork(1).Data, runtime.model, ...
                runtime.scene, strict, dobCompensation, dobEstimate);
            qpSolve = qp.solveTime;
            feasible = qp.feasible;
            fallback = qp.usedFallback;
            minimumCbf = qp.minimumCbfResidual;
        catch
            fallback = true;
            feasible = false;
            rateStep = strict.forceRateLimit(:)*strict.dt;
            lower = max(strict.forceMin(:), block.Dwork(1).Data-rateStep);
            upper = min(strict.forceMax(:), block.Dwork(1).Data+rateStep);
            command = min(max(nominal, lower), upper);
            minimumCbf = -inf;
        end
    else
        command = min(max(nominal, runtime.model.actuator.forceMin), ...
            runtime.model.actuator.forceMax);
    end
    qpAwareMismatch = zeros(6, 1);
    if runtime.useQpAwareAntiWindup && runtime.useStrictQp
        [nextLqi, nextAntiWindup] = closeLqiAntiWindupWithAppliedForce( ...
            item, lqiState, qRef-q, rawFeedback, command, feedforward, ...
            dobCompensation, runtime.qpAwareAntiWindupMismatchLimit, ...
            runtime.qpAwareAntiWindupGain);
        qpAwareMismatch = nextAntiWindup;
    end
    stepTime = toc(timer);
    diagnostic = nan(runtime.diagnosticLayout.width, 1);
    layout = runtime.diagnosticLayout;
    diagnostic(layout.controllerStepTime) = stepTime;
    diagnostic(layout.qpSolveTime) = qpSolve;
    diagnostic(layout.feasible) = double(feasible);
    diagnostic(layout.usedFallback) = double(fallback);
    diagnostic(layout.minimumCbfResidual) = minimumCbf;
    diagnostic(layout.dobWrenchEstimate) = dobEstimate;
    diagnostic(layout.dobLegCompensation) = dobCompensation;
    diagnostic(layout.qpAwareAntiWindupMismatch) = qpAwareMismatch;
    diagnostic(layout.governedAttitudePerturbation) = governed.perturbation;
    diagnostic(layout.governedAttitudeRate) = governed.rate;
    diagnostic(layout.governedAttitudeAcceleration) = governed.acceleration;

    block.Dwork(1).Data = command;
    block.Dwork(2).Data = command;
    block.Dwork(3).Data = nextLqi;
    block.Dwork(4).Data = nextAntiWindup;
    block.Dwork(7).Data = qd;
    block.Dwork(9).Data = currentTime;
    block.Dwork(10).Data = q;
    block.Dwork(11).Data = qd;
    block.Dwork(12).Data = nominal;
    block.Dwork(13).Data = diagnostic;
end
block.OutputPort(1).Data = block.Dwork(2).Data;
block.OutputPort(2).Data = block.Dwork(10).Data;
block.OutputPort(3).Data = block.Dwork(11).Data;
block.OutputPort(4).Data = block.Dwork(12).Data;
block.OutputPort(5).Data = block.Dwork(13).Data;
end
