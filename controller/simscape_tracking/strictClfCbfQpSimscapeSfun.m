function strictClfCbfQpSimscapeSfun(block)
% strictClfCbfQpSimscapeSfun - Normal-mode bridge from Simscape to strict QP.
%
% Inputs: relative pose, measured spatial velocity [v;omega], absolute
% q-reference, qdot-reference, qddot-reference, and complete nominal force.
% Outputs: QP force, absolute q, qdot, nominal force, fixed diagnostics.

setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 6;
block.NumOutputPorts = 5;
for index = 1:block.NumInputPorts
    block.InputPort(index).Dimensions = 6;
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
end

runtime = block.DialogPrm(1).Data;
layout = runtime.diagnosticLayout;
outputWidths = [6, 6, 6, 6, layout.width];
for index = 1:block.NumOutputPorts
    block.OutputPort(index).Dimensions = outputWidths(index);
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
widths = [6, 6, runtime.diagnosticLayout.width, 1, 6, 6, 6, 1, 1];
names = {'PreviousForce', 'ForceCommand', 'Diagnostic', 'LastSolveTime', ...
    'AbsolutePose', 'GeneralizedVelocity', 'NominalForce', 'SolverErrorCount', ...
    'SimulinkInitializationTime'};
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
initialForce = reshape(runtime.initialForce, 6, 1);
initialPose = reshape(runtime.q0, 6, 1);
diagnostic = nan(runtime.diagnosticLayout.width, 1);
diagnostic(runtime.diagnosticLayout.header.feasible) = 0;
diagnostic(runtime.diagnosticLayout.header.usedFallback) = 1;
diagnostic(runtime.diagnosticLayout.header.exitflag) = -20;
diagnostic(runtime.diagnosticLayout.header.solverError) = 0;

% Warm the complete interpreted call chain once inside the S-function
% execution context.  This initialization cost is logged separately and
% excluded from online controller-step timing.
warmTimer = tic;
warmConfig = runtime.config;
warmConfig.time = 0;
for warmIndex = 1:5
    stepStrictClfCbfQp(runtime.q0, runtime.initialQd, ...
        runtime.initialReferencePose, runtime.initialReferenceVelocity, ...
        runtime.initialReferenceAcceleration, initialForce, initialForce, ...
        runtime.model, runtime.scene, warmConfig);
end
simulinkInitializationTime = toc(warmTimer);
diagnostic(runtime.diagnosticLayout.header.simulinkInitializationTime) = ...
    simulinkInitializationTime;

block.Dwork(1).Data = initialForce;
block.Dwork(2).Data = initialForce;
block.Dwork(3).Data = diagnostic;
block.Dwork(4).Data = -inf;
block.Dwork(5).Data = initialPose;
block.Dwork(6).Data = zeros(6, 1);
block.Dwork(7).Data = initialForce;
block.Dwork(8).Data = 0;
block.Dwork(9).Data = simulinkInitializationTime;
end

function outputs(block)
runtime = block.DialogPrm(1).Data;
currentTime = block.CurrentTime;
lastTime = block.Dwork(4).Data;
timeTolerance = max(1e-12, 32*eps(max(1, abs(currentTime))));
if ~isfinite(lastTime) || abs(currentTime-lastTime) > timeTolerance
    relativePose = reshape(block.InputPort(1).Data, 6, 1);
    spatialVelocity = reshape(block.InputPort(2).Data, 6, 1);
    referencePose = reshape(block.InputPort(3).Data, 6, 1);
    referenceVelocity = reshape(block.InputPort(4).Data, 6, 1);
    referenceAcceleration = reshape(block.InputPort(5).Data, 6, 1);
    nominalForce = reshape(block.InputPort(6).Data, 6, 1);

    absolutePose = runtime.q0(:)+relativePose;
    rateMap = rpyRateMapZYX(absolutePose(4:6));
    if rcond(rateMap) < runtime.rpyRateRcondMin
        rpyRate = pinv(rateMap)*spatialVelocity(4:6);
    else
        rpyRate = rateMap\spatialVelocity(4:6);
    end
    generalizedVelocity = [spatialVelocity(1:3); rpyRate];
    config = runtime.config;
    config.time = currentTime;
    previousForce = block.Dwork(1).Data;
    solverError = false;
    stepTimer = tic;
    try
        [forceCommand, diagnostic] = stepStrictClfCbfQp( ...
            absolutePose, generalizedVelocity, referencePose, ...
            referenceVelocity, referenceAcceleration, nominalForce, ...
            previousForce, runtime.model, runtime.scene, config);
    catch exception
        solverError = true;
        block.Dwork(8).Data = block.Dwork(8).Data+1;
        if block.Dwork(8).Data == 1
            warning('strictClfCbfQpSimscapeSfun:StrictQpStepFailed', ...
                'Strict QP failed at t=%.6g s; using uncertified fallback. %s', ...
                currentTime, exception.message);
        end
        rateStep = config.forceRateLimit(:)*config.dt;
        lower = max(config.forceMin(:), previousForce-rateStep);
        upper = min(config.forceMax(:), previousForce+rateStep);
        forceCommand = min(max(nominalForce, lower), upper);
        diagnostic = makeErrorDiagnostic(config, lower, upper);
    end
    diagnostic.controllerStepTime = toc(stepTimer);
    encoded = encodeDiagnostic(diagnostic, forceCommand, config, ...
        runtime.diagnosticLayout, solverError);
    encoded(runtime.diagnosticLayout.header.simulinkInitializationTime) = ...
        block.Dwork(9).Data;

    block.Dwork(1).Data = forceCommand;
    block.Dwork(2).Data = forceCommand;
    block.Dwork(3).Data = encoded;
    block.Dwork(4).Data = currentTime;
    block.Dwork(5).Data = absolutePose;
    block.Dwork(6).Data = generalizedVelocity;
    block.Dwork(7).Data = nominalForce;
end

block.OutputPort(1).Data = block.Dwork(2).Data;
block.OutputPort(2).Data = block.Dwork(5).Data;
block.OutputPort(3).Data = block.Dwork(6).Data;
block.OutputPort(4).Data = block.Dwork(7).Data;
block.OutputPort(5).Data = block.Dwork(3).Data;
end

function diagnostic = makeErrorDiagnostic(config, lower, upper)
diagnostic = struct();
diagnostic.clfSlack = nan;
diagnostic.solveTime = nan;
diagnostic.controllerStepTime = nan;
diagnostic.feasible = false;
diagnostic.usedFallback = true;
diagnostic.exitflag = -99;
diagnostic.minimumCbfResidual = -inf;
diagnostic.hardConstraintViolation = inf;
diagnostic.cbfResidual = nan(40, 1);
diagnostic.forceLower = lower;
diagnostic.forceUpper = upper;
diagnostic.clf = struct('V', nan);
diagnostic.cbf = struct('stateMargins', nan(28, 1), ...
    'psi1', nan(16, 1), 'minimumStateMargin', -inf, 'minimumPsi1', -inf);
diagnostic.sigmaLowerSquared = nan;
diagnostic.usesFiniteDifferences = true;
diagnostic.activeConstraints = strings(0, 1);
diagnostic.fallbackSafetyCertified = config.fallback.isSafetyCertified;
end

function vector = encodeDiagnostic(diagnostic, forceCommand, config, layout, solverError)
vector = nan(layout.width, 1);
h = layout.header;
vector(h.clfSlack) = scalarField(diagnostic, 'clfSlack', nan);
vector(h.solveTime) = scalarField(diagnostic, 'solveTime', nan);
vector(h.controllerStepTime) = scalarField(diagnostic, 'controllerStepTime', nan);
vector(h.feasible) = double(logicalField(diagnostic, 'feasible', false));
vector(h.usedFallback) = double(logicalField(diagnostic, 'usedFallback', true));
vector(h.exitflag) = scalarField(diagnostic, 'exitflag', -99);
vector(h.minimumCbfResidual) = scalarField(diagnostic, 'minimumCbfResidual', nan);
vector(h.hardConstraintViolation) = scalarField(diagnostic, 'hardConstraintViolation', nan);
vector(h.clfValue) = nestedScalar(diagnostic, {'clf', 'V'}, nan);
vector(h.sigmaLowerSquared) = scalarField(diagnostic, 'sigmaLowerSquared', nan);
vector(h.minimumStateMargin) = nestedScalar(diagnostic, ...
    {'cbf', 'minimumStateMargin'}, nan);
vector(h.minimumPsi1) = nestedScalar(diagnostic, {'cbf', 'minimumPsi1'}, nan);
vector(h.solverError) = double(solverError);
vector(h.usesFiniteDifferences) = double(logicalField( ...
    diagnostic, 'usesFiniteDifferences', true));

stateMargins = nestedVector(diagnostic, {'cbf', 'stateMargins'}, ...
    numel(layout.stateMargins));
psi1 = nestedVector(diagnostic, {'cbf', 'psi1'}, numel(layout.psi1));
cbfResidual = vectorField(diagnostic, 'cbfResidual', numel(layout.cbfResidual));
vector(layout.stateMargins) = stateMargins;
vector(layout.psi1) = psi1;
vector(layout.cbfResidual) = cbfResidual;

activeTolerance = config.solver.activeTolerance;
activeCbf = isfinite(cbfResidual) & cbfResidual <= activeTolerance;
lower = vectorField(diagnostic, 'forceLower', 6);
upper = vectorField(diagnostic, 'forceUpper', 6);
activeBounds = false(12, 1);
for index = 1:6
    activeBounds(2*index-1) = isfinite(lower(index)) && ...
        forceCommand(index)-lower(index) <= activeTolerance;
    activeBounds(2*index) = isfinite(upper(index)) && ...
        upper(index)-forceCommand(index) <= activeTolerance;
end
active = [activeCbf; activeBounds];
vector(layout.activeMask) = double(active);
vector(h.activeConstraintCount) = sum(active);
end

function value = scalarField(source, name, fallback)
value = fallback;
if isfield(source, name) && isscalar(source.(name))
    value = double(source.(name));
end
end

function value = logicalField(source, name, fallback)
value = fallback;
if isfield(source, name) && isscalar(source.(name))
    value = logical(source.(name));
end
end

function value = nestedScalar(source, path, fallback)
value = source;
for index = 1:numel(path)
    if ~isstruct(value) || ~isfield(value, path{index})
        value = fallback;
        return;
    end
    value = value.(path{index});
end
if ~isscalar(value)
    value = fallback;
else
    value = double(value);
end
end

function value = vectorField(source, name, count)
if isfield(source, name)
    value = source.(name)(:);
else
    value = nan(count, 1);
end
if numel(value) ~= count
    value = nan(count, 1);
end
end

function value = nestedVector(source, path, count)
value = source;
for index = 1:numel(path)
    if ~isstruct(value) || ~isfield(value, path{index})
        value = nan(count, 1);
        return;
    end
    value = value.(path{index});
end
value = value(:);
if numel(value) ~= count
    value = nan(count, 1);
end
end
