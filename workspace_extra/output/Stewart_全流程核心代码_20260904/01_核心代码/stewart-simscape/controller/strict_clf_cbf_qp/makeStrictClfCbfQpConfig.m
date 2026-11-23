function config = makeStrictClfCbfQpConfig(model, overrides)
% makeStrictClfCbfQpConfig - Configure the strict CLF-CBF-QP controller.
%
% The controller has six physical decision variables (leg forces) and one
% nonnegative CLF relaxation.  Every CBF and actuator constraint is hard.

arguments
    model struct
    overrides struct = struct()
end

config = struct();
config.methodName = "Strict CLF-CBF-QP";
config.shortName = "S-CLF-CBF-QP";
config.controllerType = 11;
config.dt = 0.01;
config.time = 0.0;

config.forceMin = model.actuator.forceMin(:);
config.forceMax = model.actuator.forceMax(:);
forceScale = max(abs([config.forceMin; config.forceMax]));
config.forceRateLimit = 4 * forceScale * ones(6, 1);
config.lengthMin = model.lmin(:);
config.lengthMax = model.lmax(:);
config.lengthMargin = 0.0;
config.legSpeedLimit = model.actuator.ldotMax(:);
config.legAccelerationLimit = model.actuator.lddotMax(:);
config.sigmaSafe = model.singularity.sigmaMinSafe;

config.clf = struct();
config.clf.positionGain = diag([30, 30, 30, 18, 18, 18]);
config.clf.velocityGain = diag([11, 11, 11, 8, 8, 8]);
config.clf.rate = 2.0;
config.clf.P = makeClfMatrix(config.clf.positionGain, config.clf.velocityGain);

config.cbf = struct();
config.cbf.lengthAlpha1 = 4.0;
config.cbf.lengthAlpha2 = 4.0;
config.cbf.speedAlpha = 5.0;
config.cbf.collisionAlpha1 = 12.0;
config.cbf.collisionAlpha2 = 12.0;
config.cbf.singularityAlpha1 = 5.0;
config.cbf.singularityAlpha2 = 5.0;

config.collision = struct();
config.collision.roofStage1Distance = 0.016;
config.collision.roofFinalDistance = 0.005;
config.collision.sideDistance = 0.015;
config.collision.approachDuration = 5.0;
config.collision.insertionDuration = 2.5;
config.collision.smoothingEpsilon = 1e-6;

config.weights = struct();
config.weights.force = ones(6, 1);
config.weights.forceRate = 2e-3 * ones(6, 1);
config.weights.clfSlack = 1e6;

config.solver = struct();
config.solver.algorithm = "interior-point-convex";
config.solver.constraintTolerance = 1e-8;
config.solver.optimalityTolerance = 1e-8;
config.solver.maxIterations = 100;
config.solver.activeTolerance = 1e-6;

config.fallback = struct();
config.fallback.nominalBlend = 0.25;
config.fallback.isSafetyCertified = false;

config = mergeStructRecursive(config, overrides);
if clfGainsOverriddenWithoutExplicitP(overrides)
    config.clf.P = makeClfMatrix( ...
        config.clf.positionGain, config.clf.velocityGain);
end
config = normalizeAndValidate(config);
end

function tf = clfGainsOverriddenWithoutExplicitP(overrides)
tf = false;
if ~isfield(overrides, 'clf') || ~isstruct(overrides.clf)
    return;
end
gainWasOverridden = isfield(overrides.clf, 'positionGain') || ...
    isfield(overrides.clf, 'velocityGain');
tf = gainWasOverridden && ~isfield(overrides.clf, 'P');
end

function P = makeClfMatrix(Kp, Kd)
% Solve A'P + PA = -I without requiring Control System Toolbox.
A = [zeros(6), eye(6); -Kp, -Kd];
Q = eye(12);
operator = kron(eye(12), A.') + kron(A.', eye(12));
P = reshape(operator \ (-Q(:)), 12, 12);
P = 0.5 * (P + P.');
end

function target = mergeStructRecursive(target, source)
fields = fieldnames(source);
for index = 1:numel(fields)
    name = fields{index};
    if ~isfield(target, name)
        error('makeStrictClfCbfQpConfig:UnknownOverride', ...
            'Unknown strict-QP configuration field: %s.', name);
    end
    if isstruct(target.(name)) && isstruct(source.(name))
        target.(name) = mergeStructRecursive(target.(name), source.(name));
    else
        target.(name) = source.(name);
    end
end
end

function config = normalizeAndValidate(config)
vectorFields = {'forceMin', 'forceMax', 'forceRateLimit', 'lengthMin', ...
    'lengthMax', 'legSpeedLimit', 'legAccelerationLimit'};
for index = 1:numel(vectorFields)
    name = vectorFields{index};
    value = config.(name)(:);
    if isscalar(value)
        value = repmat(value, 6, 1);
    end
    config.(name) = value;
    validateattributes(value, {'double'}, {'real', 'finite', 'size', [6, 1]}, ...
        mfilename, name);
end

config.weights.force = expandSix(config.weights.force, 'weights.force');
config.weights.forceRate = expandSix(config.weights.forceRate, 'weights.forceRate');

validateattributes(config.dt, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.time, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(config.lengthMargin, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(config.sigmaSafe, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.clf.positionGain, {'double'}, ...
    {'real', 'finite', 'size', [6, 6]});
validateattributes(config.clf.velocityGain, {'double'}, ...
    {'real', 'finite', 'size', [6, 6]});
validateattributes(config.clf.P, {'double'}, {'real', 'finite', 'size', [12, 12]});
validateattributes(config.clf.rate, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.weights.clfSlack, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.collision.smoothingEpsilon, {'double'}, ...
    {'scalar', 'positive', 'finite'});

cbfGainFields = {'lengthAlpha1', 'lengthAlpha2', 'speedAlpha', ...
    'collisionAlpha1', 'collisionAlpha2', 'singularityAlpha1', ...
    'singularityAlpha2'};
for index = 1:numel(cbfGainFields)
    name = cbfGainFields{index};
    validateattributes(config.cbf.(name), {'double'}, ...
        {'scalar', 'positive', 'finite'}, mfilename, ['cbf.' name]);
end

if any(config.forceMin >= config.forceMax)
    error('makeStrictClfCbfQpConfig:InvalidForceBounds', ...
        'forceMin must be strictly smaller than forceMax.');
end
if any(config.lengthMin + config.lengthMargin >= ...
        config.lengthMax - config.lengthMargin)
    error('makeStrictClfCbfQpConfig:InvalidLengthBounds', ...
        'The length margin leaves an empty admissible interval.');
end
if any(config.forceRateLimit <= 0) || any(config.legSpeedLimit <= 0) || ...
        any(config.legAccelerationLimit <= 0)
    error('makeStrictClfCbfQpConfig:InvalidPositiveLimit', ...
        'Rate, speed, and acceleration limits must be positive.');
end
if min(eig(0.5 * (config.clf.P + config.clf.P.'))) <= 0
    error('makeStrictClfCbfQpConfig:InvalidClfMatrix', ...
        'clf.P must be symmetric positive definite.');
end
end

function value = expandSix(value, name)
value = value(:);
if isscalar(value)
    value = repmat(value, 6, 1);
end
validateattributes(value, {'double'}, {'real', 'finite', 'positive', 'size', [6, 1]}, ...
    mfilename, name);
end
