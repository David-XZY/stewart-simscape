function [residual, breakdown, variables] = buildStewartFgMheResiduals(z, layout, window, config, previous)
% buildStewartFgMheResiduals - 构建 SC-FG-MHE 多因子 residual
arguments
    z
    layout struct
    window struct
    config struct
    previous struct = struct()
end

if isstruct(z)
    variables = z;
else
    variables = buildStewartFgMheVariables("unpack", z, config, layout);
end

parts = {};
breakdown = struct();
[parts, breakdown] = appendPart(parts, breakdown, 'leg', legResidual(variables, window, config));
[parts, breakdown] = appendPart(parts, breakdown, 'attitude', attitudeResidual(variables, window, config));
[parts, breakdown] = appendPart(parts, breakdown, 'imu', imuResidual(variables, window, config));
[parts, breakdown] = appendPart(parts, breakdown, 'geometry', geometryResidual(variables, window, config));
[parts, breakdown] = appendPart(parts, breakdown, 'dynamics', dynamicsResidual(variables, window, config));
[parts, breakdown] = appendPart(parts, breakdown, 'actuator', actuatorResidual(variables, window, config));
[parts, breakdown] = appendPart(parts, breakdown, 'parameterPrior', parameterPriorResidual(variables, previous, config));
[parts, breakdown] = appendPart(parts, breakdown, 'statePrior', statePriorResidual(variables, window, config));

if isempty(parts)
    residual = zeros(0, 1);
else
    residual = vertcat(parts{:});
end
end

function [parts, breakdown] = appendPart(parts, breakdown, name, value)
value = value(:);
parts{end + 1} = value; %#ok<AGROW>
breakdown.(name) = struct( ...
    'count', numel(value), ...
    'rms', sqrt(mean(value.^2, 'omitnan')));
end

function r = legResidual(variables, window, config)
W = config.windowLength;
r = zeros(6, W);
relativeLength = window.measurements.relativeLength;
anchorLength = window.anchorLength(:);
for index = 1:W
    predicted = sgpIK(variables.q(:, index), window.model).L;
    measured = anchorLength + relativeLength(:, index);
    r(:, index) = config.weights.leg * robustValue( ...
        measured - predicted - variables.bL, config);
end
end

function r = attitudeResidual(variables, window, config)
attitude = window.measurements.orientation(:, 1:config.windowLength);
r = config.weights.attitude * robustValue( ...
    wrapAngleLocal(attitude - variables.q(4:6, :) - variables.bAtt), config);
end

function r = imuResidual(variables, window, config)
W = config.windowLength;
dt = window.dt;
r = zeros(12, W - 1);
acc = getMeasurement(window.measurements, 'worldAcceleration', zeros(3, W));
gyro = getMeasurement(window.measurements, 'angularVelocity', zeros(3, W));
for index = 1:(W - 1)
    predictedQ = variables.q(:, index);
    predictedQ(1:3) = predictedQ(1:3) + variables.qd(1:3, index) * dt + ...
        0.5 * (acc(:, index) - variables.ba) * dt^2;
    predictedQ(4:6) = predictedQ(4:6) + (gyro(:, index) - variables.bg) * dt;
    predictedQd = variables.qd(:, index);
    predictedQd(1:3) = predictedQd(1:3) + (acc(:, index) - variables.ba) * dt;
    predictedQd(4:6) = gyro(:, index) - variables.bg;
    r(:, index) = [
        config.weights.imuPosition * (variables.q(:, index + 1) - predictedQ)
        config.weights.imuVelocity * (variables.qd(:, index + 1) - predictedQd)
        ];
end
end

function r = geometryResidual(variables, window, config)
W = config.windowLength;
r = zeros(7, W);
for index = 1:W
    kin = sgpIK(variables.q(:, index), window.model);
    below = max(zeros(6, 1), config.geometry.legLengthMin(:) - kin.L);
    above = max(zeros(6, 1), kin.L - config.geometry.legLengthMax(:));
    jac = sgpJacobian(variables.q(:, index), window.model);
    singularity = max(0, config.geometry.sigmaSafe - jac.sigmaMin);
    r(:, index) = config.weights.geometry * [below + above; singularity];
end
end

function r = dynamicsResidual(variables, window, config)
if ~config.estimate.payload || ~isfield(window, 'teacher') || ~isfield(window.teacher, 'wrench')
    r = zeros(0, 1);
    return;
end
W = config.windowLength;
r = zeros(6, W);
for index = 1:W
    model = payloadAdjustedModel(window.model, variables);
    required = computeCompositeRequiredWrench( ...
        variables.q(:, index), variables.qd(:, index), variables.qdd(:, index), model);
    r(:, index) = config.weights.dynamics * robustValue( ...
        required - window.teacher.wrench(:, index), config);
end
end

function r = actuatorResidual(variables, window, config)
if ~config.estimate.actuator || ~isfield(window, 'teacher') || ~isfield(window.teacher, 'legForce')
    r = zeros(0, 1);
    return;
end
W = config.windowLength;
r = zeros(6, W);
legSpeed = getMeasurement(window, 'legSpeed', zeros(6, W));
pwmForce = getMeasurement(window.teacher, 'pwmForce', window.teacher.legForce);
for index = 1:W
    predicted = variables.kF .* pwmForce(:, index) - variables.cF .* legSpeed(:, index);
    r(:, index) = config.weights.actuator * (predicted - window.teacher.legForce(:, index));
end
end

function r = parameterPriorResidual(variables, previous, config)
target = struct();
target.bL = zeros(6, 1);
target.bAtt = zeros(3, 1);
target.ba = zeros(3, 1);
target.bg = zeros(3, 1);
target.dm = 0;
target.dc = zeros(3, 1);
target.kF = ones(6, 1);
target.tauF = 0;
target.cF = zeros(6, 1);
if isfield(previous, 'estimate')
    previous = previous.estimate;
end
names = fieldnames(target);
pieces = cell(numel(names), 1);
for index = 1:numel(names)
    name = names{index};
    if isfield(previous, name)
        target.(name) = previous.(name);
    end
    pieces{index} = variables.(name)(:) - target.(name)(:);
end
r = config.weights.parameterPrior * vertcat(pieces{:});
end

function r = statePriorResidual(variables, window, config)
if ~isfield(window, 'initial')
    r = zeros(0, 1);
    return;
end
qPrior = getMeasurement(window.initial, 'q', variables.q);
qdPrior = getMeasurement(window.initial, 'qd', variables.qd);
qddPrior = getMeasurement(window.initial, 'qdd', variables.qdd);
qPrior = qPrior(:, 1:config.windowLength);
qdPrior = qdPrior(:, 1:config.windowLength);
qddPrior = qddPrior(:, 1:config.windowLength);
r = config.weights.statePrior * [
    variables.q(:) - qPrior(:);
    variables.qd(:) - qdPrior(:);
    variables.qdd(:) - qddPrior(:)
    ];
r = r(:);
end

function value = getMeasurement(source, name, defaultValue)
if isfield(source, name)
    value = source.(name);
else
    value = defaultValue;
end
end

function value = robustValue(value, config)
if ~config.robust.enabled
    return;
end
if config.robust.kernel ~= "huber"
    return;
end
delta = config.robust.huberDelta;
absValue = abs(value);
scale = ones(size(value));
mask = absValue > delta;
scale(mask) = sqrt(delta ./ max(absValue(mask), eps));
value = scale .* value;
end

function angle = wrapAngleLocal(angle)
angle = atan2(sin(angle), cos(angle));
end

function model = payloadAdjustedModel(model, variables)
if isfield(model, 'dynamics') && isfield(model.dynamics, 'totalMass')
    model.dynamics.totalMass = model.dynamics.totalMass + variables.dm;
end
if isfield(model, 'dynamics') && isfield(model.dynamics, 'comP')
    model.dynamics.comP = model.dynamics.comP + variables.dc;
end
end
