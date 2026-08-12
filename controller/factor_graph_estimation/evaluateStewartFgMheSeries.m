function series = evaluateStewartFgMheSeries(overrides)
% evaluateStewartFgMheSeries - 构建合成 benchmark 并运行 SC-FG-MHE
arguments
    overrides struct = struct()
end

options = defaultOptions(overrides);
model = options.model;
config = options.config;
truth = options.truth;

[trajectory, measurements, teacher] = makeSyntheticData(model, config, truth, options);
windows = makeWindows(model, config, trajectory, measurements, teacher);
series = rollStewartFgMheEstimator(windows, config);

series.trajectory = trajectory;
series.measurements = measurements;
series.teacher = teacher;
series.truth = truth;
series.config = config;
series.variant = string(options.runVariants(1));
end

function options = defaultOptions(overrides)
options = struct();
options.model = buildOptModelCustom();
options.config = makeStewartFgMheConfig(options.model, 0.01);
options.truth = struct();
options.truth.bL = zeros(6, 1);
options.truth.bAtt = zeros(3, 1);
options.truth.ba = zeros(3, 1);
options.truth.bg = zeros(3, 1);
options.truth.dm = 0.25;
options.truth.dc = zeros(3, 1);
options.truth.kF = ones(6, 1);
options.smoke = true;
options.runVariants = "fg_mhe_with_dynamics";

fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('evaluateStewartFgMheSeries:UnknownOverride', ...
            '未知 SC-FG-MHE benchmark 配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end
options.runVariants = string(options.runVariants);
options.truth = completeTruth(options.truth);
end

function truth = completeTruth(truth)
defaults = struct('bL', zeros(6, 1), 'bAtt', zeros(3, 1), ...
    'ba', zeros(3, 1), 'bg', zeros(3, 1), 'dm', 0, ...
    'dc', zeros(3, 1), 'kF', ones(6, 1));
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(truth, names{index})
        truth.(names{index}) = defaults.(names{index});
    end
    truth.(names{index}) = truth.(names{index})(:);
end
truth.dm = truth.dm(1);
end

function [trajectory, measurements, teacher] = makeSyntheticData(model, config, truth, options)
if options.smoke
    sampleCount = config.windowLength;
else
    sampleCount = max(3 * config.windowLength, 24);
end
dt = config.sampleTime;
t = (0:sampleCount - 1) * dt;
q = repmat(model.qHome(:), 1, sampleCount);
q(1, :) = q(1, :) + 2e-4 * sin(2 * pi * t / max(t(end), dt));
q(2, :) = q(2, :) + 1.5e-4 * cos(2 * pi * t / max(t(end), dt));
q(4, :) = q(4, :) + deg2rad(0.03) * sin(2 * pi * t / max(t(end), dt));
qd = finiteDifferenceRows(q, dt);
qdd = finiteDifferenceRows(qd, dt);

anchorLength = sgpIK(q(:, 1), model).L;
relativeLength = zeros(6, sampleCount);
teacherWrench = zeros(6, sampleCount);
teacherLegForce = zeros(6, sampleCount);
teacherPwmForce = zeros(6, sampleCount);
teacherModel = payloadAdjustedModel(model, truth.dm, truth.dc);
for index = 1:sampleCount
    relativeLength(:, index) = sgpIK(q(:, index), model).L - anchorLength + truth.bL;
    teacherWrench(:, index) = computeCompositeRequiredWrench( ...
        q(:, index), qd(:, index), qdd(:, index), teacherModel);
    jac = sgpJacobian(q(:, index), model);
    teacherLegForce(:, index) = pinv(jac.Jv.') * teacherWrench(:, index);
    teacherPwmForce(:, index) = teacherLegForce(:, index) ./ max(truth.kF(:), eps);
end

measurements = struct();
measurements.relativeLength = relativeLength;
measurements.orientation = q(4:6, :) + truth.bAtt;
measurements.worldAcceleration = qdd(1:3, :) + truth.ba;
measurements.angularVelocity = qd(4:6, :) + truth.bg;
measurements.anchorLength = anchorLength;

teacher = struct();
teacher.wrench = teacherWrench;
teacher.legForce = teacherLegForce;
teacher.pwmForce = teacherPwmForce;

trajectory = struct();
trajectory.t = t;
trajectory.qTrue = q;
trajectory.qdTrue = qd;
trajectory.qddTrue = qdd;
end

function windows = makeWindows(model, config, trajectory, measurements, teacher)
sampleCount = numel(trajectory.t);
W = config.windowLength;
windowCount = max(1, sampleCount - W + 1);
windows = cell(1, windowCount);
for index = 1:windowCount
    ids = index:(index + W - 1);
    window = struct();
    window.model = model;
    window.t = trajectory.t(ids);
    window.dt = config.sampleTime;
    window.anchorLength = measurements.anchorLength;
    window.measurements = struct( ...
        'relativeLength', measurements.relativeLength(:, ids), ...
        'orientation', measurements.orientation(:, ids), ...
        'worldAcceleration', measurements.worldAcceleration(:, ids), ...
        'angularVelocity', measurements.angularVelocity(:, ids));
    window.initial = struct( ...
        'q', trajectory.qTrue(:, ids), ...
        'qd', trajectory.qdTrue(:, ids), ...
        'qdd', trajectory.qddTrue(:, ids));
    window.teacher = struct( ...
        'wrench', teacher.wrench(:, ids), ...
        'legForce', teacher.legForce(:, ids), ...
        'pwmForce', teacher.pwmForce(:, ids));
    windows{index} = window;
end
end

function derivative = finiteDifferenceRows(value, sampleTime)
derivative = zeros(size(value));
if size(value, 2) < 2
    return;
end
derivative(:, 1) = (value(:, 2) - value(:, 1)) / sampleTime;
derivative(:, end) = (value(:, end) - value(:, end - 1)) / sampleTime;
if size(value, 2) > 2
    derivative(:, 2:end - 1) = ...
        (value(:, 3:end) - value(:, 1:end - 2)) / (2 * sampleTime);
end
end

function model = payloadAdjustedModel(model, dm, dc)
if isfield(model, 'dynamics') && isfield(model.dynamics, 'totalMass')
    model.dynamics.totalMass = model.dynamics.totalMass + dm;
end
if isfield(model, 'dynamics') && isfield(model.dynamics, 'comP')
    model.dynamics.comP = model.dynamics.comP + dc(:);
end
end
