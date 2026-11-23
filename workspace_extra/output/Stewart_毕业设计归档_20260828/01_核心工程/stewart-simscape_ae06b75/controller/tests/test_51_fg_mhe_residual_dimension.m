function test_51_fg_mhe_residual_dimension
% test_51_fg_mhe_residual_dimension - 验证 SC-FG-MHE residual 维度和分组
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'factor_graph_estimation'));

model = buildOptModelCustom();
config = makeStewartFgMheConfig(model, 0.01, struct('windowLength', 5));
window = makeSyntheticFgWindow(model, config);
initial = initializeStewartFgMheWindow(window, config);
[z, layout] = buildStewartFgMheVariables("pack", initial, config);
[residual, breakdown, decoded] = buildStewartFgMheResiduals(z, layout, window, config);

assert(iscolumn(residual));
assert(isfield(decoded, 'q'));
assert(isequal(size(decoded.q), [6, config.windowLength]));
assert(breakdown.leg.count == 6 * config.windowLength);
assert(breakdown.attitude.count == 3 * config.windowLength);
assert(breakdown.imu.count == 12 * (config.windowLength - 1));
assert(isfield(breakdown, 'geometry'));
assert(breakdown.geometry.count >= config.windowLength);
assert(isfield(breakdown, 'parameterPrior'));
assert(breakdown.parameterPrior.count >= 6);
assert(isfield(breakdown, 'statePrior'));
assert(breakdown.statePrior.count == 18 * config.windowLength);
assert(numel(residual) == sumResidualCount(breakdown));
end

function window = makeSyntheticFgWindow(model, config)
W = config.windowLength;
t = (0:W - 1) * config.sampleTime;
q = repmat(model.qHome(:), 1, W);
q(1, :) = q(1, :) + 1e-4 * sin(2 * pi * t);
qd = zeros(6, W);
qdd = zeros(6, W);
anchorLength = sgpIK(q(:, 1), model).L;
relativeLength = zeros(6, W);
for index = 1:W
    relativeLength(:, index) = sgpIK(q(:, index), model).L - anchorLength;
end
window = struct();
window.model = model;
window.t = t;
window.dt = config.sampleTime;
window.anchorLength = anchorLength;
window.measurements = struct( ...
    'relativeLength', relativeLength, ...
    'orientation', q(4:6, :), ...
    'worldAcceleration', zeros(3, W), ...
    'angularVelocity', zeros(3, W));
window.initial = struct('q', q, 'qd', qd, 'qdd', qdd);
window.previous = struct();
end

function total = sumResidualCount(breakdown)
names = fieldnames(breakdown);
total = 0;
for index = 1:numel(names)
    total = total + breakdown.(names{index}).count;
end
end
