function config = makeStewartFgMheConfig(model, sampleTime, overrides)
% makeStewartFgMheConfig - 构造 SC-FG-MHE 滑窗估计配置
arguments
    model struct
    sampleTime (1, 1) double {mustBePositive, mustBeFinite}
    overrides struct = struct()
end

homeLength = sgpIK(model.qHome, model).L;
lengthMargin = 0.08;

config = struct();
config.method = "SC-FG-MHE";
config.windowLength = 8;
config.sampleTime = sampleTime;
config.anchorPose = model.qHome(:);
config.anchorLength = homeLength;

config.weights = struct();
config.weights.leg = 1 / 5e-4;
config.weights.attitude = 1 / deg2rad(0.05);
config.weights.imu = 1;
config.weights.imuPosition = 1 / 5e-3;
config.weights.imuVelocity = 1 / 0.2;
config.weights.geometry = 1;
config.weights.dynamics = 1 / 8;
config.weights.actuator = 1 / 10;
config.weights.parameterPrior = 1;
config.weights.statePrior = 1 / 2e-4;

config.robust = struct();
config.robust.enabled = false;
config.robust.kernel = "none";
config.robust.huberDelta = 1.0;

config.estimate = struct();
config.estimate.qdd = true;
config.estimate.payload = true;
config.estimate.actuator = true;

config.solver = struct();
config.solver.maxIterations = 60;
config.solver.display = 'off';
config.solver.functionTolerance = 1e-9;
config.solver.stepTolerance = 1e-9;

config.geometry = struct();
config.geometry.legLengthMin = homeLength - lengthMargin;
config.geometry.legLengthMax = homeLength + lengthMargin;
config.geometry.sigmaSafe = 0.05;

config.outputs = struct();
config.outputs.reportSubdir = fullfile('results', 'reports', 'factor_graph_estimation');

fields = fieldnames(overrides);
for index = 1:numel(fields)
    field = fields{index};
    if ~isfield(config, field)
        error('makeStewartFgMheConfig:UnknownOverride', ...
            '未知 SC-FG-MHE 配置项：%s。', field);
    end
    if isstruct(config.(field)) && isstruct(overrides.(field))
        config.(field) = mergeKnownStruct(config.(field), overrides.(field), field);
    else
        config.(field) = overrides.(field);
    end
end

config.method = string(config.method);
config.robust.kernel = string(config.robust.kernel);
end

function target = mergeKnownStruct(target, patch, parentName)
fields = fieldnames(patch);
for index = 1:numel(fields)
    field = fields{index};
    if ~isfield(target, field)
        error('makeStewartFgMheConfig:UnknownOverride', ...
            '未知 SC-FG-MHE 配置项：%s.%s。', parentName, field);
    end
    target.(field) = patch.(field);
end
end
