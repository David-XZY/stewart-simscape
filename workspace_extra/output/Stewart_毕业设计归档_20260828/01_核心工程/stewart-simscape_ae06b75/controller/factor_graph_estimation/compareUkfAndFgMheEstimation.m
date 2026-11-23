function comparison = compareUkfAndFgMheEstimation(overrides)
% compareUkfAndFgMheEstimation - 统一比较运动学、UKF 和 SC-FG-MHE
arguments
    overrides struct = struct()
end

options = defaultOptions(overrides);
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
controllerRoot = fullfile(projectRoot, 'controller');
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'pwm_identification'));
addpath(fullfile(controllerRoot, 'ukf_pose_estimation'));
addpath(fullfile(controllerRoot, 'factor_graph_estimation'));

model = options.model;
truth = options.truth;
config = makeStewartFgMheConfig(model, options.sampleTime, struct( ...
    'windowLength', options.windowLength));
baseSeries = evaluateStewartFgMheSeries(struct( ...
    'model', model, 'config', config, 'truth', truth, ...
    'smoke', options.smoke, 'runVariants', "fg_mhe_with_dynamics"));

trajectory = baseSeries.trajectory;
stableUkfBenchmark = benchmarkPwmPoseEstimators(trajectory, struct( ...
    'ukfVariant', "pose_bias_ukf", ...
    'relativeLengthBias', truth.bL, ...
    'orientationNoiseStd', zeros(3, 1), ...
    'ukfOrientationNoiseStd', zeros(3, 1), ...
    'encoderNoiseStd', 0, ...
    'accelerationNoiseStd', 0, ...
    'angularVelocityNoiseStd', 0));
legBiasUkfBenchmark = benchmarkPwmPoseEstimators(trajectory, struct( ...
    'ukfVariant', "pose_leg_bias_ukf", ...
    'relativeLengthBias', truth.bL, ...
    'orientationNoiseStd', zeros(3, 1), ...
    'ukfOrientationNoiseStd', zeros(3, 1), ...
    'encoderNoiseStd', 0, ...
    'accelerationNoiseStd', 0, ...
    'angularVelocityNoiseStd', 0));

fgVariants = [
    "fg_mhe_no_dynamics"
    "fg_mhe_with_dynamics"
    "fg_mhe_robust"
    "fg_mhe_actuator"
    ];
fgSeries = cell(numel(fgVariants), 1);
for index = 1:numel(fgVariants)
    fgConfig = variantConfig(model, options.sampleTime, options.windowLength, fgVariants(index));
    fgSeries{index} = evaluateStewartFgMheSeries(struct( ...
        'model', model, 'config', fgConfig, 'truth', truth, ...
        'smoke', options.smoke, 'runVariants', fgVariants(index)));
end

method = [
    "kinematic"
    "ukf_stable"
    "ukf_leg_bias"
    fgVariants
    ];
rowCount = numel(method);
position_rmse = zeros(rowCount, 1);
position_max_error = zeros(rowCount, 1);
attitude_rmse = zeros(rowCount, 1);
attitude_max_error = zeros(rowCount, 1);
leg_bias_rmse = zeros(rowCount, 1);
imu_bias_error = zeros(rowCount, 1);
payload_mass_error = zeros(rowCount, 1);
payload_com_error = zeros(rowCount, 1);
actuator_gain_error = zeros(rowCount, 1);
mean_solve_time = zeros(rowCount, 1);
max_solve_time = zeros(rowCount, 1);
failure_count = zeros(rowCount, 1);
outlier_sensitivity = zeros(rowCount, 1);

position_rmse(1) = stableUkfBenchmark.metrics.kinematicTranslationRms;
position_max_error(1) = stableUkfBenchmark.metrics.kinematicTranslationPeak;
attitude_rmse(1) = stableUkfBenchmark.metrics.kinematicRotationRms;
attitude_max_error(1) = stableUkfBenchmark.metrics.kinematicRotationRms;
position_rmse(2) = stableUkfBenchmark.metrics.ukfTranslationRms;
position_max_error(2) = stableUkfBenchmark.metrics.ukfTranslationPeak;
attitude_rmse(2) = stableUkfBenchmark.metrics.ukfRotationRms;
attitude_max_error(2) = stableUkfBenchmark.metrics.ukfRotationRms;
position_rmse(3) = legBiasUkfBenchmark.metrics.ukfTranslationRms;
position_max_error(3) = legBiasUkfBenchmark.metrics.ukfTranslationPeak;
attitude_rmse(3) = legBiasUkfBenchmark.metrics.ukfRotationRms;
attitude_max_error(3) = legBiasUkfBenchmark.metrics.ukfRotationRms;

for index = 1:numel(fgSeries)
    row = index + 3;
    metrics = fgMetrics(fgSeries{index}, truth);
    position_rmse(row) = metrics.positionRmse;
    position_max_error(row) = metrics.positionMax;
    attitude_rmse(row) = metrics.attitudeRmse;
    attitude_max_error(row) = metrics.attitudeMax;
    leg_bias_rmse(row) = metrics.legBiasRmse;
    imu_bias_error(row) = metrics.imuBiasError;
    payload_mass_error(row) = metrics.payloadMassError;
    payload_com_error(row) = metrics.payloadComError;
    actuator_gain_error(row) = metrics.actuatorGainError;
    mean_solve_time(row) = mean(fgSeries{index}.solveTimes);
    max_solve_time(row) = max(fgSeries{index}.solveTimes);
    failure_count(row) = fgSeries{index}.failureCount;
    outlier_sensitivity(row) = metrics.outlierSensitivity;
end

ukfReference = max(position_rmse(3), eps);
improvement_over_ukf_percent = 100 * (ukfReference - position_rmse) / ukfReference;

metrics = table(method, position_rmse, position_max_error, attitude_rmse, ...
    attitude_max_error, leg_bias_rmse, imu_bias_error, payload_mass_error, ...
    payload_com_error, actuator_gain_error, mean_solve_time, max_solve_time, ...
    failure_count, outlier_sensitivity, improvement_over_ukf_percent);

comparison = struct();
comparison.metrics = metrics;
comparison.ablation = buildAblationTable(metrics);
comparison.fgSeries = fgSeries;
comparison.trajectory = trajectory;
comparison.truth = truth;
comparison.ukfStable = stableUkfBenchmark;
comparison.ukfLegBias = legBiasUkfBenchmark;
comparison.config = config;
end

function options = defaultOptions(overrides)
options = struct();
options.model = buildOptModelCustom();
options.sampleTime = 0.01;
options.windowLength = 8;
options.smoke = false;
options.truth = struct( ...
    'bL', [0.8; -0.4; 0.6; -0.5; 0.3; -0.2] * 1e-3, ...
    'bAtt', zeros(3, 1), ...
    'ba', zeros(3, 1), ...
    'bg', zeros(3, 1), ...
    'dm', 0.35, ...
    'dc', [0.01; -0.008; 0.006], ...
    'kF', 1 + [0.04; -0.03; 0.02; -0.02; 0.01; -0.01]);
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('compareUkfAndFgMheEstimation:UnknownOverride', ...
            '未知对比配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end
end

function config = variantConfig(model, sampleTime, windowLength, variant)
config = makeStewartFgMheConfig(model, sampleTime, struct('windowLength', windowLength));
if variant == "fg_mhe_no_dynamics"
    config.estimate.payload = false;
    config.estimate.actuator = false;
elseif variant == "fg_mhe_with_dynamics"
    config.estimate.payload = true;
    config.estimate.actuator = false;
elseif variant == "fg_mhe_robust"
    config.estimate.payload = true;
    config.estimate.actuator = false;
    config.robust.enabled = true;
    config.robust.kernel = "huber";
elseif variant == "fg_mhe_actuator"
    config.estimate.payload = true;
    config.estimate.actuator = true;
end
end

function metrics = fgMetrics(series, truth)
estimate = series.finalEstimate;
qError = estimate.q - series.trajectory.qTrue(:, 1:size(estimate.q, 2));
metrics = struct();
metrics.positionRmse = sqrt(mean(qError(1:3, :).^2, 'all'));
metrics.positionMax = max(abs(qError(1:3, :)), [], 'all');
metrics.attitudeRmse = sqrt(mean(qError(4:6, :).^2, 'all'));
metrics.attitudeMax = max(abs(qError(4:6, :)), [], 'all');
metrics.legBiasRmse = sqrt(mean((estimate.bL(:) - truth.bL(:)).^2));
metrics.imuBiasError = norm([estimate.ba(:) - truth.ba(:); estimate.bg(:) - truth.bg(:)]);
metrics.payloadMassError = abs(estimate.dm - truth.dm);
metrics.payloadComError = norm(estimate.dc(:) - truth.dc(:));
metrics.actuatorGainError = sqrt(mean((estimate.kF(:) - truth.kF(:)).^2));
metrics.outlierSensitivity = metrics.positionRmse + metrics.legBiasRmse;
end

function ablation = buildAblationTable(metrics)
names = [
    "FG-MHE without dynamics factor"
    "FG-MHE with dynamics factor"
    "FG-MHE with robust kernel"
    "FG-MHE with actuator factor"
    "FG-MHE full method"
    ];
sourceMethods = [
    "fg_mhe_no_dynamics"
    "fg_mhe_with_dynamics"
    "fg_mhe_robust"
    "fg_mhe_actuator"
    "fg_mhe_actuator"
    ];
position_rmse = zeros(numel(names), 1);
attitude_rmse = zeros(numel(names), 1);
parameter_error = zeros(numel(names), 1);
outlier_sensitivity = zeros(numel(names), 1);
solve_time = zeros(numel(names), 1);
for index = 1:numel(names)
    row = find(string(metrics.method) == sourceMethods(index), 1);
    position_rmse(index) = metrics.position_rmse(row);
    attitude_rmse(index) = metrics.attitude_rmse(row);
    parameter_error(index) = metrics.leg_bias_rmse(row) + ...
        metrics.payload_mass_error(row) + metrics.actuator_gain_error(row);
    outlier_sensitivity(index) = metrics.outlier_sensitivity(row);
    solve_time(index) = metrics.mean_solve_time(row);
end
method = names;
ablation = table(method, position_rmse, attitude_rmse, parameter_error, ...
    outlier_sensitivity, solve_time);
end
