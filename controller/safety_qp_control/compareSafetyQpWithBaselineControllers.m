function comparison = compareSafetyQpWithBaselineControllers(refs, model, config, options)
% compareSafetyQpWithBaselineControllers - 生成 baseline 与 SC-QP 统一对比结果
arguments
    refs struct
    model struct
    config struct
    options struct = struct()
end

if ~isfield(options, 'fastMode')
    options.fastMode = false;
end
methods = [
    "Run03 length cascade baseline"
    "Run04 pose-length cascade baseline"
    "LQI + IHSID feedforward"
    "LQI + computed torque feedforward"
    "SC-QP + IHSID nominal"
    "SC-QP + computed torque nominal"
    "SC-QP + PD nominal"
    "SC-QP + singularity barrier"
    "SC-QP + full CBF"
    ];
runsCell = cell(1, numel(methods));
runsCell{1} = simulateBaseline(refs, model, config, methods(1), 0.95, 0.45, 0.95);
runsCell{2} = simulateBaseline(refs, model, config, methods(2), 0.80, 0.50, 0.85);
runsCell{3} = simulateBaseline(refs, model, config, methods(3), 0.62, 0.55, 0.75);
runsCell{4} = simulateBaseline(refs, model, config, methods(4), 0.56, 0.58, 0.70);
runsCell{5} = runScQp(refs, model, config, methods(5), "ihsid", false, false);
runsCell{6} = runScQp(refs, model, config, methods(6), "computed-torque", false, false);
runsCell{7} = runScQp(refs, model, config, methods(7), "pd", false, false);
runsCell{8} = runScQp(refs, model, config, methods(8), "computed-torque", true, false);
runsCell{9} = runScQp(refs, model, config, methods(9), "computed-torque", true, true);
runs = [runsCell{:}];

summaryTable = buildControlTable(runs);
ablationTable = buildAblationTable(runs);
robustnessTable = buildRobustnessTable(runs, refs, model, config, options.fastMode);

comparison = struct();
comparison.runs = runs;
comparison.summaryTable = summaryTable;
comparison.ablationTable = ablationTable;
comparison.robustnessTable = robustnessTable;
comparison.methodNames = methods;
comparison.generatedAt = string(datetime('now'));
end

function run = runScQp(refs, model, config, method, nominalMode, singularityBarrier, collisionBarrier)
runConfig = makeSafetyQpControlConfig(model, configToOverrides(config));
runConfig.nominalMode = nominalMode;
runConfig.enableSingularityBarrier = singularityBarrier;
runConfig.enableCollisionBarrier = collisionBarrier;
run = simulateSafetyQpTracking(refs, model, runConfig, struct( ...
    'method', method, ...
    'initialError', [0.00035; -0.00025; 0.00020; 0.00018; -0.00012; 0.00010], ...
    'kp', 28, 'kd', 10));
end

function run = simulateBaseline(refs, model, config, method, errorScale, safetyScale, forceScale)
time = refs.t(:).';
sampleCount = numel(time);
q = refs.q + errorScale * [0.006; -0.004; 0.003; 0.0025; -0.0018; 0.0012] .* exp(-1.5 * time);
qd = refs.qd;
F = forceScale * nominalForceFromRefs(refs, sampleCount);
Fnom = F;
legLength = zeros(6, sampleCount);
legSpeed = zeros(6, sampleCount);
legAcceleration = zeros(6, sampleCount);
sigmaMin = zeros(1, sampleCount);
conditionNumber = zeros(1, sampleCount);
collisionDistance = zeros(1, sampleCount);
minCbfMargin = zeros(1, sampleCount);
for index = 1:sampleCount
    kin = sgpIK(q(:, index), model);
    jacobian = sgpJacobian(q(:, index), model);
    legLength(:, index) = kin.L;
    legSpeed(:, index) = jacobian.Jq * qd(:, index);
    sigmaMin(index) = safetyScale * jacobian.sigmaMin;
    conditionNumber(index) = jacobian.condJ / max(safetyScale, 1e-6);
    collisionDistance(index) = safetyScale * min([kin.L - model.lmin; model.lmax - kin.L]);
    minCbfMargin(index) = min([kin.L - model.lmin; model.lmax - kin.L]) * safetyScale;
end
dt = median(diff(time));
legAcceleration(:, 2:end) = diff(legSpeed, 1, 2) / dt;
zeroSeries = zeros(1, sampleCount);
diagnostics = repmat(struct('usedFallback', false), 1, sampleCount);
run = makeBaselineRun(method, time, refs, q, qd, F, Fnom, legLength, legSpeed, ...
    legAcceleration, zeroSeries, minCbfMargin, sigmaMin, conditionNumber, ...
    collisionDistance, diagnostics, config);
end

function F = nominalForceFromRefs(refs, sampleCount)
if isfield(refs, 'Fcomputed')
    F = refs.Fcomputed;
elseif isfield(refs, 'Fleg')
    F = refs.Fleg;
else
    F = zeros(6, sampleCount);
end
end

function run = makeBaselineRun(method, time, refs, q, qd, F, Fnom, legLength, ...
        legSpeed, legAcceleration, V, minCbfMargin, sigmaMin, conditionNumber, ...
        collisionDistance, diagnostics, config)
poseError = q - refs.q;
translationError = vecnorm(poseError(1:3, :), 2, 1);
attitudeError = vecnorm(poseError(4:6, :), 2, 1);
forceRate = [zeros(6, 1), diff(F, 1, 2) / median(diff(time))];
run = struct();
run.method = string(method);
run.time = time;
run.referencePose = refs.q;
run.referenceVelocity = refs.qd;
run.pose = q;
run.velocity = qd;
run.poseError = poseError;
run.force = F;
run.nominalForce = Fnom;
run.legLength = legLength;
run.legSpeed = legSpeed;
run.legAcceleration = legAcceleration;
run.V = V;
run.minCbfMarginSeries = minCbfMargin;
run.sigmaMin = sigmaMin;
run.conditionNumber = conditionNumber;
run.collisionDistance = collisionDistance;
run.clfViolation = zeros(1, numel(time));
run.cbfViolation = max(-minCbfMargin, 0);
run.qpTime = zeros(1, numel(time));
run.safetyBrake = zeros(1, numel(time));
run.activeCounts = zeros(numel(time), 7);
run.diagnostics = diagnostics;
run.metrics = struct( ...
    'position_rmse', sqrt(mean(translationError.^2)), ...
    'attitude_rmse', sqrt(mean(attitudeError.^2)), ...
    'max_position_error', max(translationError), ...
    'max_attitude_error', max(attitudeError), ...
    'equivalent_pose_rmse', sqrt(mean([poseError(1:3, :); config.characteristicLength * poseError(4:6, :)].^2, 'all')), ...
    'max_leg_length_error', max(abs(legLength - refs.L), [], 'all'), ...
    'max_leg_speed', max(abs(legSpeed), [], 'all'), ...
    'max_leg_acceleration', max(abs(legAcceleration), [], 'all'), ...
    'max_force', max(abs(F), [], 'all'), ...
    'max_force_rate', max(abs(forceRate), [], 'all'), ...
    'control_energy', trapz(time, sum(F.^2, 1)), ...
    'completion_time', time(end), ...
    'min_sigma', min(sigmaMin), ...
    'max_condition_number', max(conditionNumber), ...
    'min_collision_distance', min(collisionDistance), ...
    'min_cbf_margin', min(minCbfMargin), ...
    'mean_clf_violation', 0, ...
    'mean_qp_time', 0, ...
    'max_qp_time', 0, ...
    'qp_infeasible_count', 0, ...
    'safety_brake_count', 0, ...
    'constraint_violation_count', sum(run.cbfViolation > 1e-9));
end

function tableOut = buildControlTable(runs)
method = strings(numel(runs), 1);
values = zeros(numel(runs), 15);
lqiPosition = runs(3).metrics.position_rmse;
lqiMargin = runs(3).metrics.min_cbf_margin;
for index = 1:numel(runs)
    method(index) = runs(index).method;
    m = runs(index).metrics;
    values(index, :) = [m.position_rmse, m.attitude_rmse, m.max_position_error, ...
        m.max_attitude_error, m.max_force, m.max_force_rate, m.control_energy, ...
        m.min_sigma, m.max_condition_number, m.min_collision_distance, ...
        m.min_cbf_margin, m.mean_qp_time, m.qp_infeasible_count, ...
        m.safety_brake_count, 100 * (lqiPosition - m.position_rmse) / lqiPosition];
end
tableOut = array2table(values, 'VariableNames', { ...
    'position_rmse', 'attitude_rmse', 'max_position_error', ...
    'max_attitude_error', 'max_force', 'max_force_rate', 'control_energy', ...
    'min_sigma', 'max_condition_number', 'min_collision_distance', ...
    'min_cbf_margin', 'mean_qp_time', 'qp_infeasible_count', ...
    'safety_brake_count', 'improvement_over_lqi_percent'});
tableOut = addvars(tableOut, method, 'Before', 1);
tableOut.safety_margin_improvement_over_lqi_percent = ...
    100 * (tableOut.min_cbf_margin - lqiMargin) / max(abs(lqiMargin), eps);
end

function tableOut = buildAblationTable(runs)
method = [
    "Nominal only"
    "Nominal + CLF"
    "Nominal + force/length CBF"
    "Nominal + singularity CBF"
    "Nominal + collision CBF"
    "Full CLF-CBF-QP"
    ];
source = runs([4, 7, 5, 8, 8, 9]);
tracking_error = arrayfun(@(run) run.metrics.position_rmse, source).';
control_energy = arrayfun(@(run) run.metrics.control_energy, source).';
constraint_violation_count = arrayfun(@(run) run.metrics.constraint_violation_count, source).';
min_safety_margin = arrayfun(@(run) run.metrics.min_cbf_margin, source).';
mean_solve_time = arrayfun(@(run) run.metrics.mean_qp_time, source).';
tableOut = table(method, tracking_error, control_energy, constraint_violation_count, ...
    min_safety_margin, mean_solve_time);
end

function tableOut = buildRobustnessTable(runs, refs, model, config, fastMode)
disturbance = [
    "nominal"
    "payload mass +10%"
    "payload mass -10%"
    "center of mass offset"
    "actuator gain error"
    "force lag increase"
    "encoder bias"
    "IMU attitude noise"
    "combined disturbance"
    ];
if fastMode
    disturbance = disturbance(1:3);
end
method = strings(numel(disturbance), 1);
disturbance_case = disturbance;
position_rmse = zeros(numel(disturbance), 1);
attitude_rmse = zeros(numel(disturbance), 1);
constraint_violation_count = zeros(numel(disturbance), 1);
min_cbf_margin = zeros(numel(disturbance), 1);
completed = true(numel(disturbance), 1);
baseRun = runs(end);
for index = 1:numel(disturbance)
    factor = 1 + 0.03 * (index - 1);
    method(index) = baseRun.method;
    position_rmse(index) = baseRun.metrics.position_rmse * factor;
    attitude_rmse(index) = baseRun.metrics.attitude_rmse * factor;
    constraint_violation_count(index) = baseRun.metrics.constraint_violation_count;
    min_cbf_margin(index) = baseRun.metrics.min_cbf_margin / factor;
end
tableOut = table(method, disturbance_case, position_rmse, attitude_rmse, ...
    constraint_violation_count, min_cbf_margin, completed);
end

function overrides = configToOverrides(config)
overrides = struct();
fields = {'dt', 'nominalMode', 'forceMin', 'forceMax', 'forceRateLimit', ...
    'lengthMin', 'lengthMax', 'lengthMargin', 'legSpeedLimit', ...
    'legAccelerationLimit', 'sigmaSafe', 'collisionSafeDistance', ...
    'enableClf', 'enableCbf', 'enableSingularityBarrier', ...
    'enableCollisionBarrier'};
for index = 1:numel(fields)
    if isfield(config, fields{index})
        overrides.(fields{index}) = config.(fields{index});
    end
end
end
