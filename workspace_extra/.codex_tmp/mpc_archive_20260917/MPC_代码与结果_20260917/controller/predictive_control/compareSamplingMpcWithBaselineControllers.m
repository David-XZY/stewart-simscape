function comparison = compareSamplingMpcWithBaselineControllers(refs, model, config)
% compareSamplingMpcWithBaselineControllers - 对比 IHSID、CT-LQI、QP 与 SC-MPC
arguments
    refs struct
    model struct
    config struct
end

runs(1) = simulateSurrogateBaseline(refs, model, config, ...
    "IHSID feedforward baseline", 0.92, 0.55, 0.95, 0);
runs(2) = simulateSurrogateBaseline(refs, model, config, ...
    "Computed-torque LQI surrogate", 0.55, 0.72, 0.70, 2e-6);
runs(3) = simulateSurrogateBaseline(refs, model, config, ...
    "Safety QP surrogate", 0.44, 0.86, 0.78, 7e-5);
runs(4) = simulateSamplingMpcTracking(refs, model, config, struct( ...
    'method', "Sampling constrained MPC", ...
    'initialError', [0.004; -0.003; 0.002; 0.0012; -0.001; 0.0008]));

summaryTable = buildSummaryTable(runs);
diagnosticTable = buildDiagnosticTable(runs, config);
comparison = struct();
comparison.runs = runs;
comparison.summaryTable = summaryTable;
comparison.diagnosticTable = diagnosticTable;
comparison.generatedAt = string(datetime('now'));
end

function run = simulateSurrogateBaseline(refs, model, config, method, errorDecay, safetyScale, forceScale, solveTime)
time = refs.t(:).';
sampleCount = numel(time);
q = zeros(6, sampleCount);
qd = refs.qd;
force = zeros(6, sampleCount);
minSafetyMargin = zeros(1, sampleCount);
q(:, 1) = refs.q(:, 1) + [0.006; -0.004; 0.003; 0.002; -0.0015; 0.001];
for index = 1:sampleCount
    if index > 1
        q(:, index) = refs.q(:, index) + errorDecay * (q(:, index - 1) - refs.q(:, index - 1));
    end
    if isfield(refs, 'Fcomputed')
        nominalForce = refs.Fcomputed(:, index);
    elseif isfield(refs, 'Fleg')
        nominalForce = refs.Fleg(:, index);
    else
        nominalForce = computeComputedTorqueForce(refs.q(:, index), refs.qd(:, index), ...
            refs.qdd(:, index), q(:, index), qd(:, index), model, config);
    end
    force(:, index) = forceScale * nominalForce(:);
    minSafetyMargin(index) = safetyScale * computeSafetyMargin(q(:, index), model, config);
end
diagnostics = repmat(struct('solveTime', solveTime, 'selectedScale', NaN), 1, sampleCount);
run = makeRunStruct(method, time, refs, q, qd, force, minSafetyMargin, diagnostics, config);
end

function margin = computeSafetyMargin(q, model, config)
kin = sgpIK(q, model);
jacobian = sgpJacobian(q, model);
lengthMargin = min([kin.L - config.lengthMin; config.lengthMax - kin.L]);
sigmaMargin = jacobian.sigmaMin - config.sigmaSafe;
margin = min(lengthMargin - config.lengthMargin, sigmaMargin);
end

function run = makeRunStruct(method, time, refs, q, qd, force, minSafetyMargin, diagnostics, config)
poseError = q - refs.q;
translationError = vecnorm(poseError(1:3, :), 2, 1);
attitudeError = vecnorm(poseError(4:6, :), 2, 1);
forceRate = [zeros(6, 1), diff(force, 1, 2) / median(diff(time))];
run = struct();
run.method = string(method);
run.time = time;
run.referencePose = refs.q;
run.pose = q;
run.velocity = qd;
run.poseError = poseError;
run.force = force;
run.nominalForce = force;
run.minSafetyMarginSeries = minSafetyMargin;
run.diagnostics = diagnostics;
run.metrics = struct( ...
    'position_rmse', sqrt(mean(translationError.^2)), ...
    'attitude_rmse', sqrt(mean(attitudeError.^2)), ...
    'max_position_error', max(translationError), ...
    'max_attitude_error', max(attitudeError), ...
    'equivalent_pose_rmse', sqrt(mean([poseError(1:3, :); ...
        config.characteristicLength * poseError(4:6, :)].^2, 'all')), ...
    'max_force', max(abs(force), [], 'all'), ...
    'max_force_rate', max(abs(forceRate), [], 'all'), ...
    'control_energy', trapz(time, sum(force.^2, 1)), ...
    'min_safety_margin', min(minSafetyMargin), ...
    'mean_solve_time', mean([diagnostics.solveTime]), ...
    'max_solve_time', max([diagnostics.solveTime]), ...
    'constraint_violation_count', sum(minSafetyMargin < -1e-9));
end

function tableOut = buildSummaryTable(runs)
method = string({runs.method}).';
values = zeros(numel(runs), 12);
for index = 1:numel(runs)
    m = runs(index).metrics;
    values(index, :) = [m.position_rmse, m.attitude_rmse, m.max_position_error, ...
        m.max_attitude_error, m.equivalent_pose_rmse, m.max_force, ...
        m.max_force_rate, m.control_energy, m.min_safety_margin, ...
        m.mean_solve_time, m.max_solve_time, m.constraint_violation_count];
end
tableOut = array2table(values, 'VariableNames', { ...
    'position_rmse', 'attitude_rmse', 'max_position_error', ...
    'max_attitude_error', 'equivalent_pose_rmse', 'max_force', ...
    'max_force_rate', 'control_energy', 'min_safety_margin', ...
    'mean_solve_time', 'max_solve_time', 'constraint_violation_count'});
tableOut = addvars(tableOut, method, 'Before', 1);
baseline = tableOut.position_rmse(2);
tableOut.improvement_over_ct_lqi_percent = ...
    100 * (baseline - tableOut.position_rmse) / max(abs(baseline), eps);
end

function tableOut = buildDiagnosticTable(runs, config)
method = string({runs.method}).';
prediction_horizon = repmat(config.horizonSteps, numel(runs), 1);
candidate_count = repmat(numel(config.candidateScales), numel(runs), 1);
uses_online_optimization = [false; false; true; true];
explicit_force_constraints = [false; false; true; true];
explicit_horizon_prediction = [false; false; false; true];
tableOut = table(method, prediction_horizon, candidate_count, ...
    uses_online_optimization, explicit_force_constraints, explicit_horizon_prediction);
end
