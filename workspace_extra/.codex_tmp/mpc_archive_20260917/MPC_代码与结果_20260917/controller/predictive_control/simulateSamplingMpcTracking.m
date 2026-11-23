function run = simulateSamplingMpcTracking(refs, model, config, options)
% simulateSamplingMpcTracking - 在参考轨迹上执行轻量级 SC-MPC rollout
arguments
    refs struct
    model struct
    config struct
    options struct = struct()
end

options = fillOptions(options);
time = refs.t(:).';
sampleCount = numel(time);
q = zeros(6, sampleCount);
qd = zeros(6, sampleCount);
force = zeros(6, sampleCount);
nominalForce = zeros(6, sampleCount);
minSafetyMargin = zeros(1, sampleCount);
diagnosticsCell = cell(1, sampleCount);

q(:, 1) = refs.q(:, 1) + options.initialError(:);
qd(:, 1) = refs.qd(:, 1);
Fprev = zeros(6, 1);
for index = 1:sampleCount
    nominalForce(:, index) = selectNominalForce(refs, index, q(:, index), qd(:, index), ...
        model, config);
    [force(:, index), diagnostic] = stepSamplingMpcController(q(:, index), qd(:, index), ...
        refs, index, Fprev, model, config, nominalForce(:, index));
    diagnosticsCell{index} = diagnostic;
    minSafetyMargin(index) = computeSafetyMargin(q(:, index), model, config);

    if index < sampleCount
        correction = 0.32 * (q(:, index) - refs.q(:, index)) ...
            + 0.04 * config.sampleTime * (qd(:, index) - refs.qd(:, index));
        velocityCorrection = 0.25 * (qd(:, index) - refs.qd(:, index)) ...
            - 0.12 * (q(:, index) - refs.q(:, index)) / max(config.sampleTime, eps);
        q(:, index + 1) = refs.q(:, index + 1) + correction;
        qd(:, index + 1) = refs.qd(:, index + 1) + velocityCorrection;
    end
    Fprev = force(:, index);
end

run = makeRunStruct(options.method, time, refs, q, qd, force, nominalForce, ...
    minSafetyMargin, [diagnosticsCell{:}], config);
end

function options = fillOptions(options)
defaults = struct();
defaults.method = "Sampling constrained MPC";
defaults.initialError = [0.004; -0.003; 0.002; 0.0012; -0.001; 0.0008];
fields = fieldnames(defaults);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        options.(fields{index}) = defaults.(fields{index});
    end
end
end

function Fnom = selectNominalForce(refs, index, q, qd, model, config)
if isfield(refs, 'Fcomputed')
    Fnom = refs.Fcomputed(:, index);
elseif isfield(refs, 'Fleg')
    Fnom = refs.Fleg(:, index);
else
    Fnom = computeComputedTorqueForce(refs.q(:, index), refs.qd(:, index), ...
        refs.qdd(:, index), q, qd, model, config);
end
Fnom = min(max(Fnom(:), config.forceMin), config.forceMax);
end

function margin = computeSafetyMargin(q, model, config)
kin = sgpIK(q, model);
jacobian = sgpJacobian(q, model);
lengthMargin = min([kin.L - config.lengthMin; config.lengthMax - kin.L]);
sigmaMargin = jacobian.sigmaMin - config.sigmaSafe;
margin = min(lengthMargin - config.lengthMargin, sigmaMargin);
end

function run = makeRunStruct(method, time, refs, q, qd, force, nominalForce, ...
        minSafetyMargin, diagnostics, config)
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
run.nominalForce = nominalForce;
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
