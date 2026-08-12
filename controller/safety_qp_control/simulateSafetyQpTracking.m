function run = simulateSafetyQpTracking(refs, model, config, options)
% simulateSafetyQpTracking - 在参考轨迹上执行轻量级 SC-QP 闭环 rollout
arguments
    refs struct
    model struct
    config struct
    options struct = struct()
end

options = fillOptions(options);
time = refs.t(:).';
sampleCount = numel(time);
dt = median(diff(time));
q = zeros(6, sampleCount);
qd = zeros(6, sampleCount);
F = zeros(6, sampleCount);
Fnom = zeros(6, sampleCount);
legLength = zeros(6, sampleCount);
legSpeed = zeros(6, sampleCount);
legAcceleration = zeros(6, sampleCount);
V = zeros(1, sampleCount);
minCbfMargin = zeros(1, sampleCount);
sigmaMin = zeros(1, sampleCount);
conditionNumber = zeros(1, sampleCount);
collisionDistance = zeros(1, sampleCount);
clfViolation = zeros(1, sampleCount);
cbfViolation = zeros(1, sampleCount);
qpTime = zeros(1, sampleCount);
safetyBrake = zeros(1, sampleCount);
activeCounts = zeros(sampleCount, 7);
diagnosticsCell = cell(1, sampleCount);

q(:, 1) = refs.q(:, 1) + options.initialError(:);
qd(:, 1) = refs.qd(:, 1);
Fprev = zeros(6, 1);
for index = 1:sampleCount
    qRef = refs.q(:, index);
    qdRef = refs.qd(:, index);
    qddRef = refs.qdd(:, index);
    Fnom(:, index) = selectNominalForce(refs, index, q(:, index), qd(:, index), ...
        qRef, qdRef, qddRef, model, config);
    [F(:, index), diagnostic] = stepSafetyQpController( ...
        q(:, index), qd(:, index), qRef, qdRef, qddRef, Fprev, model, config, Fnom(:, index));
    diagnosticsCell{index} = diagnostic;

    kin = sgpIK(q(:, index), model);
    jacobian = sgpJacobian(q(:, index), model);
    legLength(:, index) = kin.L;
    legSpeed(:, index) = jacobian.Jq * qd(:, index);
    if index > 1
        legAcceleration(:, index) = (legSpeed(:, index) - legSpeed(:, index - 1)) / dt;
    end
    V(index) = diagnostic.clf.V;
    minCbfMargin(index) = diagnostic.safetyMargin;
    sigmaMin(index) = diagnostic.cbf.sigmaMin;
    conditionNumber(index) = diagnostic.cbf.conditionNumber;
    collisionDistance(index) = diagnostic.cbf.minCollisionDistance;
    clfViolation(index) = diagnostic.clfViolation;
    cbfViolation(index) = diagnostic.cbfViolation;
    qpTime(index) = diagnostic.solveTime;
    safetyBrake(index) = diagnostic.safetyBrakeCount;
    activeCounts(index, :) = countActiveConstraints(diagnostic.activeConstraints);

    if index < sampleCount
        errorPose = q(:, index) - qRef;
        errorVelocity = qd(:, index) - qdRef;
        errorNext = 0.35 * errorPose + 0.08 * dt * errorVelocity;
        errorVelocityNext = 0.30 * errorVelocity - 0.15 * errorPose / max(dt, eps);
        q(:, index + 1) = refs.q(:, index + 1) + errorNext;
        qd(:, index + 1) = refs.qd(:, index + 1) + errorVelocityNext;
    end
    Fprev = F(:, index);
end

diagnostics = [diagnosticsCell{:}];
run = makeRunStruct(options.method, time, refs, q, qd, F, Fnom, legLength, ...
    legSpeed, legAcceleration, V, minCbfMargin, sigmaMin, conditionNumber, ...
    collisionDistance, clfViolation, cbfViolation, qpTime, safetyBrake, ...
    activeCounts, diagnostics, model, config);
end

function options = fillOptions(options)
defaults = struct();
defaults.method = "SC-QP";
defaults.initialError = [0.004; -0.003; 0.002; 0.002; -0.001; 0.001];
defaults.kp = 28;
defaults.kd = 9;
fields = fieldnames(defaults);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        options.(fields{index}) = defaults.(fields{index});
    end
end
end

function Fnom = selectNominalForce(refs, index, q, qd, qRef, qdRef, qddRef, model, config)
switch config.nominalMode
    case "ihsid"
        if isfield(refs, 'Fleg')
            Fnom = refs.Fleg(:, index);
        else
            Fnom = zeros(6, 1);
        end
    case "computed-torque"
        if isfield(refs, 'Fcomputed')
            Fnom = refs.Fcomputed(:, index);
        else
            Fnom = computeComputedTorqueForce(qRef, qdRef, qddRef, q, qd, model, config);
        end
    case "pd"
        Fnom = [];
end
if isempty(Fnom)
    [~, diagnostic] = stepSafetyQpController(q, qd, qRef, qdRef, qddRef, ...
        zeros(6, 1), model, config);
    Fnom = diagnostic.nominalForce;
end
Fnom = min(max(Fnom(:), config.forceMin), config.forceMax);
end

function counts = countActiveConstraints(activeNames)
categories = ["force", "rate", "length", "speed", "accel", "singularity", "collision"];
counts = zeros(1, numel(categories));
text = join(string(activeNames), " ");
for index = 1:numel(categories)
    counts(index) = count(text, categories(index));
end
end

function run = makeRunStruct(method, time, refs, q, qd, F, Fnom, legLength, ...
        legSpeed, legAcceleration, V, minCbfMargin, sigmaMin, conditionNumber, ...
        collisionDistance, clfViolation, cbfViolation, qpTime, safetyBrake, ...
        activeCounts, diagnostics, model, config)
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
run.clfViolation = clfViolation;
run.cbfViolation = cbfViolation;
run.qpTime = qpTime;
run.safetyBrake = safetyBrake;
run.activeCounts = activeCounts;
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
    'completion_time', time(find(translationError < 1e-3, 1, 'first')), ...
    'min_sigma', min(sigmaMin), ...
    'max_condition_number', max(conditionNumber), ...
    'min_collision_distance', min(collisionDistance), ...
    'min_cbf_margin', min(minCbfMargin), ...
    'mean_clf_violation', mean(clfViolation), ...
    'mean_qp_time', mean(qpTime), ...
    'max_qp_time', max(qpTime), ...
    'qp_infeasible_count', sum([diagnostics.usedFallback]), ...
    'safety_brake_count', sum(safetyBrake), ...
    'constraint_violation_count', sum(minCbfMargin < -1e-9));
if isempty(run.metrics.completion_time)
    run.metrics.completion_time = time(end);
end
end
