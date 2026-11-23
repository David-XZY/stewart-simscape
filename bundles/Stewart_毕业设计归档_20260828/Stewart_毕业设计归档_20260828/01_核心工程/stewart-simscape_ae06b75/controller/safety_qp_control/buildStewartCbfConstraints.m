function cbf = buildStewartCbfConstraints(q, qd, Fprev, model, config)
% buildStewartCbfConstraints - 构造 Stewart 平台 barrier-like 安全约束
arguments
    q double
    qd double
    Fprev double
    model struct
    config struct
end

q = q(:);
qd = qd(:);
Fprev = Fprev(:);
validateattributes(q, {'double'}, {'numel', 6, 'finite'});
validateattributes(qd, {'double'}, {'numel', 6, 'finite'});
validateattributes(Fprev, {'double'}, {'numel', 6, 'finite'});

kin = sgpIK(q, model);
jacobian = sgpJacobian(q, model);
Ld = jacobian.Jq * qd;
accelMap = platformAccelerationMap(q, model, config);
legAccelMap = jacobian.Jq * accelMap;
rateStep = config.forceRateLimit(:) * config.dt;

A = [];
b = [];
names = strings(0, 1);
margins = [];

[A, b, names, margins] = appendBound(A, b, names, margins, eye(6), ...
    config.forceMax, "force_upper", config.forceMax - Fprev);
[A, b, names, margins] = appendBound(A, b, names, margins, -eye(6), ...
    -config.forceMin, "force_lower", Fprev - config.forceMin);
[A, b, names, margins] = appendBound(A, b, names, margins, eye(6), ...
    Fprev + rateStep, "force_rate_upper", rateStep);
[A, b, names, margins] = appendBound(A, b, names, margins, -eye(6), ...
    -Fprev + rateStep, "force_rate_lower", rateStep);

lengthUpperMargin = config.lengthMax - config.lengthMargin - kin.L;
lengthLowerMargin = kin.L - (config.lengthMin + config.lengthMargin);
[A, b, names, margins] = appendStateBarrier(A, b, names, margins, ...
    lengthUpperMargin, "length_upper");
[A, b, names, margins] = appendStateBarrier(A, b, names, margins, ...
    lengthLowerMargin, "length_lower");

speedUpperMargin = config.legSpeedLimit - Ld;
speedLowerMargin = config.legSpeedLimit + Ld;
[A, b, names, margins] = appendStateBarrier(A, b, names, margins, ...
    speedUpperMargin, "leg_speed_upper");
[A, b, names, margins] = appendStateBarrier(A, b, names, margins, ...
    speedLowerMargin, "leg_speed_lower");

[A, b, names, margins] = appendBound(A, b, names, margins, legAccelMap, ...
    config.legAccelerationLimit, "leg_accel_upper", config.legAccelerationLimit);
[A, b, names, margins] = appendBound(A, b, names, margins, -legAccelMap, ...
    config.legAccelerationLimit, "leg_accel_lower", config.legAccelerationLimit);

singularityMargin = jacobian.sigmaMin - config.sigmaSafe;
if config.enableSingularityBarrier
    [A, b, names, margins] = appendStateBarrier(A, b, names, margins, ...
        singularityMargin, "singularity_barrier");
end

collisionDistance = estimateCollisionDistance(q, model);
collisionMargin = collisionDistance - config.collisionSafeDistance;
if config.enableCollisionBarrier
    [A, b, names, margins] = appendStateBarrier(A, b, names, margins, ...
        collisionMargin, "collision_barrier");
end

cbf = struct();
cbf.A = A;
cbf.b = b;
cbf.names = names;
cbf.margin = margins(:);
cbf.minMargin = min(margins);
cbf.sigmaMin = jacobian.sigmaMin;
cbf.conditionNumber = jacobian.condJ;
cbf.minCollisionDistance = collisionDistance;
cbf.legLength = kin.L;
cbf.legSpeed = Ld;
cbf.legAccelerationMap = legAccelMap;
end

function [A, b, names, margins] = appendBound(A, b, names, margins, Ai, bi, name, margin)
A = [A; Ai]; %#ok<AGROW>
b = [b; bi(:)]; %#ok<AGROW>
names = [names; repmat(string(name), size(Ai, 1), 1)]; %#ok<AGROW>
margins = [margins; margin(:)]; %#ok<AGROW>
end

function [A, b, names, margins] = appendStateBarrier(A, b, names, margins, margin, name)
margin = margin(:);
A = [A; zeros(numel(margin), 6)]; %#ok<AGROW>
b = [b; max(margin, -abs(margin))]; %#ok<AGROW>
names = [names; repmat(string(name), numel(margin), 1)]; %#ok<AGROW>
margins = [margins; margin]; %#ok<AGROW>
end

function accelMap = platformAccelerationMap(q, model, config)
jacobian = sgpJacobian(q, model);
massMatrix = diag([ ...
    model.dynamics.totalMass * ones(1, 3), ...
    max(diag(model.dynamics.inertiaAtCOM_P))./max(config.characteristicLength, eps)^2 * ones(1, 3)]);
accelMap = massMatrix \ jacobian.Jv.';
end

function distance = estimateCollisionDistance(q, model)
kin = sgpIK(q, model);
workspaceMargin = min([kin.L - model.lmin; model.lmax - kin.L]);
heightMargin = max(q(3) - 0.2, 0);
distance = min([workspaceMargin; heightMargin]);
end
