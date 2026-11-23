function [clf, cbf] = buildStrictClfCbfConstraints( ...
    q, qd, qRef, qdRef, qddRef, model, scene, config, evaluation)
% buildStrictClfCbfConstraints - Assemble one soft CLF and all hard CBFs.
arguments
    q double
    qd double
    qRef double
    qdRef double
    qddRef double
    model struct
    scene struct
    config struct
    evaluation struct = struct()
end

q = validatedSix(q, 'q');
qd = validatedSix(qd, 'qd');
qRef = validatedSix(qRef, 'qRef');
qdRef = validatedSix(qdRef, 'qdRef');
qddRef = validatedSix(qddRef, 'qddRef');
if isempty(fieldnames(evaluation))
    evaluation = evaluateStrictClfCbfAdModel(q, qd, model, scene, config);
end

errorState = [q-qRef; qd-qdRef];
P = config.clf.P;
V = errorState.' * P * errorState;
gradientV = 2 * errorState.' * P;
errorDrift = [qd-qdRef; evaluation.drift-qddRef];
errorInput = [zeros(6, 6); evaluation.inputMap];
clfDrift = gradientV * errorDrift;
clfInput = gradientV * errorInput;
clf = struct();
clf.A = clfInput;
clf.b = -config.clf.rate*V - clfDrift;
clf.V = V;
clf.drift = clfDrift;
clf.input = clfInput;
clf.error = errorState;

A = zeros(0, 6);
b = zeros(0, 1);
names = strings(0, 1);
stateNames = strings(0, 1);
stateMargins = zeros(0, 1);
psi1Names = strings(0, 1);
psi1Values = zeros(0, 1);

lengthAlpha1 = config.cbf.lengthAlpha1;
lengthAlpha2 = config.cbf.lengthAlpha2;
for legIndex = 1:6
    map = evaluation.legAccelerationMap(legIndex, :);
    accelDrift = evaluation.legAccelerationDrift(legIndex);
    speed = evaluation.legSpeed(legIndex);

    hLower = evaluation.legLength(legIndex) - ...
        (config.lengthMin(legIndex)+config.lengthMargin);
    hdotLower = speed;
    hocbfLowerDrift = accelDrift + ...
        (lengthAlpha1+lengthAlpha2)*hdotLower + ...
        lengthAlpha1*lengthAlpha2*hLower;
    [A, b, names] = appendInequality(A, b, names, -map, ...
        hocbfLowerDrift, "length_lower_"+legIndex);
    [stateNames, stateMargins] = appendState(stateNames, stateMargins, ...
        "length_lower_"+legIndex, hLower);
    [psi1Names, psi1Values] = appendState(psi1Names, psi1Values, ...
        "length_lower_"+legIndex, hdotLower+lengthAlpha1*hLower);

    hUpper = (config.lengthMax(legIndex)-config.lengthMargin) - ...
        evaluation.legLength(legIndex);
    hdotUpper = -speed;
    hocbfUpperDrift = -accelDrift + ...
        (lengthAlpha1+lengthAlpha2)*hdotUpper + ...
        lengthAlpha1*lengthAlpha2*hUpper;
    [A, b, names] = appendInequality(A, b, names, map, ...
        hocbfUpperDrift, "length_upper_"+legIndex);
    [stateNames, stateMargins] = appendState(stateNames, stateMargins, ...
        "length_upper_"+legIndex, hUpper);
    [psi1Names, psi1Values] = appendState(psi1Names, psi1Values, ...
        "length_upper_"+legIndex, hdotUpper+lengthAlpha1*hUpper);
end

speedAlpha = config.cbf.speedAlpha;
for legIndex = 1:6
    map = evaluation.legAccelerationMap(legIndex, :);
    accelDrift = evaluation.legAccelerationDrift(legIndex);
    speed = evaluation.legSpeed(legIndex);
    limit = config.legSpeedLimit(legIndex);

    hUpper = limit-speed;
    [A, b, names] = appendInequality(A, b, names, map, ...
        -accelDrift+speedAlpha*hUpper, "speed_upper_"+legIndex);
    [stateNames, stateMargins] = appendState(stateNames, stateMargins, ...
        "speed_upper_"+legIndex, hUpper);

    hLower = limit+speed;
    [A, b, names] = appendInequality(A, b, names, -map, ...
        accelDrift+speedAlpha*hLower, "speed_lower_"+legIndex);
    [stateNames, stateMargins] = appendState(stateNames, stateMargins, ...
        "speed_lower_"+legIndex, hLower);
end

for legIndex = 1:6
    map = evaluation.legAccelerationMap(legIndex, :);
    accelDrift = evaluation.legAccelerationDrift(legIndex);
    limit = config.legAccelerationLimit(legIndex);
    [A, b, names] = appendInequality(A, b, names, map, ...
        limit-accelDrift, "acceleration_upper_"+legIndex);
    [A, b, names] = appendInequality(A, b, names, -map, ...
        limit+accelDrift, "acceleration_lower_"+legIndex);
end

roofThreshold = evaluateStrictRoofThreshold(config.time, scene, config);
sideDistance = scene.collision.safeDistance;
distanceThreshold = [roofThreshold.value; sideDistance; sideDistance];
distanceThresholdRate = [roofThreshold.rate; 0; 0];
distanceThresholdAcceleration = [roofThreshold.acceleration; 0; 0];
collisionNames = ["collision_roof"; "collision_left"; "collision_right"];
collisionAlpha1 = config.cbf.collisionAlpha1;
collisionAlpha2 = config.cbf.collisionAlpha2;
for obstacleIndex = 1:3
    gradient = evaluation.collisionGradient(obstacleIndex, :);
    hessian = evaluation.collisionHessian(:, :, obstacleIndex);
    h = evaluation.collisionGap(obstacleIndex)-distanceThreshold(obstacleIndex);
    hdot = gradient*qd-distanceThresholdRate(obstacleIndex);
    relativeAccelerationDrift = gradient*evaluation.drift + ...
        qd.'*hessian*qd-distanceThresholdAcceleration(obstacleIndex);
    relativeAccelerationMap = gradient*evaluation.inputMap;
    hocbfDrift = relativeAccelerationDrift + ...
        (collisionAlpha1+collisionAlpha2)*hdot + ...
        collisionAlpha1*collisionAlpha2*h;
    [A, b, names] = appendInequality(A, b, names, ...
        -relativeAccelerationMap, hocbfDrift, collisionNames(obstacleIndex));
    [stateNames, stateMargins] = appendState(stateNames, stateMargins, ...
        collisionNames(obstacleIndex), h);
    [psi1Names, psi1Values] = appendState(psi1Names, psi1Values, ...
        collisionNames(obstacleIndex), hdot+collisionAlpha1*h);
end

singularityAlpha1 = config.cbf.singularityAlpha1;
singularityAlpha2 = config.cbf.singularityAlpha2;
h = evaluation.singularityBarrier;
gradient = evaluation.singularityGradient;
hessian = evaluation.singularityHessian;
hdot = gradient*qd;
relativeAccelerationDrift = gradient*evaluation.drift + qd.'*hessian*qd;
relativeAccelerationMap = gradient*evaluation.inputMap;
hocbfDrift = relativeAccelerationDrift + ...
    (singularityAlpha1+singularityAlpha2)*hdot + ...
    singularityAlpha1*singularityAlpha2*h;
[A, b, names] = appendInequality(A, b, names, ...
    -relativeAccelerationMap, hocbfDrift, "singularity");
[stateNames, stateMargins] = appendState(stateNames, stateMargins, ...
    "singularity", h);
[psi1Names, psi1Values] = appendState(psi1Names, psi1Values, ...
    "singularity", hdot+singularityAlpha1*h);

cbf = struct();
cbf.A = A;
cbf.b = b;
cbf.names = names;
cbf.stateNames = stateNames;
cbf.stateMargins = stateMargins;
cbf.psi1Names = psi1Names;
cbf.psi1 = psi1Values;
cbf.minimumStateMargin = min(stateMargins);
cbf.minimumPsi1 = min(psi1Values);
cbf.roofThreshold = roofThreshold;
cbf.distanceThreshold = distanceThreshold;
cbf.evaluation = evaluation;
cbf.hasSlack = false;
end

function value = validatedSix(value, name)
value = value(:);
validateattributes(value, {'double'}, {'real', 'finite', 'size', [6, 1]}, ...
    mfilename, name);
end

function [A, b, names] = appendInequality(A, b, names, row, bound, name)
A(end+1, :) = row;
b(end+1, 1) = bound;
names(end+1, 1) = string(name);
end

function [names, values] = appendState(names, values, name, value)
names(end+1, 1) = string(name);
values(end+1, 1) = value;
end
