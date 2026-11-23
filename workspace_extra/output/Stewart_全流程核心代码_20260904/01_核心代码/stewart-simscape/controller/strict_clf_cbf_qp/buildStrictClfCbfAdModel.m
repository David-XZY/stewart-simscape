function ad = buildStrictClfCbfAdModel(model, scene, config)
% buildStrictClfCbfAdModel - Build exact CasADi maps used by strict QP.
%
% No finite-difference derivative is used here.  CasADi differentiates the
% smooth rigid-body, support-gap, and singularity expressions directly.

ensureCasadiOnPath();
import casadi.*

validateScene(scene);
q = SX.sym('q', 6, 1);
qd = SX.sym('qd', 6, 1);

[R, E, Edot] = zyxKinematics(q(4:6), qd(4:6));
[L, Jv, Jq, Jbar, rUpper, unitLeg] = stewartKinematics(q, R, E, model);

[drift, inputMap, H, rigidBias] = rigidBodyAffineDynamics( ...
    qd, R, E, Edot, Jv, model);

legSpeed = Jq * qd;
geometricLegBias = legAccelerationBias(qd, E, Edot, rUpper, unitLeg, ...
    L, legSpeed);
legAccelerationDrift = geometricLegBias + Jq * drift;
legAccelerationMap = Jq * inputMap;

dynamicsFunction = Function('strict_affine_dynamics', {q, qd}, ...
    {drift, inputMap, H, rigidBias, Jv, Jq, Jbar, L, legSpeed, ...
    geometricLegBias, legAccelerationDrift, legAccelerationMap});

[collisionGap, collisionGradient, collisionHessian, normalBanks] = ...
    collisionCertificates(q, R, scene, config.collision.smoothingEpsilon);
collisionFunction = Function('strict_collision_certificates', {q}, ...
    {collisionGap, collisionGradient, collisionHessian});

gram = Jbar * Jbar.';
sigmaLowerSquared = 1 / trace(solve(gram, SX.eye(6)));
singularityBarrier = sigmaLowerSquared - config.sigmaSafe^2;
[singularityHessian, singularityGradientColumn] = hessian(singularityBarrier, q);
singularityGradient = singularityGradientColumn.';
singularityFunction = Function('strict_singularity_barrier', {q}, ...
    {sigmaLowerSquared, singularityBarrier, singularityGradient, ...
    singularityHessian, Jbar});

ad = struct();
ad.dynamics = dynamicsFunction;
ad.collision = collisionFunction;
ad.singularity = singularityFunction;
ad.collisionNormalBanks = normalBanks;
ad.backend = "CasADi algorithmic differentiation";
ad.usesFiniteDifferences = false;
ad.smoothingEpsilon = config.collision.smoothingEpsilon;
end

function ensureCasadiOnPath()
try
    probe = casadi.SX.sym('strict_casadi_probe'); %#ok<NASGU>
    return;
catch
end

controllerDirectory = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(controllerDirectory);
candidates = dir(fullfile(projectRoot, 'lib', 'casadi-windows-matlabR*-v*'));
if isempty(candidates)
    error('buildStrictClfCbfAdModel:CasadiUnavailable', ...
        ['CasADi is required for exact barrier derivatives.  No ordinary ' ...
        'finite-difference fallback is permitted.']);
end
addpath(fullfile(candidates(1).folder, candidates(1).name));
try
    probe = casadi.SX.sym('strict_casadi_probe'); %#ok<NASGU>
catch exception
    error('buildStrictClfCbfAdModel:CasadiUnavailable', ...
        'CasADi could not be loaded: %s', exception.message);
end
end

function [R, E, Edot] = zyxKinematics(rpy, rpyRate)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
sr = sin(roll); cr = cos(roll);
sp = sin(pitch); cp = cos(pitch);
sy = sin(yaw); cy = cos(yaw);

R = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr; ...
     sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr; ...
     -sp,   cp*sr,          cp*cr];
E = [cy*cp, -sy, 0; ...
     sy*cp,  cy, 0; ...
     -sp,     0, 1];

pitchRate = rpyRate(2);
yawRate = rpyRate(3);
Edot = [ ...
    -cy*sp*pitchRate-sy*cp*yawRate, -cy*yawRate, 0; ...
    -sy*sp*pitchRate+cy*cp*yawRate, -sy*yawRate, 0; ...
    -cp*pitchRate,                    0,           0];
end

function [L, Jv, Jq, Jbar, rUpper, unitLeg] = stewartKinematics(q, R, E, model)
import casadi.*
if isfield(model, 'legMap')
    upperAnchors = model.B(:, model.legMap);
else
    upperAnchors = model.B;
end
rUpper = R * upperAnchors;
L = SX.zeros(6, 1);
unitLeg = SX.zeros(3, 6);
Jv = SX.zeros(6, 6);
for legIndex = 1:6
    legVector = q(1:3) + rUpper(:, legIndex) - model.A(:, legIndex);
    L(legIndex) = sqrt(legVector.' * legVector);
    unitLeg(:, legIndex) = legVector / L(legIndex);
    momentRow = cross3(rUpper(:, legIndex), unitLeg(:, legIndex));
    Jv(legIndex, :) = [unitLeg(:, legIndex).', momentRow.'];
end
Jq = Jv * blockDiagonalIdentityE(E);
characteristicLength = model.singularity.characteristicLength;
Jbar = Jv * diag([1, 1, 1, 1/characteristicLength, ...
    1/characteristicLength, 1/characteristicLength]);
end

function [drift, inputMap, H, bias] = rigidBodyAffineDynamics(qd, R, E, Edot, Jv, model)
mass = model.dynamics.totalMass;
centerOfMass = R * model.dynamics.comP;
inertiaWorld = R * model.dynamics.inertiaAtCOM_P * R.';
angularRate = E * qd(4:6);
angularAccelerationBias = Edot * qd(4:6);
centerSkew = skew3(centerOfMass);

H = [mass*eye(3), -mass*centerSkew*E; ...
     mass*centerSkew, (inertiaWorld-mass*centerSkew*centerSkew)*E];

if model.dynamics.includeGravity
    gravity = model.g;
else
    gravity = zeros(3, 1);
end
centerAccelerationBias = cross3(angularAccelerationBias, centerOfMass) + ...
    cross3(angularRate, cross3(angularRate, centerOfMass));
forceBias = mass * (centerAccelerationBias - gravity);
momentBias = inertiaWorld * angularAccelerationBias + ...
    cross3(angularRate, inertiaWorld * angularRate) + ...
    cross3(centerOfMass, forceBias);
bias = [forceBias; momentBias];

inputMap = solve(H, Jv.');
drift = solve(H, -bias);
end

function bias = legAccelerationBias(qd, E, Edot, rUpper, unitLeg, L, legSpeed)
import casadi.*
translationRate = qd(1:3);
angularRate = E * qd(4:6);
angularAccelerationBias = Edot * qd(4:6);
bias = SX.zeros(6, 1);
for legIndex = 1:6
    radius = rUpper(:, legIndex);
    upperVelocity = translationRate + cross3(angularRate, radius);
    upperAccelerationBias = cross3(angularAccelerationBias, radius) + ...
        cross3(angularRate, cross3(angularRate, radius));
    bias(legIndex) = unitLeg(:, legIndex).' * upperAccelerationBias + ...
        (upperVelocity.'*upperVelocity-legSpeed(legIndex)^2) / L(legIndex);
end
end

function [gaps, gradients, hessianStack, normalBanks] = collisionCertificates(q, R, scene, epsilon)
import casadi.*
cylinderCenter = q(1:3) + R * scene.objectCylinder.center_P;
cylinderAxis = R * scene.objectCylinder.axis_P;
cylinderHalfLength = 0.5 * scene.objectCylinder.length;
cylinderRadius = scene.objectCylinder.radius;

obstacles = [scene.hood.roof, scene.hood.leftSkirt, scene.hood.rightSkirt];
normalBanks = buildFixedNormalBanks(scene);
gaps = SX.zeros(3, 1);
gradients = SX.zeros(3, 6);
hessianStack = SX.zeros(18, 6);
for obstacleIndex = 1:3
    obstacle = obstacles(obstacleIndex);
    bank = normalBanks{obstacleIndex};
    candidateGaps = SX.zeros(size(bank, 2), 1);
    for normalIndex = 1:size(bank, 2)
        normal = bank(:, normalIndex);
        boxMinimum = normal.' * obstacle.center_S - ...
            sum(obstacle.halfSize(:) .* abs(obstacle.R_S.' * normal));
        axialProjection = normal.' * cylinderAxis;
        smoothAxialSupport = cylinderHalfLength * ...
            sqrt(axialProjection^2 + epsilon^2);
        smoothRadialSupport = cylinderRadius * ...
            sqrt(1-axialProjection^2 + epsilon^2);
        candidateGaps(normalIndex) = boxMinimum - normal.' * cylinderCenter - ...
            smoothAxialSupport - smoothRadialSupport;
    end
    gaps(obstacleIndex) = smoothMaximumLowerBound(candidateGaps, epsilon);
    [barrierHessian, barrierGradientColumn] = hessian(gaps(obstacleIndex), q);
    gradients(obstacleIndex, :) = barrierGradientColumn.';
    rows = (obstacleIndex-1)*6 + (1:6);
    hessianStack(rows, :) = barrierHessian;
end
end

function banks = buildFixedNormalBanks(scene)
% A fixed normal bank avoids the false infeasibility caused by one face
% normal while retaining a rigorous support-plane separation certificate.
baseDirections = [scene.box.R_S, -scene.box.R_S];
% Roof clearance has a prescribed physical meaning, so retain its inward
% lower-face normal.  Side skirts use richer fixed banks to avoid a false
% unsafe classification while preserving support-plane rigor.
banks = {scene.box.R_S(:, 3), baseDirections, baseDirections};

samplePoses = scene.q0(:);
if isfield(scene, 'qWaypoint') && isfield(scene, 'qGoal')
    tau = linspace(0, 1, 9);
    approach = scene.q0(:) * (1-tau) + scene.qWaypoint(:) * tau;
    insertion = scene.qWaypoint(:) * (1-tau(2:end)) + scene.qGoal(:) * tau(2:end);
    samplePoses = [approach, insertion];
end
if exist('evaluateCylinderBoxDistanceNumeric', 'file') == 2
    for sampleIndex = 1:size(samplePoses, 2)
        info = evaluateCylinderBoxDistanceNumeric(samplePoses(:, sampleIndex), scene);
        for obstacleIndex = 2:3
            banks{obstacleIndex}(:, end+1) = info.bestNormals(:, obstacleIndex);
        end
    end
end
for obstacleIndex = 1:3
    banks{obstacleIndex} = uniqueUnitDirections(banks{obstacleIndex});
end
end

function directions = uniqueUnitDirections(raw)
directions = zeros(3, 0);
for index = 1:size(raw, 2)
    candidate = raw(:, index);
    candidate = candidate / norm(candidate);
    if isempty(directions) || all(abs(directions.'*candidate-1) > 1e-10)
        directions(:, end+1) = candidate; %#ok<AGROW>
    end
end
end

function value = smoothMaximumLowerBound(values, epsilon)
% Pairwise m_e(a,b) <= max(a,b), hence the recursive value is a smooth,
% conservative lower bound of the best fixed-normal support gap.
value = values(1);
for index = 2:numel(values)
    difference = value - values(index);
    value = 0.5 * (value + values(index) + ...
        sqrt(difference^2 + epsilon^2) - epsilon);
end
end

function matrix = blockDiagonalIdentityE(E)
import casadi.*
matrix = SX.zeros(6, 6);
matrix(1:3, 1:3) = SX.eye(3);
matrix(4:6, 4:6) = E;
end

function value = cross3(left, right)
value = skew3(left) * right;
end

function matrix = skew3(vector)
matrix = [0, -vector(3), vector(2); ...
          vector(3), 0, -vector(1); ...
          -vector(2), vector(1), 0];
end

function validateScene(scene)
required = {'box', 'hood', 'objectCylinder', 'collision', 'phase'};
for index = 1:numel(required)
    if ~isfield(scene, required{index})
        error('buildStrictClfCbfAdModel:InvalidScene', ...
            'scene.%s is required.', required{index});
    end
end
end
