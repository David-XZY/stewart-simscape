function report = diagnoseFatropNativeConstraintGroups(z, model, scene, disc, options)
% diagnoseFatropNativeConstraintGroups - 按约束组诊断 FATROP-native 不等式违反
if nargin < 5 || isempty(options)
    options = struct();
end
nativeOptions = makeNativeOptions(options);
data = evaluateFatropNativeHSTrajectoryNumeric(z, model, scene, disc);

pathValues = [];
roofValues = [];
leftValues = [];
rightValues = [];
monoValues = [];
softInsertionValues = [];
softSideValues = [];

for intervalIndex = 1:disc.numIntervals
    nodeX = data.Xnode(:, intervalIndex);
    midX = data.Xmid(:, intervalIndex);
    nodePoint = data.nodePoint{intervalIndex};
    midPoint = data.midPoint{intervalIndex};

    [pathValues, roofValues, leftValues, rightValues, monoValues, softInsertionValues, softSideValues] = ...
        appendPointDiagnostics(pathValues, roofValues, leftValues, rightValues, monoValues, softInsertionValues, softSideValues, ...
        nodeX, nodePoint, intervalIndex, true, scene, disc, nativeOptions);
    [pathValues, roofValues, leftValues, rightValues, monoValues, softInsertionValues, softSideValues] = ...
        appendPointDiagnostics(pathValues, roofValues, leftValues, rightValues, monoValues, softInsertionValues, softSideValues, ...
        midX, midPoint, intervalIndex, false, scene, disc, nativeOptions);
end

report = struct();
report.maxPathViolation = maxOrZero([pathValues(:); 0]);
report.maxRoofViolation = maxOrZero([roofValues(:); 0]);
report.maxLeftSideViolation = maxOrZero([leftValues(:); 0]);
report.maxRightSideViolation = maxOrZero([rightValues(:); 0]);
report.maxInsertionMonotonicViolation = maxOrZero([monoValues(:); 0]);
report.maxIneqViolation = max([report.maxPathViolation, report.maxRoofViolation, ...
    report.maxLeftSideViolation, report.maxRightSideViolation, ...
    report.maxInsertionMonotonicViolation]);
report.maxSoftInsertionResidual = maxOrZero(abs(softInsertionValues(:)));
report.maxSoftSideCollisionResidual = maxOrZero([softSideValues(:); 0]);
report.constraintProfile = nativeOptions.constraintProfile;
report.insertionMode = nativeOptions.insertionMode;
report.sideCollisionScope = nativeOptions.sideCollisionScope;
report.sideCollisionMode = nativeOptions.sideCollisionMode;
end

function [pathValues, roofValues, leftValues, rightValues, monoValues, softInsertionValues, softSideValues] = ...
    appendPointDiagnostics(pathValues, roofValues, leftValues, rightValues, monoValues, softInsertionValues, softSideValues, ...
    X, point, pointIndex, isNode, scene, disc, nativeOptions)
q = X(1:6);
if nativeOptions.includePathConstraints
    pathValues = [pathValues; point.cPath(:)]; %#ok<AGROW>
end
if nativeOptions.includeCollisionConstraints && isApproachPoint(pointIndex, isNode, disc)
    roofValues = [roofValues; fixedAxisCollisionIneqNumeric(q, scene.box.R_S * [0; 0; 1], ...
        scene.hood.roof, scene.collision.stage1ConstraintDistance, scene)]; %#ok<AGROW>
end
if nativeOptions.includeCollisionConstraints && shouldApplySideCollision(pointIndex, isNode, disc, nativeOptions)
    cLeft = fixedAxisCollisionIneqNumeric(q, scene.box.R_S * [0; 1; 0], ...
        scene.hood.leftSkirt, scene.collision.safeDistance, scene);
    cRight = fixedAxisCollisionIneqNumeric(q, scene.box.R_S * [0; -1; 0], ...
        scene.hood.rightSkirt, scene.collision.safeDistance, scene);
    if strcmp(nativeOptions.sideCollisionMode, 'hard')
        if sideMaskAllows(pointIndex, isNode, nativeOptions, 'left')
            leftValues = [leftValues; cLeft]; %#ok<AGROW>
        end
        if sideMaskAllows(pointIndex, isNode, nativeOptions, 'right')
            rightValues = [rightValues; cRight]; %#ok<AGROW>
        end
    elseif strcmp(nativeOptions.sideCollisionMode, 'smooth_finite')
        leftValues = [leftValues; smoothFiniteCollisionIneqNumeric(q, ...
            scene.hood.leftSkirt, scene.collision.safeDistance, scene, nativeOptions)]; %#ok<AGROW>
        rightValues = [rightValues; smoothFiniteCollisionIneqNumeric(q, ...
            scene.hood.rightSkirt, scene.collision.safeDistance, scene, nativeOptions)]; %#ok<AGROW>
    else
        softSideValues = [softSideValues; max(cLeft, 0); max(cRight, 0)]; %#ok<AGROW>
    end
end
if nativeOptions.includeInsertionConstraints && isStage2Point(pointIndex, isNode, disc)
    [gLine, cMono] = insertionConstraintsNumeric(X, scene);
    monoValues = [monoValues; cMono(:)]; %#ok<AGROW>
    if strcmp(nativeOptions.insertionMode, 'soft')
        softInsertionValues = [softInsertionValues; gLine(:)]; %#ok<AGROW>
    end
end
end

function nativeOptions = makeNativeOptions(options)
if isfield(options, 'constraintProfile') && ~isempty(options.constraintProfile)
    profile = lower(char(string(options.constraintProfile)));
else
    profile = 'full';
end
profile = strrep(profile, '-', '_');
nativeOptions = struct();
nativeOptions.constraintProfile = profile;
nativeOptions.includePathConstraints = true;
nativeOptions.includeCollisionConstraints = true;
nativeOptions.includeInsertionConstraints = true;
nativeOptions.insertionMode = 'soft';
nativeOptions.sideCollisionScope = 'all';
nativeOptions.sideCollisionMode = 'hard';
nativeOptions.sideCollisionMask = struct();
nativeOptions.softPositiveEps = 1e-6;
nativeOptions.collisionSmoothBeta = 80;
switch profile
    case 'full'
    case 'full_smooth_collision'
        nativeOptions.sideCollisionMode = 'smooth_finite';
    case 'full_stage2_side'
        nativeOptions.sideCollisionScope = 'stage2';
    case 'full_masked_side'
        nativeOptions.sideCollisionScope = 'masked_nominal';
    case 'full_soft_side'
        nativeOptions.sideCollisionMode = 'soft';
    case {'full_hard', 'hard_insertion'}
        nativeOptions.insertionMode = 'hard';
    case 'dynamics'
        nativeOptions.includePathConstraints = false;
        nativeOptions.includeCollisionConstraints = false;
        nativeOptions.includeInsertionConstraints = false;
    case 'actuator'
        nativeOptions.includeCollisionConstraints = false;
        nativeOptions.includeInsertionConstraints = false;
    case 'actuator_collision'
        nativeOptions.includeInsertionConstraints = false;
    case 'no_insertion'
        nativeOptions.includeInsertionConstraints = false;
    case 'no_collision'
        nativeOptions.includeCollisionConstraints = false;
    otherwise
        error('diagnoseFatropNativeConstraintGroups:UnknownConstraintProfile', ...
            '未知 FATROP-native 约束分层: %s。', profile);
end
if isfield(options, 'insertionMode') && ~isempty(options.insertionMode)
    nativeOptions.insertionMode = lower(char(string(options.insertionMode)));
end
if isfield(options, 'sideCollisionScope') && ~isempty(options.sideCollisionScope)
    nativeOptions.sideCollisionScope = lower(char(string(options.sideCollisionScope)));
end
if isfield(options, 'sideCollisionMode') && ~isempty(options.sideCollisionMode)
    nativeOptions.sideCollisionMode = lower(char(string(options.sideCollisionMode)));
end
if isfield(options, 'sideCollisionMask') && ~isempty(options.sideCollisionMask)
    nativeOptions.sideCollisionMask = options.sideCollisionMask;
end
if isfield(options, 'collisionSmoothBeta') && ~isempty(options.collisionSmoothBeta)
    nativeOptions.collisionSmoothBeta = options.collisionSmoothBeta;
end
end

function tf = isApproachPoint(pointIndex, isNode, disc)
if isNode
    tf = pointIndex <= disc.numIntervalsApproach + 1;
else
    tf = pointIndex <= disc.numIntervalsApproach;
end
end

function tf = isStage2Point(pointIndex, isNode, disc)
if isNode
    tf = pointIndex >= disc.waypointNodeIndex;
else
    tf = pointIndex >= disc.numIntervalsApproach + 1;
end
end

function tf = shouldApplySideCollision(pointIndex, isNode, disc, nativeOptions)
switch nativeOptions.sideCollisionScope
    case 'all'
        tf = true;
    case 'stage2'
        tf = isStage2Point(pointIndex, isNode, disc);
    case 'masked_nominal'
        tf = true;
    otherwise
        tf = false;
end
end

function tf = sideMaskAllows(pointIndex, isNode, nativeOptions, sideName)
if ~strcmp(nativeOptions.sideCollisionScope, 'masked_nominal')
    tf = true;
    return;
end
if isempty(fieldnames(nativeOptions.sideCollisionMask))
    tf = true;
    return;
end
if isNode
    key = [sideName, 'Node'];
else
    key = [sideName, 'Mid'];
end
mask = nativeOptions.sideCollisionMask.(key);
tf = pointIndex <= numel(mask) && mask(pointIndex);
end

function c = fixedAxisCollisionIneqNumeric(q, normalWorld, obstacle, requiredDistance, scene)
[centerS, axisS] = cylinderPoseWorldNumeric(q, scene);
normalWorld = normalWorld(:);
eta = abs(obstacle.R_S.' * normalWorld);
mu = normalWorld.' * axisS;
zeta = sqrt(mu^2 + scene.collision.smoothingEps^2);
rho = sqrt(1 - mu^2 + scene.collision.smoothingEps^2);
gap = normalWorld.' * (obstacle.center_S - centerS) - obstacle.halfSize.' * eta - ...
    0.5 * scene.objectCylinder.length * zeta - scene.objectCylinder.radius * rho;
c = requiredDistance - gap;
end

function c = smoothFiniteCollisionIneqNumeric(q, obstacle, requiredDistance, scene, nativeOptions)
[centerS, axisS] = cylinderPoseWorldNumeric(q, scene);
centerVector = obstacle.center_S - centerS;
centerNormal = centerVector ./ sqrt(centerVector.' * centerVector + nativeOptions.softPositiveEps^2);
candidateNormals = [obstacle.R_S(:, 1), -obstacle.R_S(:, 1), ...
    obstacle.R_S(:, 2), -obstacle.R_S(:, 2), ...
    obstacle.R_S(:, 3), -obstacle.R_S(:, 3), ...
    axisS, -axisS, centerNormal, -centerNormal];
candidateGaps = zeros(size(candidateNormals, 2), 1);
for i = 1:numel(candidateGaps)
    candidateGaps(i) = supportGapNumeric(candidateNormals(:, i), centerS, axisS, obstacle, scene, nativeOptions);
end
gap = log(sum(exp(nativeOptions.collisionSmoothBeta * candidateGaps))) / nativeOptions.collisionSmoothBeta;
c = requiredDistance - gap;
end

function gap = supportGapNumeric(n, cylinderCenter, cylinderAxis, obstacle, scene, nativeOptions)
n = n(:);
boxMin = n.' * obstacle.center_S - sum(obstacle.halfSize(:) .* sqrt((obstacle.R_S.' * n).^2 + nativeOptions.softPositiveEps^2));
mu = n.' * cylinderAxis;
axial = 0.5 * scene.objectCylinder.length * sqrt(mu^2 + nativeOptions.softPositiveEps^2);
radial = scene.objectCylinder.radius * sqrt(1 - mu^2 + nativeOptions.softPositiveEps^2);
cylinderMax = n.' * cylinderCenter + axial + radial;
gap = boxMin - cylinderMax;
end

function [centerS, axisS] = cylinderPoseWorldNumeric(q, scene)
R = rpy2rotmZYX(q(4:6));
centerS = q(1:3) + R * scene.objectCylinder.center_P;
axisS = R * scene.objectCylinder.axis_P;
end

function [gLine, cMono] = insertionConstraintsNumeric(X, scene)
q = X(1:6);
qd = X(7:12);
[pCylinder_B, ~] = cylinderPoseInBoxNumeric(q, scene);
lineHeight = insertionLineHeightNumeric(pCylinder_B(1), scene);
gLine = [pCylinder_B(2);
         pCylinder_B(3) - lineHeight;
         q(4:6) - scene.box.rpy];
R = rpy2rotmZYX(q(4:6));
omega = rpyRateMapZYX(q(4:6)) * qd(4:6);
pCd_S = qd(1:3) + cross(omega, R * scene.objectCylinder.center_P);
pCd_B = scene.box.R_S.' * pCd_S;
cMono = -pCd_B(1);
end

function [pCylinder_B, axis_B] = cylinderPoseInBoxNumeric(q, scene)
R = rpy2rotmZYX(q(4:6));
pCylinder_S = q(1:3) + R * scene.objectCylinder.center_P;
axis_S = R * scene.objectCylinder.axis_P;
pCylinder_B = scene.box.R_S.' * (pCylinder_S - scene.box.center_S);
axis_B = scene.box.R_S.' * axis_S;
end

function z = insertionLineHeightNumeric(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end

function value = maxOrZero(v)
if isempty(v)
    value = 0;
else
    value = max(v);
end
end
