function [distances, nextSeed, diagnostics] = ...
        evaluateCylinderObbDistancesExact(q, scene, seed)
% evaluateCylinderObbDistancesExact - Independent finite-cylinder/OBB distance.
%
% The optimization variables parameterize one point in the finite solid
% cylinder and one point in the OBB.  This is an independent convex
% point-to-point distance calculation and does not reuse the QP collision
% certificate or its fixed normal bank.
arguments
    q double
    scene struct
    seed double = zeros(6, 3)
end
q = q(:);
validateattributes(q, {'double'}, ...
    {'real', 'finite', 'size', [6, 1]});
validateattributes(seed, {'double'}, ...
    {'real', 'finite', 'size', [6, 3]});

R = rpy2rotmZYX(q(4:6));
cylinder = scene.objectCylinder;
center = q(1:3)+R*cylinder.center_P;
axisDirection = R*cylinder.axis_P;
axisDirection = axisDirection/norm(axisDirection);
if abs(axisDirection(1)) < 0.8
    basisSeed = [1; 0; 0];
else
    basisSeed = [0; 1; 0];
end
radialOne = cross(axisDirection, basisSeed);
radialOne = radialOne/norm(radialOne);
radialTwo = cross(axisDirection, radialOne);

obstacles = scene.hood.obstacles;
distances = zeros(3, 1);
nextSeed = zeros(6, 3);
exitflag = zeros(3, 1);
iterations = zeros(3, 1);
for obstacleIndex = 1:3
    obstacle = obstacles(obstacleIndex);
    mapping = [axisDirection, radialOne, radialTwo, -obstacle.R_S];
    offset = center-obstacle.center_S;
    objective = @(value) squaredPointDistance(value, mapping, offset);
    nonlinear = @(value) radialDisk(value, cylinder.radius);
    lower = [-0.5*cylinder.length; -cylinder.radius; -cylinder.radius; ...
        -obstacle.halfSize(:)];
    upper = [0.5*cylinder.length; cylinder.radius; cylinder.radius; ...
        obstacle.halfSize(:)];
    initial = min(max(seed(:, obstacleIndex), lower), upper);
    radialNorm = norm(initial(2:3));
    if radialNorm > cylinder.radius
        initial(2:3) = initial(2:3)*cylinder.radius/radialNorm;
    end
    [solution, squaredDistance, flag, output] = fmincon( ...
        objective, initial, [], [], [], [], lower, upper, nonlinear, ...
        exactDistanceOptions());
    if flag <= 0 || ~isfinite(squaredDistance)
        [solution, squaredDistance, flag, output] = fmincon( ...
            objective, zeros(6, 1), [], [], [], [], lower, upper, ...
            nonlinear, exactDistanceOptions());
    end
    if flag <= 0 || ~isfinite(squaredDistance)
        error('evaluateCylinderObbDistancesExact:OptimizationFailed', ...
            'Independent distance solve failed for obstacle %d.', ...
            obstacleIndex);
    end
    distances(obstacleIndex) = sqrt(max(squaredDistance, 0));
    nextSeed(:, obstacleIndex) = solution;
    exitflag(obstacleIndex) = flag;
    iterations(obstacleIndex) = output.iterations;
end
diagnostics = struct('exitflag', exitflag, 'iterations', iterations, ...
    'method', "independent convex point-to-point distance");
end

function [value, gradient] = squaredPointDistance(z, mapping, offset)
difference = offset+mapping*z;
value = difference.'*difference;
if nargout > 1
    gradient = 2*mapping.'*difference;
end
end

function [constraint, equality, gradient, equalityGradient] = ...
        radialDisk(z, radius)
constraint = z(2)^2+z(3)^2-radius^2;
equality = [];
if nargout > 2
    gradient = zeros(6, 1);
    gradient(2) = 2*z(2);
    gradient(3) = 2*z(3);
    equalityGradient = [];
end
end

function options = exactDistanceOptions()
persistent cachedOptions
if isempty(cachedOptions)
    cachedOptions = optimoptions('fmincon', 'Display', 'off', ...
        'Algorithm', 'sqp', 'SpecifyObjectiveGradient', true, ...
        'SpecifyConstraintGradient', true, ...
        'ConstraintTolerance', 1e-10, ...
        'OptimalityTolerance', 1e-10, 'StepTolerance', 1e-11, ...
        'MaxIterations', 80, 'MaxFunctionEvaluations', 500);
end
options = cachedOptions;
end
