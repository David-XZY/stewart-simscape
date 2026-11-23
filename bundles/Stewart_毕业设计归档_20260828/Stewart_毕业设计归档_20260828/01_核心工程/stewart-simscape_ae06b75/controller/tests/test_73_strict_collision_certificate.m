function test_73_strict_collision_certificate
% Every smooth certificate must under-estimate independent convex distance.
[model, scene, controllerRoot] = testEnvironment();
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));
config = makeStrictClfCbfQpConfig(model);

tau = linspace(0, 1, 7);
approach = scene.q0(:)*(1-tau) + scene.qWaypoint(:)*tau;
insertion = scene.qWaypoint(:)*(1-tau(2:end)) + scene.qGoal(:)*tau(2:end);
poses = [approach, insertion];
for index = 1:size(poses, 2)
    q = poses(:, index);
    evaluation = evaluateStrictClfCbfAdModel(q, zeros(6, 1), model, scene, config);
    independentDistance = zeros(3, 1);
    for obstacleIndex = 1:3
        independentDistance(obstacleIndex) = independentCylinderObbDistance( ...
            q, scene.objectCylinder, scene.hood.obstacles(obstacleIndex));
    end
    assert(all(evaluation.collisionGap <= independentDistance+2e-6));
    roof = evaluateCylinderBoxClearance(q, scene);
    assert(evaluation.collisionGap(1) <= roof.distance+2e-10);
    assert(roof.distance-evaluation.collisionGap(1) < 2e-5);
end

before = evaluateStrictRoofThreshold(scene.phase.durationApproach, scene, config);
after = evaluateStrictRoofThreshold(scene.phase.durationApproach+ ...
    scene.phase.durationInsertion, scene, config);
middle = evaluateStrictRoofThreshold(scene.phase.durationApproach+ ...
    0.5*scene.phase.durationInsertion, scene, config);
assert(abs(before.value-scene.collision.stage1ConstraintDistance) < 1e-14);
assert(abs(after.value-scene.collision.finalGap) < 1e-14);
assert(before.rate == 0 && before.acceleration == 0);
assert(after.rate == 0 && after.acceleration == 0);
assert(middle.value < before.value && middle.value > after.value);
end

function distance = independentCylinderObbDistance(q, cylinder, obstacle)
% Independent convex point-to-point distance.  Variables parameterize one
% point in the finite cylinder and one point in the OBB.
R = rpy2rotmZYX(q(4:6));
center = q(1:3) + R*cylinder.center_P;
axis = R*cylinder.axis_P;
axis = axis/norm(axis);
if abs(axis(1)) < 0.8
    seed = [1; 0; 0];
else
    seed = [0; 1; 0];
end
radialOne = cross(axis, seed);
radialOne = radialOne/norm(radialOne);
radialTwo = cross(axis, radialOne);

pointDifference = @(z) center + axis*z(1) + radialOne*z(2) + ...
    radialTwo*z(3) - obstacle.center_S - obstacle.R_S*z(4:6);
objective = @(z) pointDifference(z).'*pointDifference(z);
lower = [-0.5*cylinder.length; -cylinder.radius; -cylinder.radius; ...
    -obstacle.halfSize(:)];
upper = [0.5*cylinder.length; cylinder.radius; cylinder.radius; ...
    obstacle.halfSize(:)];
nonlinear = @(z) radialDisk(z, cylinder.radius);
options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
    'ConstraintTolerance', 1e-11, 'OptimalityTolerance', 1e-11, ...
    'StepTolerance', 1e-12, 'MaxIterations', 200);
[~, squaredDistance, exitflag] = fmincon(objective, zeros(6, 1), ...
    [], [], [], [], lower, upper, nonlinear, options);
assert(exitflag > 0);
distance = sqrt(max(squaredDistance, 0));
end

function [constraint, equality] = radialDisk(z, radius)
constraint = z(2)^2+z(3)^2-radius^2;
equality = [];
end

function [model, scene, controllerRoot] = testEnvironment()
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
end
