function test_72_strict_ad_barrier_derivatives
% Validate CasADi gradients/Hessians and the singular-value lower bound.
[model, scene, controllerRoot] = testEnvironment();
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));
config = makeStrictClfCbfQpConfig(model);
q = model.qHome + [0.01; -0.015; 0.02; 0.015; -0.01; 0.02];
qd = [0.02; -0.01; 0.015; 0.01; -0.008; 0.006];
direction = [0.3; -0.4; 0.2; 0.5; -0.1; 0.25];
direction = direction/norm(direction);
evaluation = evaluateStrictClfCbfAdModel(q, qd, model, scene, config);

% Central differences below are independent test oracles only; controller
% derivatives are the CasADi AD quantities checked by this test.
step = 2e-6;
plus = evaluateStrictClfCbfAdModel(q+step*direction, qd, model, scene, config);
minus = evaluateStrictClfCbfAdModel(q-step*direction, qd, model, scene, config);
collisionDirectional = (plus.collisionGap-minus.collisionGap)/(2*step);
assert(max(abs(collisionDirectional-evaluation.collisionGradient*direction)) < 2e-5);

gradientDirectional = (plus.collisionGradient-minus.collisionGradient)/(2*step);
for obstacleIndex = 1:3
    expected = evaluation.collisionHessian(:, :, obstacleIndex)*direction;
    assert(max(abs(gradientDirectional(obstacleIndex, :).'-expected)) < 3e-4);
end
sigmaDirectional = (plus.singularityBarrier-minus.singularityBarrier)/(2*step);
assert(abs(sigmaDirectional-evaluation.singularityGradient*direction) < 2e-5);
assert(max(abs(evaluation.singularityHessian-evaluation.singularityHessian.'), [], 'all') < 1e-9);

poses = [model.qHome, scene.qWaypoint, scene.qGoal, q];
for index = 1:size(poses, 2)
    item = evaluateStrictClfCbfAdModel(poses(:, index), zeros(6, 1), model, scene, config);
    sigmaMinSquared = min(svd(item.Jbar))^2;
    assert(item.sigmaLowerSquared <= sigmaMinSquared+1e-12);
    assert(item.sigmaLowerSquared > 0);
end

% The cached graph must be rebuilt when any obstacle orientation changes.
rotatedScene = scene;
angle = deg2rad(7);
localRotation = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; 0, 0, 1];
rotatedScene.hood.leftSkirt.R_S = ...
    scene.hood.leftSkirt.R_S*localRotation;
rotated = evaluateStrictClfCbfAdModel(q, qd, model, rotatedScene, config);
assert(abs(rotated.collisionGap(2)-evaluation.collisionGap(2)) > 1e-9);
end

function [model, scene, controllerRoot] = testEnvironment()
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
end
