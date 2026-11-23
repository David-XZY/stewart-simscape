function test_74_strict_clf_cbf_qp_solver_smoke
% The nominal safe state must solve with only the scalar CLF relaxation.
[model, scene, controllerRoot] = testEnvironment();
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));
config = makeStrictClfCbfQpConfig(model);
q = model.qHome;
qd = zeros(6, 1);
nominalForce = inverseDynamicsCompositeRigidBody(q, qd, zeros(6, 1), model);
previousForce = nominalForce;

[forceCommand, diagnostic] = stepStrictClfCbfQp(q, qd, q, qd, qd, ...
    nominalForce, previousForce, model, scene, config);

assert(isequal(size(forceCommand), [6, 1]));
assert(all(isfinite(forceCommand)));
assert(diagnostic.status == "solved");
assert(diagnostic.feasible && ~diagnostic.usedFallback);
assert(diagnostic.decisionVariableCount == 7);
assert(diagnostic.softConstraintCount == 1);
assert(~diagnostic.hasCbfSlack && ~diagnostic.cbf.hasSlack);
assert(size(diagnostic.cbf.A, 1) == 40);
assert(diagnostic.hardConstraintCount == 64);
assert(diagnostic.hardConstraintViolation <= 1e-8);
assert(diagnostic.minimumCbfResidual >= -1e-8);
assert(diagnostic.cbf.minimumStateMargin > 0);
assert(diagnostic.cbf.minimumPsi1 > 0);
assert(all(forceCommand >= diagnostic.forceLower-1e-9));
assert(all(forceCommand <= diagnostic.forceUpper+1e-9));
assert(all(forceCommand >= config.forceMin-1e-9));
assert(all(forceCommand <= config.forceMax+1e-9));
assert(all(abs(forceCommand-previousForce) <= ...
    config.forceRateLimit*config.dt+1e-9));

evaluation = diagnostic.cbf.evaluation;
legAcceleration = evaluation.legAccelerationDrift + ...
    evaluation.legAccelerationMap*forceCommand;
assert(all(abs(legAcceleration) <= config.legAccelerationLimit+1e-8));
assert(diagnostic.derivativeBackend == "CasADi algorithmic differentiation");
assert(~diagnostic.usesFiniteDifferences);

% An infeasible hard barrier must be reported as an uncertified fallback.
impossibleCbf = struct('A', zeros(1, 6), 'b', -1, ...
    'names', "deliberately_impossible");
zeroClf = struct('A', zeros(1, 6), 'b', 0);
[~, fallback] = solveStrictClfCbfQp( ...
    nominalForce, previousForce, zeroClf, impossibleCbf, config);
assert(fallback.usedFallback && ~fallback.feasible);
assert(~fallback.fallbackSafetyCertified);
assert(fallback.hardConstraintViolation > 0);
end

function [model, scene, controllerRoot] = testEnvironment()
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
end
