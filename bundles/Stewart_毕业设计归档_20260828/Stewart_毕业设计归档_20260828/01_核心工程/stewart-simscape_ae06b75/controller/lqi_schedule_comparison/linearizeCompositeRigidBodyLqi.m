function linearization = linearizeCompositeRigidBodyLqi(q, qd, force, model, options)
% linearizeCompositeRigidBodyLqi - Linearize the exact rigid-body plant.
%
% The state is x=[q; qd] and the input is the six leg-force vector.  The
% state matrix is evaluated by central differences, while the input map is
% taken from H(q,qd)^(-1)*Jv(q)' so it has the same definition as the exact
% nonlinear dynamics used by the comparison study.
arguments
    q double
    qd double
    force double
    model struct
    options.stateStep double = [1e-6 * ones(6, 1); 1e-5 * ones(6, 1)]
    options.inputStep (1, 1) double {mustBePositive} = 1
    options.checkConvergence (1, 1) logical = true
end

q = q(:);
qd = qd(:);
force = force(:);
validateattributes(q, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(qd, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(force, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(options.stateStep, {'double'}, ...
    {'real', 'finite', 'positive', 'size', [12, 1]});

state = [q; qd];
[stateDerivative, auxiliary] = stateDynamicsCompositeRigidBody(state, force, model);
if auxiliary.rcondH < model.num.rcondMin
    error('linearizeCompositeRigidBodyLqi:IllConditionedMassMatrix', ...
        'The reference mass matrix is ill-conditioned (rcond=%.3e).', ...
        auxiliary.rcondH);
end

forceMap = auxiliary.Jout.Jv.';
forceMapRcond = rcond(forceMap);
if forceMapRcond < model.num.rcondMin
    error('linearizeCompositeRigidBodyLqi:IllConditionedJacobian', ...
        'The reference force map is ill-conditioned (rcond=%.3e).', ...
        forceMapRcond);
end

A = stateJacobian(state, force, model, options.stateStep);
B = [zeros(6, 6); auxiliary.H \ forceMap];
C = [eye(6), zeros(6, 6)];
D = zeros(6, 6);

numericB = zeros(12, 6);
for inputIndex = 1:6
    perturbation = zeros(6, 1);
    perturbation(inputIndex) = options.inputStep;
    plus = stateDynamicsCompositeRigidBody(state, force + perturbation, model);
    minus = stateDynamicsCompositeRigidBody(state, force - perturbation, model);
    numericB(:, inputIndex) = (plus - minus) / (2 * options.inputStep);
end

if options.checkConvergence
    AHalfStep = stateJacobian(state, force, model, options.stateStep / 2);
    aConvergenceRelative = norm(A-AHalfStep, 'fro') / max(1, norm(AHalfStep, 'fro'));
else
    AHalfStep = nan(size(A));
    aConvergenceRelative = NaN;
end

linearization = struct();
linearization.state = state;
linearization.force = force;
linearization.stateDerivative = stateDerivative;
linearization.A = A;
linearization.B = B;
linearization.C = C;
linearization.D = D;
linearization.plant = ss(A, B, C, D);
linearization.massMatrix = auxiliary.H;
linearization.jacobian = auxiliary.Jout.Jv;
linearization.massMatrixRcond = auxiliary.rcondH;
linearization.forceMapRcond = forceMapRcond;
linearization.numericB = numericB;
linearization.inputMapRelativeError = norm(B-numericB, 'fro') / ...
    max(1, norm(numericB, 'fro'));
linearization.AHalfStep = AHalfStep;
linearization.aConvergenceRelative = aConvergenceRelative;
linearization.stateStep = options.stateStep;
end

function A = stateJacobian(state, force, model, stateStep)
A = zeros(12, 12);
for stateIndex = 1:12
    perturbation = zeros(12, 1);
    perturbation(stateIndex) = stateStep(stateIndex);
    plus = stateDynamicsCompositeRigidBody(state + perturbation, force, model);
    minus = stateDynamicsCompositeRigidBody(state - perturbation, force, model);
    A(:, stateIndex) = (plus - minus) / (2 * stateStep(stateIndex));
end
end
