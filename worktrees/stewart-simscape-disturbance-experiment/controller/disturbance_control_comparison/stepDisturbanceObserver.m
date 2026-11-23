function [compensation, state, diagnostic] = stepDisturbanceObserver( ...
        q, qd, measuredAcceleration, appliedForce, model, config, state)
% stepDisturbanceObserver - Estimate external wrench and cancel it in leg space.
%
% Plant convention:
%   H*qdd = Jv'*F + Wdist - Wbias.
% Therefore residual=H*qdd+Wbias-Jv'*F estimates Wdist, and the cancelling
% leg-force command satisfies Jv'*Fcomp=-Wdist_hat.
arguments
    q double
    qd double
    measuredAcceleration double
    appliedForce double
    model struct
    config struct
    state struct
end

q = reshape(q, 6, 1);
qd = reshape(qd, 6, 1);
measuredAcceleration = reshape(measuredAcceleration, 6, 1);
appliedForce = reshape(appliedForce, 6, 1);
jacobian = sgpJacobian(q, model);
biasWrench = computeCompositeRequiredWrench(q, qd, zeros(6, 1), model);
massMatrix = zeros(6, 6);
for axis = 1:6
    unitAcceleration = zeros(6, 1);
    unitAcceleration(axis) = 1;
    massMatrix(:, axis) = computeCompositeRequiredWrench( ...
        q, qd, unitAcceleration, model)-biasWrench;
end
rawResidual = massMatrix*measuredAcceleration+biasWrench- ...
    jacobian.Jv.'*appliedForce;

if state.initialized
    estimate = config.filterAlpha*state.wrenchEstimate+ ...
        (1-config.filterAlpha)*rawResidual;
else
    % Zero initialization avoids interpreting the known initial equilibrium
    % as an impulsive disturbance.
    estimate = zeros(6, 1);
    state.initialized = true;
end

forceMap = jacobian.Jv.';
if rcond(forceMap) < config.mappingRcondMin
    rawCompensation = -pinv(forceMap)*estimate;
else
    rawCompensation = -(forceMap\estimate);
end
limited = min(max(rawCompensation, -config.legForceLimit), config.legForceLimit);
maximumChange = config.rateLimit*config.sampleTime;
change = min(max(limited-state.legCompensation, -maximumChange), maximumChange);
compensation = state.legCompensation+change;
forceLimitActive = any(abs(rawCompensation) > config.legForceLimit+eps);
rateLimitActive = any(abs(limited-state.legCompensation) > maximumChange+eps);

state.wrenchEstimate = estimate;
state.legCompensation = compensation;
diagnostic = struct('rawResidual', rawResidual, ...
    'wrenchEstimate', estimate, 'rawCompensation', rawCompensation, ...
    'limitedCompensation', compensation, ...
    'forceLimitActive', forceLimitActive, ...
    'rateLimitActive', rateLimitActive, ...
    'mappingRcond', rcond(forceMap));
end
