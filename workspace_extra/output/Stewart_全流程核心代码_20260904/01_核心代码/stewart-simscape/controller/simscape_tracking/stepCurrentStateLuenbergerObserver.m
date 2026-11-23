function [estimate, diagnostic] = stepCurrentStateLuenbergerObserver( ...
        previousEstimate, measuredPose, previousForce, model, observer, dt, ...
        previousMeasuredPose)
% stepCurrentStateLuenbergerObserver - One nonlinear observer interval.
arguments
    previousEstimate double
    measuredPose double
    previousForce double
    model struct
    observer struct
    dt (1, 1) double {mustBePositive, mustBeFinite} = observer.sampleTime
    previousMeasuredPose double = measuredPose
end

previousEstimate = reshape(previousEstimate, 12, 1);
measuredPose = reshape(measuredPose, 6, 1);
previousForce = reshape(previousForce, 6, 1);
previousMeasuredPose = reshape(previousMeasuredPose, 6, 1);
validateattributes(previousEstimate, {'double'}, {'real', 'finite'});
validateattributes(measuredPose, {'double'}, {'real', 'finite'});
validateattributes(previousForce, {'double'}, {'real', 'finite'});
validateattributes(previousMeasuredPose, {'double'}, {'real', 'finite'});

measurementIncrement = computePoseInnovation(measuredPose, previousMeasuredPose);
midpointMeasurement = previousMeasuredPose+0.5*measurementIncrement;
k1 = observerDerivative(previousEstimate, previousMeasuredPose, ...
    previousForce, model, observer.gain);
k2 = observerDerivative(previousEstimate+0.5*dt*k1, midpointMeasurement, ...
    previousForce, model, observer.gain);
k3 = observerDerivative(previousEstimate+0.5*dt*k2, midpointMeasurement, ...
    previousForce, model, observer.gain);
k4 = observerDerivative(previousEstimate+dt*k3, measuredPose, ...
    previousForce, model, observer.gain);
estimate = previousEstimate+dt*(k1+2*k2+2*k3+k4)/6;
if any(~isfinite(estimate))
    error('stepCurrentStateLuenbergerObserver:NonfiniteEstimate', ...
        'The observer produced a nonfinite state estimate.');
end

innovation = computePoseInnovation(measuredPose, estimate(1:6));
[~, auxiliary] = stateDynamicsCompositeRigidBody(estimate, previousForce, model);
jacobian = sgpJacobian(estimate(1:6), model);
jacobianRcond = rcond(jacobian.Jv);
if auxiliary.rcondH < model.num.rcondMin || ...
        jacobianRcond < model.num.rcondMin
    error('stepCurrentStateLuenbergerObserver:IllConditionedModel', ...
        'Observer state is ill-conditioned (rcondH=%.3e, rcondJv=%.3e).', ...
        auxiliary.rcondH, jacobianRcond);
end
diagnostic = struct('innovation', innovation, ...
    'innovationNorm', norm(innovation), 'rcondH', auxiliary.rcondH, ...
    'rcondJv', jacobianRcond);
end

function value = observerDerivative(state, measuredPose, force, model, gain)
plantDerivative = stateDynamicsCompositeRigidBody(state, force, model);
innovation = computePoseInnovation(measuredPose, state(1:6));
value = plantDerivative+gain*innovation;
end
