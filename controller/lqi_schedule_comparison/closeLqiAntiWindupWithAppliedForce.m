function [nextState, mismatch, diagnostic] = ...
        closeLqiAntiWindupWithAppliedForce(sample, controllerState, ...
        poseError, rawFeedbackForce, appliedTotalForce, feedforwardForce, ...
        dobForce, mismatchLimit, mismatchGain)
% closeLqiAntiWindupWithAppliedForce - Back-calculate from final QP output.
%
% The equivalent feedback actually delivered to the plant is
% Fapplied-Ffeedforward-Fdob. Its mismatch from the unsaturated LQI output is
% passed through the existing anti-windup input channel.
arguments
    sample struct
    controllerState double
    poseError double
    rawFeedbackForce double
    appliedTotalForce double
    feedforwardForce double
    dobForce double
    mismatchLimit double
    mismatchGain (1, 1) double {mustBePositive} = 1
end

controllerState = controllerState(:);
poseError = reshape(poseError, 6, 1);
rawFeedbackForce = reshape(rawFeedbackForce, 6, 1);
appliedTotalForce = reshape(appliedTotalForce, 6, 1);
feedforwardForce = reshape(feedforwardForce, 6, 1);
dobForce = reshape(dobForce, 6, 1);
mismatchLimit = mismatchLimit(:);
if isscalar(mismatchLimit), mismatchLimit = repmat(mismatchLimit, 6, 1); end
validateattributes(mismatchLimit, {'double'}, ...
    {'real', 'finite', 'positive', 'size', [6, 1]});

equivalentAppliedFeedback = appliedTotalForce-feedforwardForce-dobForce;
rawMismatch = equivalentAppliedFeedback-rawFeedbackForce;
scaledMismatch = mismatchGain*rawMismatch;
mismatch = min(max(scaledMismatch, -mismatchLimit), mismatchLimit);
nextState = sample.controllerA*controllerState+ ...
    sample.controllerB*[poseError; mismatch];
diagnostic = struct('equivalentAppliedFeedback', equivalentAppliedFeedback, ...
    'rawMismatch', rawMismatch, 'mismatchGain', mismatchGain, ...
    'scaledMismatch', scaledMismatch, 'limitedMismatch', mismatch, ...
    'limitActive', any(abs(scaledMismatch) > mismatchLimit+eps));
end
