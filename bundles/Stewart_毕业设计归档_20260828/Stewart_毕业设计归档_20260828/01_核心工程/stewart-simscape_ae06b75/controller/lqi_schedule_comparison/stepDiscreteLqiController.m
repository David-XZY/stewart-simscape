function [nextState, nextAntiWindup, rawForce, saturatedForce] = ...
        stepDiscreteLqiController(sample, controllerState, poseError, antiWindup, forceLimit)
% stepDiscreteLqiController - One fixed or scheduled discrete LQI update.
arguments
    sample struct
    controllerState double
    poseError double
    antiWindup double
    forceLimit double
end

controllerState = controllerState(:);
poseError = poseError(:);
antiWindup = antiWindup(:);
forceLimit = forceLimit(:);
if isscalar(forceLimit)
    forceLimit = repmat(forceLimit, 6, 1);
end
validateattributes(poseError, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(antiWindup, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(forceLimit, {'double'}, {'real', 'finite', 'positive', 'size', [6, 1]});

trackingInput = [poseError; antiWindup];
rawForce = sample.controllerC*controllerState + sample.controllerD*trackingInput;
saturatedForce = min(max(rawForce, -forceLimit), forceLimit);
nextAntiWindup = saturatedForce-rawForce;
nextState = sample.controllerA*controllerState + ...
    sample.controllerB*[poseError; nextAntiWindup];
end
