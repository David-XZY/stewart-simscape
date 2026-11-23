function innovation = computePoseInnovation(measuredPose, estimatedPose)
% computePoseInnovation - Pose residual with continuous wrapped RPY terms.
measuredPose = reshape(measuredPose, 6, 1);
estimatedPose = reshape(estimatedPose, 6, 1);
validateattributes(measuredPose, {'double'}, {'real', 'finite'});
validateattributes(estimatedPose, {'double'}, {'real', 'finite'});
innovation = measuredPose-estimatedPose;
innovation(4:6) = atan2(sin(innovation(4:6)), cos(innovation(4:6)));
end
