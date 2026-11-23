function targetForce = computeComputedTorqueForce(referencePose, referenceVelocity, ...
        referenceAcceleration, feedbackPose, feedbackVelocity, model, config)
% computeComputedTorqueForce - 由参考轨迹计算纯逆动力学腿力前馈
arguments
    referencePose double
    referenceVelocity double
    referenceAcceleration double
    feedbackPose double
    feedbackVelocity double
    model struct
    config struct
end

referencePose = referencePose(:);
referenceVelocity = referenceVelocity(:);
referenceAcceleration = referenceAcceleration(:);
feedbackPose = feedbackPose(:);
feedbackVelocity = feedbackVelocity(:);

validateattributes(referencePose, {'double'}, {'real', 'finite', 'numel', 6});
validateattributes(referenceVelocity, {'double'}, {'real', 'finite', 'numel', 6});
validateattributes(referenceAcceleration, {'double'}, {'real', 'finite', 'numel', 6});
validateattributes(feedbackPose, {'double'}, {'real', 'finite', 'numel', 6});
validateattributes(feedbackVelocity, {'double'}, {'real', 'finite', 'numel', 6});

jacobian = sgpJacobian(referencePose, model);
wrench = computeCompositeRequiredWrench( ...
    referencePose, referenceVelocity, referenceAcceleration, model);
matrixToSolve = jacobian.Jv.';
if rcond(matrixToSolve) < model.num.rcondMin
    targetForce = pinv(matrixToSolve) * wrench;
else
    targetForce = matrixToSolve \ wrench;
end
targetForce = min(max(targetForce, model.actuator.forceMin), model.actuator.forceMax);
end
