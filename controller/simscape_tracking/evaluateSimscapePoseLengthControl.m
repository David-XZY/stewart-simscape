function report = evaluateSimscapePoseLengthControl(simout, refs, model, design, config)
% evaluateSimscapePoseLengthControl - 验收位姿外环纯腿长输入控制
report = evaluateSimscapeLengthCascadeControl(simout, refs, model, design, config);
poseError = interp1(report.poseTime, report.poseError, refs.t(:), 'linear');
filteredPoseError = firstOrderLowpass(poseError, design.poseFeedbackFilterAlpha);
poseLengthCorrection = zeros(size(filteredPoseError));
for sampleIndex = 1:numel(refs.t)
    jacobian = sgpJacobian(refs.q(:, sampleIndex), model);
    poseLengthCorrection(sampleIndex, :) = ...
        (design.poseFeedbackGain * jacobian.Jq * filteredPoseError(sampleIndex, :).').';
end
poseLengthCorrection = min(max(poseLengthCorrection, -config.poseCorrectionLimit), ...
    config.poseCorrectionLimit);
report.poseLengthCorrectionTime = refs.t(:);
report.filteredPoseError = filteredPoseError;
report.poseLengthCorrection = poseLengthCorrection;
report.metrics.maxAbsPoseLengthCorrection = max(abs(poseLengthCorrection), [], 'all');
equivalentPoseError = max(abs([poseError(:, 1:3), 0.5 * poseError(:, 4:6)]), [], 2);
rippleMask = refs.t(:) >= 5;
rippleWindow = max(3, round(0.5 / config.sampleTime));
rippleResidual = equivalentPoseError(rippleMask) - ...
    movmean(equivalentPoseError(rippleMask), rippleWindow);
report.metrics.post5sEquivalentPoseRippleRms = sqrt(mean(rippleResidual.^2));
report.acceptance.poseCorrectionPassed = ...
    report.metrics.maxAbsPoseLengthCorrection <= config.poseCorrectionLimit + 1e-9;
report.passed = report.passed && report.acceptance.poseCorrectionPassed;
end

function filtered = firstOrderLowpass(value, alpha)
% firstOrderLowpass - 重建 Run04 离散一阶位姿误差低通
filtered = zeros(size(value));
filtered(1, :) = (1 - alpha) * value(1, :);
for sampleIndex = 2:size(value, 1)
    filtered(sampleIndex, :) = alpha * filtered(sampleIndex - 1, :) + ...
        (1 - alpha) * value(sampleIndex, :);
end
end
