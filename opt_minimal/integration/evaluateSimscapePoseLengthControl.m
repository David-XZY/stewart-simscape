function report = evaluateSimscapePoseLengthControl(simout, refs, model, design, config)
% evaluateSimscapePoseLengthControl - 验收位姿外环纯腿长输入控制
report = evaluateSimscapeLengthCascadeControl(simout, refs, model, design, config);
poseLengthCorrection = report.poseError * design.poseToLengthGain.';
poseLengthCorrection = min(max(poseLengthCorrection, -config.poseCorrectionLimit), ...
    config.poseCorrectionLimit);
report.poseLengthCorrection = poseLengthCorrection;
report.metrics.maxAbsPoseLengthCorrection = max(abs(poseLengthCorrection), [], 'all');
report.acceptance.poseCorrectionPassed = ...
    report.metrics.maxAbsPoseLengthCorrection <= config.poseCorrectionLimit + 1e-9;
report.passed = report.passed && report.acceptance.poseCorrectionPassed;
end
