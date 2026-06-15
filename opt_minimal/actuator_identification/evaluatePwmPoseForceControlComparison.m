function report = evaluatePwmPoseForceControlComparison(comparison, teacher)
% evaluatePwmPoseForceControlComparison - 验收双线 PWM 力输入轨迹控制
identified = comparison.identified;
poseError = identified.qTrue - identified.qReference;

metrics = comparison.metrics;
metrics.identifiedTranslationAxisPeak = max(abs(poseError(1:3, :)), [], 2);
metrics.identifiedTranslationPeak = max(abs(poseError(1:3, :)), [], 'all');
metrics.identifiedRotationPeak = max(abs(poseError(4:6, :)), [], 'all');
metrics.identifiedMaxAbsPwm = max(abs(identified.pwm), [], 'all');
metrics.identifiedMaxAbsTrueForce = max(abs(identified.trueForce), [], 'all');
settled = identified.t >= comparison.options.ukfWarmupDuration;
estimatorPoseError = identified.qFeedback - identified.qTrue;
estimatorVelocityError = identified.qdFeedback - identified.qdTrue;
characteristicLength = 0.5;
equivalentVelocityError = [estimatorVelocityError(1:3, :); ...
    characteristicLength * estimatorVelocityError(4:6, :)];
metrics.estimatorSettledTranslationAxisPeak = ...
    max(abs(estimatorPoseError(1:3, settled)), [], 2);
metrics.estimatorSettledTranslationRms = ...
    sqrt(mean(estimatorPoseError(1:3, settled).^2, 'all'));
metrics.estimatorVelocityRms = sqrt(mean(equivalentVelocityError(:, settled).^2, 'all'));

acceptance = struct();
acceptance.finitePassed = all(isfinite(identified.qTrue), 'all') && ...
    all(isfinite(identified.trueForce), 'all') && all(isfinite(identified.estimatedForce), 'all');
acceptance.forceTrackingPassed = metrics.identifiedForceTrackingNrmse <= 0.075;
acceptance.strictTranslationPassed = all(metrics.identifiedTranslationAxisPeak <= 0.001);
acceptance.translationPassed = all(metrics.identifiedTranslationAxisPeak <= 0.005);
acceptance.estimatorTranslationPassed = ...
    all(metrics.estimatorSettledTranslationAxisPeak <= 0.002);
acceptance.estimatorVelocityPassed = metrics.estimatorVelocityRms <= 0.01;
acceptance.rotationPassed = metrics.identifiedRotationPeak <= deg2rad(1);
acceptance.pwmPassed = metrics.identifiedMaxAbsPwm <= teacher.pwmMax + 1e-9;
acceptance.forceLimitPassed = metrics.identifiedMaxAbsTrueForce <= max(teacher.forceLimit) + 1e-9;
acceptance.truthIsolationPassed = ~identified.observability.usedTrueForceFeedback && ...
    ~identified.observability.usedTruePoseFeedback && identified.observability.usedEncoderImuFusion;

report = struct();
report.metrics = metrics;
report.acceptance = acceptance;
gatingAcceptance = rmfield(acceptance, 'strictTranslationPassed');
report.passed = all(cell2mat(struct2cell(gatingAcceptance)));
end
