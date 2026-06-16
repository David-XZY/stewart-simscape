function test_29_pwm_dual_line_control_contract
% test_29_pwm_dual_line_control_contract - 验证双线独立控制与真值隔离
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct('sampleCount', 1500, 'randomSeed', 29));
identified = trainGrayNarxForceIdentifier(dataset, teacher);
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');

comparison = comparePwmPoseForceControlLines(sample.refs, teacher, identified, struct( ...
    'duration', 0.8));
assert(strcmp(comparison.oracle.plantType, teacher.type));
assert(strcmp(comparison.identifiedTruthFeedback.plantType, teacher.type));
assert(strcmp(comparison.identified.plantType, teacher.type));
assert(comparison.oracle.observability.usedTrueForceFeedback);
assert(comparison.oracle.observability.usedTruePoseFeedback);
assert(~comparison.identifiedTruthFeedback.observability.usedTrueForceFeedback);
assert(comparison.identifiedTruthFeedback.observability.usedTruePoseFeedback);
assert(comparison.identifiedTruthFeedback.observability.usedTrueVelocityFeedback);
assert(comparison.identifiedTruthFeedback.observability.usedIdentifiedForceFeedback);
assert(~comparison.identifiedTruthFeedback.observability.usedEncoderImuFusion);
assert(~comparison.identifiedTruthFeedback.observability.usedUkfPoseFusion);
assert(max(abs(comparison.identifiedTruthFeedback.qFeedback - ...
    comparison.identifiedTruthFeedback.qTrue), [], 'all') < 1e-12);
assert(max(abs(comparison.identifiedTruthFeedback.qdFeedback - ...
    comparison.identifiedTruthFeedback.qdTrue), [], 'all') < 1e-12);
assert(max(abs(comparison.identifiedTruthFeedback.legSpeed - ...
    comparison.identifiedTruthFeedback.trueLegSpeed), [], 'all') < 1e-12);
assert(~comparison.identified.observability.usedTrueForceFeedback);
assert(~comparison.identified.observability.usedTruePoseFeedback);
assert(comparison.identified.observability.usedEncoderImuFusion);
assert(~comparison.identified.observability.usedEncoderDifferencedLegSpeed);
assert(~comparison.identified.observability.usedTrueLegSpeedFeedback);
assert(comparison.identified.observability.usedFeedbackPoseJacobian);
assert(comparison.identified.observability.usedFusedPoseLegSpeed);
assert(any(abs(comparison.identified.legSpeed - comparison.identified.trueLegSpeed) > 1e-9, 'all'));
assert(all(isfinite(comparison.oracle.trueForce), 'all'));
assert(all(isfinite(comparison.identifiedTruthFeedback.trueForce), 'all'));
assert(all(isfinite(comparison.identifiedTruthFeedback.estimatedForce), 'all'));
assert(all(isfinite(comparison.identified.trueForce), 'all'));
assert(all(isfinite(comparison.identified.estimatedForce), 'all'));
assert(isfield(comparison.metrics, 'identifiedTruthFeedbackForceTrackingNrmse'));
assert(isfield(comparison.metrics, 'ukfTranslationRmsPenalty'));
assert(comparison.metrics.identifiedForceEstimateAlignmentSamples == 1);
assert(isfinite(comparison.metrics.identifiedAlignedForceEstimateRms));
assert(isfinite(comparison.metrics.identifiedSameSampleForceEstimateRms));
assert(comparison.metrics.identifiedForceTrackingNrmse <= 0.075);

ukfComparison = comparePwmPoseForceControlLines(sample.refs, teacher, identified, struct( ...
    'duration', 0.8, 'poseEstimatorMode', "ukf", 'sensorNoiseEnabled', true));
assert(ukfComparison.identified.observability.usedUkfPoseFusion);
assert(~ukfComparison.identified.observability.usedTruePoseFeedback);
assert(ukfComparison.identified.observability.usedRelativeEncoder);
assert(ukfComparison.identified.observability.usedImuAcceleration);
assert(ukfComparison.identified.observability.usedImuAngularVelocity);
assert(~ukfComparison.identified.observability.usedPositionMeasurement);
assert(ukfComparison.identified.observability.usedHomeCalibration);
assert(all(isfinite(ukfComparison.identified.qFeedback), 'all'));
end
