function test_30_pwm_full_tracking
% test_30_pwm_full_tracking - 验证辨识反馈线完整轨迹硬验收
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct('sampleCount', 3000, 'randomSeed', 30));
initialIdentifier = trainGrayNarxForceIdentifier(dataset, teacher);
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');
identified = refineGrayNarxForceIdentifierWithControlRollout( ...
    dataset, teacher, initialIdentifier, sample.refs, makePwmClosedLoopRefinementOptions());
assert(identified.training.closedLoopRefinement.usedDistinctReference);
assert(identified.training.closedLoopRefinement.addedSampleCount > 0);
assert(identified.training.closedLoopRefinement.usedUkfPoseFusion);

comparison = comparePwmPoseForceControlLines(sample.refs, teacher, identified);
report = evaluatePwmPoseForceControlComparison(comparison, teacher);
oracleAxisPeak = max(abs(comparison.oracle.qTrue(1:3, :) - ...
    comparison.oracle.qReference(1:3, :)), [], 2);
assert(report.passed);
assert(report.metrics.identifiedForceTrackingNrmse <= 0.075);
assert(all(report.metrics.identifiedTranslationAxisPeak <= 0.005));
assert(all(report.metrics.estimatorSettledTranslationAxisPeak <= 0.002));
assert(report.metrics.estimatorVelocityRms <= 0.01);
assert(report.metrics.identifiedAlignedForceEstimateRms <= 75);
assert(report.metrics.identifiedRotationPeak <= deg2rad(1));
assert(all(oracleAxisPeak <= 0.00035));
end
