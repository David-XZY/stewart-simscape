function test_31_pwm_pose_estimator_benchmark
% test_31_pwm_pose_estimator_benchmark - 验证运动学重建与 UKF 位姿融合公平基准
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct('sampleCount', 1200, 'randomSeed', 31));
identified = trainGrayNarxForceIdentifier(dataset, teacher);
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');
comparison = comparePwmPoseForceControlLines(sample.refs, teacher, identified, ...
    struct('duration', 0.8));

benchmark = benchmarkPwmPoseEstimators(comparison.identified);
assert(benchmark.sensorModel.usesNoisyOrientation);
assert(~benchmark.sensorModel.usesFullPoseMeasurement);
assert(~benchmark.sensorModel.usesPositionMeasurement);
assert(benchmark.sensorModel.usesRelativeEncoder);
assert(isfield(benchmark.measurements, 'relativeLength'));
assert(~isfield(benchmark.measurements, 'pose'));
assert(~isfield(benchmark.measurements, 'position'));
assert(isfield(benchmark, 'ukfWorld'));
assert(isfield(benchmark, 'ukfSpecificForce'));
assert(benchmark.metrics.ukfVelocityRms < benchmark.metrics.kinematicVelocityRms);
assert(all(isfinite(benchmark.ukf.pose), 'all'));
assert(all(isfinite(benchmark.ukf.velocity), 'all'));
end
