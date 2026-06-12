function test_17_pose_force_control_contract
% test_17_pose_force_control_contract - 验证力输入位姿跟踪配置、综合分与模型 Variant
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

model = buildOptModelCustom();
config = makeSimscapePoseForceConfig(model, struct( ...
    'bandwidthHz', 8, 'gainScale', 0.8, 'rotationGainScale', 1.2));
assert(config.bandwidthHz == 8);
assert(config.gainScale == 0.8);
assert(config.rotationGainScale == 1.2);
assert(config.gravityEnabled);
assert(config.characteristicLength == model.Lc);

candidate = makeReport(2e-3, 8e-4, 2e-3);
baseline = makeReport(2.5e-3, 1e-3, 2.5e-3);
comparison = comparePoseTrackingPerformance(candidate, baseline, model.Lc);
assert(comparison.score < 1);
assert(comparison.passed);
assert(comparison.translationPeakRatio == 0.8);
assert(comparison.rotationPeakRatio == 0.8);

modelFile = fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx');
load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_platform_model', 0));
variant = 'stewart_platform_model/Controller/Reference-Tracking-X';
assert(getSimulinkBlockHandle(variant) ~= -1);
assert(strcmp(get_param(variant, 'VariantControl'), 'controller.type == 6'));
end

function report = makeReport(lengthPeak, translationPeak, rotationPeak)
poseError = [
    translationPeak, 0, 0, rotationPeak, 0, 0
    translationPeak / 2, 0, 0, rotationPeak / 2, 0, 0];
report = struct();
report.passed = true;
report.poseError = poseError;
report.metrics = struct( ...
    'maxLengthTrackingPeak', lengthPeak, ...
    'maxTranslationPeak', translationPeak, ...
    'maxRotationPeak', rotationPeak);
end
