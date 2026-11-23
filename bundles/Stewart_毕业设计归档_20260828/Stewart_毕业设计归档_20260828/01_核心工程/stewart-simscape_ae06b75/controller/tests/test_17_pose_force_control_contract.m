function test_17_pose_force_control_contract
% test_17_pose_force_control_contract - 验证力输入位姿跟踪配置、综合分与模型 Variant
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
config = makeSimscapePoseForceConfig(model, struct( ...
    'bandwidthHz', 8, 'gainScale', 0.8, 'rotationGainScale', 1.2));
assert(config.bandwidthHz == 8);
assert(config.gainScale == 0.8);
assert(config.rotationGainScale == 1.2);
assert(config.controlLaw == "computed-torque");
assert(config.feedbackLaw == "lqi-output");
assert(config.actuatorMode == "nonideal-force");
assert(config.gravityEnabled);
assert(config.characteristicLength == model.Lc);
assert(config.lqiTranslationScale > 0);
assert(config.lqiRotationScale > 0);
assert(config.lqiVelocityScale > 0);
assert(config.lqiIntegralScale > 0);
assert(config.lqiIntegralLeakHz > 0);

legacyConfig = makeSimscapePoseForceConfig(model, struct( ...
    'controlLaw', "linear-pose-force", 'actuatorMode', "ideal-force"));
assert(legacyConfig.controlLaw == "linear-pose-force");
assert(legacyConfig.feedbackLaw == "lqi-output");
assert(legacyConfig.actuatorMode == "ideal-force");

try
    makeSimscapePoseForceConfig(model, struct('feedbackLaw', "unsupported-feedback"));
    error('test_17_pose_force_control_contract:UnsupportedFeedbackAccepted', ...
        'feedbackLaw 不应接受未声明的反馈律。');
catch errorInfo
    assert(~strcmp(errorInfo.identifier, ...
        'test_17_pose_force_control_contract:UnsupportedFeedbackAccepted'));
end

sourceFiles = {
    fullfile(controllerRoot, 'simscape_tracking', 'designSimscapePoseForceController.m')
    fullfile(controllerRoot, 'simscape_tracking', 'designLqiPoseForceController.m')
    };
for sourceIndex = 1:numel(sourceFiles)
    sourceText = fileread(sourceFiles{sourceIndex});
    assert(isempty(regexpi(sourceText, 'stabilizerControllers')), ...
        'CT-LQI 主线源码中不允许出现旧回退补偿器。');
end

computedController = initializeController('type', 'computed-torque-force');
assert(computedController.type == 10);

q = model.qHome;
qd = zeros(6, 1);
qdd = [0.01; -0.02; 0.03; 0.001; -0.002; 0.003];
targetForce = computeComputedTorqueForce(q, qd, qdd, q, qd, model, config);
expectedForce = inverseDynamicsCompositeRigidBody(q, qd, qdd, model);
assert(max(abs(targetForce - expectedForce)) < 1e-9);

simscapeData = buildSimscapeLengthControlData(model, buildCylinderBoxTransferScene(model), ...
    'nonideal-force');
assert(simscapeData.stewart.actuators.type == 7);
assert(strcmp(simscapeData.nonidealForceActuator.type, 'nonideal-force-actuator'));
assert(isfield(simscapeData.stewart, 'nonidealForceActuator'));

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
variant = 'stewart_platform_model/Controller/Computed-Torque-Force';
assert(getSimulinkBlockHandle(variant) ~= -1);
assert(strcmp(get_param(variant, 'VariantControl'), 'controller.type == 10'));
assert(getSimulinkBlockHandle([variant, '/LQI Error Derivative']) == -1);
assert(getSimulinkBlockHandle([variant, '/LQI Feedback Vector']) ~= -1);
assert(getSimulinkBlockHandle([variant, '/LQI Feedback Limit']) ~= -1);
assert(getSimulinkBlockHandle([variant, '/LQI Anti Windup Difference']) ~= -1);

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
