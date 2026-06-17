function test_28_pwm_actuator_mapping_and_switching
% test_28_pwm_actuator_mapping_and_switching - 验证 PWM 参数映射与开关级校验
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
simscapeData = buildSimscapeLengthControlData(model, scene, 'pwm-force');
assert(simscapeData.stewart.actuators.type == 6);
assert(strcmp(simscapeData.pwmActuator.type, 'high-fidelity-pwm-actuator'));
assert(simscapeData.stewart.pwmActuator.pwmMax == 4198);

report = validateAveragePwmActuatorAgainstSwitching(simscapeData.pwmActuator, struct( ...
    'axisIndex', 1, 'duty', 0.7, 'duration', 0.15, 'legSpeed', 0.05));
assert(report.metrics.relativeMeanForceError <= 0.02);
assert(report.passed);

gridReport = validateAveragePwmActuatorAcrossGrid(simscapeData.pwmActuator);
assert(gridReport.metrics.maxNormalizedForceError <= 0.006);
assert(gridReport.passed);
end
