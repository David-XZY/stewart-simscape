function test_26_pwm_force_controller_contract
% test_26_pwm_force_controller_contract - 验证高保真/灰箱双线力内环契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct('sampleCount', 1200, 'randomSeed', 26));
identified = trainGrayNarxForceIdentifier(dataset, teacher);

oracle = makePwmForceController('oracle', teacher);
gray = makePwmForceController('identified', identified);
assert(strcmp(oracle.feedbackSignal, 'trueForce'));
assert(strcmp(gray.feedbackSignal, 'estimatedForce'));
assert(~isfield(gray, 'teacher'), '灰箱控制器不得持有高保真教师模型。');

oracleState = initializePwmForceControllerState(oracle);
grayState = initializePwmForceControllerState(gray);
targetForce = 900 * ones(6, 1);
legSpeed = 0.1 * ones(6, 1);
[oracleState, oraclePwm] = stepPwmForceController( ...
    oracleState, targetForce, zeros(6, 1), legSpeed, oracle);
[grayState, grayPwm] = stepPwmForceController( ...
    grayState, targetForce, zeros(6, 1), legSpeed, gray);
assert(all(oraclePwm > 0));
assert(all(grayPwm > 0));
assert(all(abs(oraclePwm) <= teacher.pwmMax));
assert(all(abs(grayPwm) <= teacher.pwmMax));

[~, limitedPwm] = stepPwmForceController( ...
    oracleState, -targetForce, zeros(6, 1), legSpeed, oracle);
assert(all(abs(limitedPwm - oraclePwm) <= oracle.pwmRateLimit + 1e-9));
end
