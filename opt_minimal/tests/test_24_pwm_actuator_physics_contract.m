function test_24_pwm_actuator_physics_contract
% test_24_pwm_actuator_physics_contract - 验证高保真 PWM 执行器物理内核
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'actuator_identification'));

actuator = makeHighFidelityPwmActuator();
assert(actuator.axisCount == 6);
assert(actuator.sampleTime == 0.005);
assert(actuator.pwmMax == 4198);
assert(all(actuator.forceLimit == 2400));
assert(all(actuator.deadzonePwm > 0));
assert(numel(unique(actuator.deadzonePwm)) > 1, '六轴必须保留参数离散性。');

state = initializeHighFidelityPwmActuatorState(actuator);
[state, zeroOutput] = stepHighFidelityPwmActuator( ...
    state, zeros(6, 1), zeros(6, 1), actuator);
assert(max(abs(zeroOutput.force)) < 1e-12);
assert(all(zeroOutput.appliedPwm == 0));

[~, positiveOutput] = stepHighFidelityPwmActuator( ...
    state, actuator.pwmMax * ones(6, 1), zeros(6, 1), actuator);
[~, negativeOutput] = stepHighFidelityPwmActuator( ...
    state, -actuator.pwmMax * ones(6, 1), zeros(6, 1), actuator);
assert(all(positiveOutput.force > 0));
assert(all(negativeOutput.force < 0));
assert(all(abs(positiveOutput.force) <= actuator.forceLimit + 1e-9));
assert(all(abs(negativeOutput.force) <= actuator.forceLimit + 1e-9));

deadzoneCommand = 0.5 * min(actuator.deadzonePwm) * ones(6, 1);
[~, deadzoneOutput] = stepHighFidelityPwmActuator( ...
    state, deadzoneCommand, zeros(6, 1), actuator);
assert(max(abs(deadzoneOutput.force)) < 1e-12);
assert(all(deadzoneOutput.appliedPwm == 0));
end
