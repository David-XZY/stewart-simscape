function report = validateAveragePwmActuatorAgainstSwitching(actuator, overrides)
% validateAveragePwmActuatorAgainstSwitching - 单轴开关级电气模型校验平均值模型
arguments
    actuator struct
    overrides struct = struct()
end
options = struct('axisIndex', 1, 'duty', 0.7, 'duration', 0.15, 'legSpeed', 0.05);
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('validateAveragePwmActuatorAgainstSwitching:UnknownOverride', ...
            '未知开关级校验配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end
axisIndex = options.axisIndex;
pwm = options.duty * actuator.pwmMax;
state = initializeHighFidelityPwmActuatorState(actuator);
sampleCount = round(options.duration / actuator.sampleTime);
averageForce = zeros(sampleCount, 1);
speed = zeros(6, 1);
speed(axisIndex) = options.legSpeed;
command = zeros(6, 1);
command(axisIndex) = pwm;
for index = 1:sampleCount
    [state, output] = stepHighFidelityPwmActuator(state, command, speed, actuator);
    averageForce(index) = output.force(axisIndex);
end

switchingForce = simulateSwitchingAxis(actuator, axisIndex, pwm, options.legSpeed, options.duration);
tailStartAverage = max(1, floor(0.7 * numel(averageForce)));
tailStartSwitching = max(1, floor(0.7 * numel(switchingForce)));
meanAverage = mean(averageForce(tailStartAverage:end));
meanSwitching = mean(switchingForce(tailStartSwitching:end));
relativeError = abs(meanAverage - meanSwitching) / max(abs(meanSwitching), 1);

report = struct();
report.averageForce = averageForce;
report.switchingForce = switchingForce;
report.metrics = struct('meanAverageForce', meanAverage, ...
    'meanSwitchingForce', meanSwitching, 'relativeMeanForceError', relativeError);
report.passed = relativeError <= 0.02;
end

function force = simulateSwitchingAxis(actuator, axisIndex, pwm, legSpeed, duration)
carrierFrequency = 5000;
carrierResolution = 400;
timeStep = 1 / (carrierFrequency * carrierResolution);
count = ceil(duration / timeStep);
duty = max((abs(pwm) - actuator.deadzonePwm(axisIndex)) / actuator.pwmMax, 0);
direction = sign(pwm);
current = 0;
force = zeros(count, 1);
motorSpeed = legSpeed * 2 * pi * actuator.gearRatio(axisIndex) / actuator.screwLead(axisIndex);
gain = actuator.torqueConstant(axisIndex) * actuator.gearRatio(axisIndex) * ...
    2 * pi / actuator.screwLead(axisIndex) * actuator.efficiency(axisIndex);
for index = 1:count
    carrierPhase = mod(index - 1, carrierResolution) / carrierResolution;
    voltage = direction * actuator.busVoltage(axisIndex) * double(carrierPhase < duty);
    currentRate = (voltage - actuator.resistance(axisIndex) * current - ...
        actuator.backEmfConstant(axisIndex) * motorSpeed) / actuator.inductance(axisIndex);
    current = current + timeStep * currentRate;
    current = min(max(current, -actuator.currentLimit(axisIndex)), actuator.currentLimit(axisIndex));
    electromagnetic = gain * current;
    friction = sign(legSpeed) * actuator.coulombFriction(axisIndex) + ...
        actuator.viscousFriction(axisIndex) * legSpeed;
    force(index) = min(max(electromagnetic - friction, ...
        -actuator.forceLimit(axisIndex)), actuator.forceLimit(axisIndex));
end
end
