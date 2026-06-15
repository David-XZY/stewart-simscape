function dataset = generatePwmIdentificationDataset(teacher, overrides)
% generatePwmIdentificationDataset - 生成包含现实采集量与隐藏真值的辨识数据
arguments
    teacher struct
    overrides struct = struct()
end
options = struct('sampleCount', 6000, 'randomSeed', 1);
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('generatePwmIdentificationDataset:UnknownOverride', ...
            '未知辨识数据配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end
validateattributes(options.sampleCount, {'double'}, {'scalar', 'integer', 'positive'});
validateattributes(options.randomSeed, {'double'}, {'scalar', 'integer'});
rng(options.randomSeed);
count = options.sampleCount;
t = (0:count - 1).' * teacher.sampleTime;
axisPhase = linspace(0, pi, 6);

pwm = zeros(count, 6);
for axisIndex = 1:6
    multisine = 1150 * sin(2 * pi * 0.35 * t + axisPhase(axisIndex)) + ...
        900 * sin(2 * pi * 1.1 * t + 0.7 * axisPhase(axisIndex)) + ...
        520 * sin(2 * pi * 2.8 * t + 1.3 * axisPhase(axisIndex));
    block = 80;
    randomSteps = repelem(1200 * sign(randn(ceil(count / block), 1)), block);
    pwm(:, axisIndex) = multisine + randomSteps(1:count);
    pwm(:, axisIndex) = min(max(pwm(:, axisIndex), -teacher.pwmMax), teacher.pwmMax);
end
[platformPose, platformVelocity, trueLegSpeed, encoderLength, measuredPose, legSpeed] = ...
    buildPlatformFeasibleMotion(t, teacher.sampleTime);

state = initializeHighFidelityPwmActuatorState(teacher);
trueForce = zeros(count, 6);
appliedPwm = zeros(count, 6);
current = zeros(count, 6);
for sampleIndex = 1:count
    [state, sample] = stepHighFidelityPwmActuator( ...
        state, pwm(sampleIndex, :).', trueLegSpeed(sampleIndex, :).', teacher);
    trueForce(sampleIndex, :) = sample.force.';
    appliedPwm(sampleIndex, :) = sample.appliedPwm.';
    current(sampleIndex, :) = sample.current.';
end

legAcceleration = [zeros(1, 6); diff(legSpeed) / teacher.sampleTime];

trainEnd = floor(0.60 * count);
validationEnd = floor(0.80 * count);
split = struct();
split.train = false(count, 1);
split.validation = false(count, 1);
split.test = false(count, 1);
split.train(1:trainEnd) = true;
split.validation(trainEnd + 1:validationEnd) = true;
split.test(validationEnd + 1:end) = true;

dataset = struct('sampleTime', teacher.sampleTime, 'time', t, 'pwm', pwm, ...
    'appliedPwm', appliedPwm, 'encoderLength', encoderLength, ...
    'legSpeed', legSpeed, 'trueLegSpeed', trueLegSpeed, ...
    'legAcceleration', legAcceleration, ...
    'measuredPose', measuredPose, 'platformPose', platformPose, ...
    'platformVelocity', platformVelocity, 'trueForce', trueForce, ...
    'hiddenCurrent', current, 'split', split, ...
    'description', 'PWM、编码器与位姿可观测量；真实力和电流仅用于教师评价。');
end

function [pose, velocity, trueLegSpeed, encoderLength, measuredPose, measuredLegSpeed] = ...
        buildPlatformFeasibleMotion(time, sampleTime)
model = buildOptModelCustom();
count = numel(time);
amplitude = [0.018; 0.016; 0.055; 0.035; 0.030; 0.040];
frequency = [0.19; 0.23; 0.17; 0.31; 0.27; 0.21];
secondaryAmplitude = 0.30 * amplitude;
secondaryFrequency = [0.71; 0.63; 0.57; 0.83; 0.77; 0.69];
tertiaryAmplitude = 0.20 * amplitude;
tertiaryFrequency = [1.70; 1.90; 2.10; 2.20; 1.80; 2.00];
pose = zeros(count, 6);
velocity = zeros(count, 6);
for dofIndex = 1:6
    pose(:, dofIndex) = model.qHome(dofIndex) + ...
        amplitude(dofIndex) * (1 - cos(2 * pi * frequency(dofIndex) * time)) + ...
        secondaryAmplitude(dofIndex) * (1 - cos(2 * pi * secondaryFrequency(dofIndex) * time)) + ...
        tertiaryAmplitude(dofIndex) * (1 - cos(2 * pi * tertiaryFrequency(dofIndex) * time));
    velocity(:, dofIndex) = ...
        amplitude(dofIndex) * 2 * pi * frequency(dofIndex) * ...
        sin(2 * pi * frequency(dofIndex) * time) + ...
        secondaryAmplitude(dofIndex) * 2 * pi * secondaryFrequency(dofIndex) * ...
        sin(2 * pi * secondaryFrequency(dofIndex) * time) + ...
        tertiaryAmplitude(dofIndex) * 2 * pi * tertiaryFrequency(dofIndex) * ...
        sin(2 * pi * tertiaryFrequency(dofIndex) * time);
end

trueLegSpeed = zeros(count, 6);
encoderLength = zeros(count, 6);
for sampleIndex = 1:count
    jacobian = sgpJacobian(pose(sampleIndex, :).', model);
    trueLegSpeed(sampleIndex, :) = (jacobian.Jq * velocity(sampleIndex, :).').';
    encoderLength(sampleIndex, :) = sgpIK(pose(sampleIndex, :).', model).L.';
end
encoderLength = round(encoderLength * 16500) / 16500;
measuredPose = pose;
measuredPose(:, 1:3) = round(measuredPose(:, 1:3) / 1e-5) * 1e-5;
measuredPose(:, 4:6) = round(measuredPose(:, 4:6) / 1e-5) * 1e-5;
measuredVelocity = [zeros(1, 6); diff(measuredPose) / sampleTime];
rawMeasuredLegSpeed = zeros(count, 6);
for sampleIndex = 1:count
    jacobian = sgpJacobian(measuredPose(sampleIndex, :).', model);
    rawMeasuredLegSpeed(sampleIndex, :) = ...
        (jacobian.Jq * measuredVelocity(sampleIndex, :).').';
end
measuredLegSpeed = lowpassEncoderSpeed(rawMeasuredLegSpeed, 0.6);
end

function filtered = lowpassEncoderSpeed(rawSpeed, alpha)
filtered = zeros(size(rawSpeed));
for sampleIndex = 2:size(rawSpeed, 1)
    filtered(sampleIndex, :) = alpha * filtered(sampleIndex - 1, :) + ...
        (1 - alpha) * rawSpeed(sampleIndex, :);
end
end
