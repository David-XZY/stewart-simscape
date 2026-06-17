function [state, output] = stepHighFidelityPwmActuator(state, pwmCommand, legSpeed, actuator)
% stepHighFidelityPwmActuator - 推进平均值 H 桥、电机与丝杠执行器一步
pwmCommand = min(max(pwmCommand(:), -actuator.pwmMax), actuator.pwmMax);
legSpeed = legSpeed(:);
appliedPwm = applyDeadzone(pwmCommand, actuator.deadzonePwm);
duty = appliedPwm ./ actuator.pwmMax;
motorSpeed = legSpeed .* (2 * pi .* actuator.gearRatio ./ actuator.screwLead);
voltage = actuator.busVoltage .* duty;
steadyCurrent = (voltage - actuator.backEmfConstant .* motorSpeed) ./ actuator.resistance;
electricalDecay = exp(-actuator.resistance .* actuator.sampleTime ./ actuator.inductance);
state.current = steadyCurrent + (state.current - steadyCurrent) .* electricalDecay;
state.current = min(max(state.current, -actuator.currentLimit), actuator.currentLimit);

forceGain = actuator.torqueConstant .* actuator.gearRatio .* ...
    (2 * pi ./ actuator.screwLead) .* actuator.efficiency;
electromagneticForce = forceGain .* state.current;
frictionForce = computeFriction(legSpeed, electromagneticForce, actuator);
force = min(max(electromagneticForce - frictionForce, ...
    -actuator.forceLimit), actuator.forceLimit);
force(appliedPwm == 0 & abs(legSpeed) < 1e-12) = 0;

state.previousAppliedPwm = appliedPwm;
output = struct('force', force, 'electromagneticForce', electromagneticForce, ...
    'frictionForce', frictionForce, 'current', state.current, ...
    'motorSpeed', motorSpeed, 'appliedPwm', appliedPwm, 'duty', duty);
end

function appliedPwm = applyDeadzone(command, deadzone)
magnitude = abs(command);
appliedPwm = sign(command) .* max(magnitude - deadzone, 0);
end

function friction = computeFriction(speed, driveForce, actuator)
direction = sign(speed);
stopped = abs(speed) < 1e-8;
direction(stopped) = sign(driveForce(stopped));
stribeck = actuator.coulombFriction + ...
    (actuator.staticFriction - actuator.coulombFriction) .* ...
    exp(-(abs(speed) ./ actuator.stribeckVelocity).^2);
friction = direction .* stribeck + actuator.viscousFriction .* speed;
friction(stopped & abs(driveForce) <= actuator.staticFriction) = driveForce( ...
    stopped & abs(driveForce) <= actuator.staticFriction);
end
