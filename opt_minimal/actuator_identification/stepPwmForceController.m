function [state, pwmCommand, diagnostics] = stepPwmForceController( ...
    state, targetForce, feedbackForce, legSpeed, controller)
% stepPwmForceController - 灰箱逆前馈加力反馈 PI 生成有符号 PWM
targetForce = targetForce(:);
feedbackForce = feedbackForce(:);
legSpeed = legSpeed(:);
model = controller.inverseModel;

friction = approximateFriction(legSpeed, targetForce, model);
forceGain = model.torqueConstant .* model.gearRatio .* ...
    (2 * pi ./ model.screwLead) .* model.efficiency;
desiredCurrent = (targetForce + friction) ./ forceGain;
motorSpeed = legSpeed .* (2 * pi .* model.gearRatio ./ model.screwLead);
desiredVoltage = model.resistance .* desiredCurrent + model.backEmfConstant .* motorSpeed;
duty = min(max(desiredVoltage ./ model.busVoltage, -1), 1);
feedforwardPwm = duty .* model.pwmMax + sign(duty) .* model.deadzonePwm;

forceError = targetForce - feedbackForce;
candidateIntegral = state.integralError + controller.sampleTime * forceError;
candidateIntegral = min(max(candidateIntegral, -controller.integralLimit), controller.integralLimit);
feedbackPwm = controller.kpPwmPerNewton .* forceError + ...
    controller.kiPwmPerNewtonSecond .* candidateIntegral;
rawPwm = feedforwardPwm + feedbackPwm;
saturatedPwm = min(max(rawPwm, -controller.pwmMax), controller.pwmMax);

rateDelta = min(max(saturatedPwm - state.previousPwm, ...
    -controller.pwmRateLimit), controller.pwmRateLimit);
pwmCommand = state.previousPwm + rateDelta;
notSaturated = abs(rawPwm - saturatedPwm) < 1e-9;
state.integralError(notSaturated) = candidateIntegral(notSaturated);
state.previousPwm = pwmCommand;

diagnostics = struct('forceError', forceError, 'feedforwardPwm', feedforwardPwm, ...
    'feedbackPwm', feedbackPwm, 'rawPwm', rawPwm, 'saturatedPwm', saturatedPwm);
end

function friction = approximateFriction(speed, targetForce, model)
direction = sign(speed);
stopped = abs(speed) < 1e-8;
direction(stopped) = sign(targetForce(stopped));
friction = direction .* model.coulombFriction + model.viscousFriction .* speed;
end
