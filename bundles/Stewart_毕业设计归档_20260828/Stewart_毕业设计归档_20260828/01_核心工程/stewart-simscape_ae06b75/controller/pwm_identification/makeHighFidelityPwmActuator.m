function actuator = makeHighFidelityPwmActuator(overrides)
% makeHighFidelityPwmActuator - 构造六轴高保真平均值 PWM 执行器参数
arguments
    overrides struct = struct()
end

axisScale = [1.00; 0.97; 1.03; 0.95; 1.02; 0.98];
actuator = struct();
actuator.type = 'high-fidelity-pwm-actuator';
actuator.axisCount = 6;
actuator.sampleTime = 0.005;
actuator.integrationSubsteps = 10;
actuator.pwmMax = 4198;
actuator.busVoltage = 48 * ones(6, 1);
actuator.deadzonePwm = [620; 700; 660; 760; 680; 640];
actuator.resistance = 1.2 ./ axisScale;
actuator.inductance = 0.025 * ones(6, 1);
actuator.torqueConstant = 0.12 * axisScale;
actuator.backEmfConstant = 0.04 ./ axisScale;
actuator.gearRatio = 2 * ones(6, 1);
actuator.screwLead = 0.02 * ones(6, 1);
actuator.efficiency = 0.86 * axisScale;
actuator.forceLimit = 2400 * ones(6, 1);
actuator.coulombFriction = [72; 82; 68; 91; 76; 80];
actuator.staticFriction = actuator.coulombFriction + [48; 55; 44; 62; 50; 53];
actuator.stribeckVelocity = [0.018; 0.020; 0.017; 0.022; 0.019; 0.020];
actuator.viscousFriction = [85; 92; 80; 98; 88; 90];
actuator.currentLimit = 45 * ones(6, 1);

fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(actuator, fields{index})
        error('makeHighFidelityPwmActuator:UnknownOverride', ...
            '未知高保真执行器参数：%s。', fields{index});
    end
    actuator.(fields{index}) = overrides.(fields{index});
end
end
