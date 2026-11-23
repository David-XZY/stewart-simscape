function p=ch5ActuatorProfile(mode,c,m)
p=struct('mode',string(mode),'forceMin',m.actuator.forceMin,'forceMax',m.actuator.forceMax, ...
    'slewRate',ones(6,1)*8000/c.Ts,'tau',.001,'delay',1e-6,'deadzone',zeros(6,1));
end
