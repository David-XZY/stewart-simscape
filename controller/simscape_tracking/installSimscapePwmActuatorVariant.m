function installSimscapePwmActuatorVariant(projectRoot)
% installSimscapePwmActuatorVariant - 在现有支链模型中安装平均值 PWM 物理执行器分支
arguments
    projectRoot {mustBeTextScalar} = ""
end
if strlength(string(projectRoot)) == 0
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
modelFile = fullfile(char(projectRoot), 'matlab', 'simscape_subsystems', 'stewart_strut.slx');
load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_strut', 0));

source = 'stewart_strut/Actuator/Classical';
target = 'stewart_strut/Actuator/PWM-Physical';
if getSimulinkBlockHandle(target) ~= -1
    delete_block(target);
end
add_block(source, target, 'CopyOption', 'nolink');
set_param(target, 'VariantControl', 'stewart.actuators.type==6');

converter = find_system(target, 'SearchDepth', 1, ...
    'LookUnderMasks', 'all', 'MaskType', 'Simulink-PS Converter');
inputBlock = [target, '/Fi'];
delete_line(get_param(inputBlock, 'LineHandles').Outport);

add_block('simulink/Discontinuities/Dead Zone', [target, '/PWM Dead Zone'], ...
    'LowerValue', '-stewart.pwmActuator.deadzonePwm(i)', ...
    'UpperValue', 'stewart.pwmActuator.deadzonePwm(i)', ...
    'Position', [275 405 365 455]);
add_block('simulink/Math Operations/Gain', [target, '/PWM Voltage'], ...
    'Gain', 'stewart.pwmActuator.busVoltage(i)/stewart.pwmActuator.pwmMax', ...
    'Position', [390 405 480 455]);
add_block('simulink/Continuous/Transfer Fcn', [target, '/Electrical Dynamics'], ...
    'Numerator', '1', ...
    'Denominator', '[stewart.pwmActuator.inductance(i), stewart.pwmActuator.resistance(i)]', ...
    'Position', [505 405 615 455]);
add_block('simulink/Math Operations/Gain', [target, '/Electromechanical Force'], ...
    'Gain', ['stewart.pwmActuator.torqueConstant(i)*stewart.pwmActuator.gearRatio(i)*', ...
    '2*pi/stewart.pwmActuator.screwLead(i)*stewart.pwmActuator.efficiency(i)'], ...
    'Position', [640 405 750 455]);
add_block('simulink/Discontinuities/Saturation', [target, '/Force Saturation'], ...
    'UpperLimit', 'stewart.pwmActuator.forceLimit(i)', ...
    'LowerLimit', '-stewart.pwmActuator.forceLimit(i)', ...
    'Position', [775 405 875 455]);

add_line(target, 'Fi/1', 'PWM Dead Zone/1');
add_line(target, 'PWM Dead Zone/1', 'PWM Voltage/1');
add_line(target, 'PWM Voltage/1', 'Electrical Dynamics/1');
add_line(target, 'Electrical Dynamics/1', 'Electromechanical Force/1');
add_line(target, 'Electromechanical Force/1', 'Force Saturation/1');
add_line(target, 'Force Saturation/1', [get_param(converter{1}, 'Name'), '/1']);

save_system('stewart_strut', modelFile);
clear cleanup;
end
