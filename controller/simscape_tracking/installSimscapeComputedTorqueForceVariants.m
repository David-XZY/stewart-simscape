function installSimscapeComputedTorqueForceVariants(projectRoot)
% installSimscapeComputedTorqueForceVariants - 安装计算力矩控制和非理想力执行器 Variant
arguments
    projectRoot {mustBeTextScalar} = ""
end

if strlength(string(projectRoot)) == 0
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
projectRoot = char(projectRoot);
installReferenceSignal(projectRoot);
installControllerVariant(projectRoot);
installActuatorVariant(projectRoot);
end

function installReferenceSignal(projectRoot)
modelFile = fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx');
load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_platform_model', 0));

referenceBlock = 'stewart_platform_model/Reference';
target = [referenceBlock, '/Computed Torque Force Reference'];
if ~blockExists(target)
    add_block('simulink/Sources/From Workspace', target, ...
        'VariableName', 'references.uCT', ...
        'Position', [55 275 190 305]);
end
busCreator = [referenceBlock, '/Bus Creator'];
set_param(busCreator, 'Inputs', '5');
lineHandles = get_param(target, 'LineHandles');
if lineHandles.Outport == -1
    signalLine = add_line(referenceBlock, ...
        'Computed Torque Force Reference/1', 'Bus Creator/5');
else
    signalLine = lineHandles.Outport;
end
set_param(signalLine, 'Name', 'uCT');

feedforwardVariant = 'stewart_platform_model/Actuator Feedforward Variant';
if blockExists(feedforwardVariant)
    set_param(feedforwardVariant, 'VariantControls', ...
        {'stewart.actuators.type ~= 5', ...
         'stewart.actuators.type == 5'});
end

save_system('stewart_platform_model', modelFile);
clear cleanup;
end

function installControllerVariant(projectRoot)
modelFile = fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx');
load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_platform_model', 0));

source = 'stewart_platform_model/Controller/Reference-Tracking-X';
target = 'stewart_platform_model/Controller/Computed-Torque-Force';
if blockExists(target)
    delete_block(target);
end
add_block(source, target, 'CopyOption', 'nolink');
set_param(target, 'VariantControl', 'controller.type == 10');
installLqiFeedbackInput(target);

save_system('stewart_platform_model', modelFile);
clear cleanup;
end

function installLqiFeedbackInput(target)
errorMux = [target, '/Mux'];
gainBlock = [target, '/Gain'];
ltiBlock = [target, '/LTI System'];
derivativeBlock = [target, '/LQI Error Derivative'];
feedbackMux = [target, '/LQI Feedback Vector'];
feedbackLimit = [target, '/LQI Feedback Limit'];
antiWindupSum = [target, '/LQI Anti Windup Difference'];

deleteConnectedLines(ltiBlock);
deleteConnectedLines(gainBlock);
if blockExists(derivativeBlock)
    delete_block(derivativeBlock);
end
if blockExists(feedbackMux)
    delete_block(feedbackMux);
end
if blockExists(feedbackLimit)
    delete_block(feedbackLimit);
end
if blockExists(antiWindupSum)
    delete_block(antiWindupSum);
end
add_block('simulink/Signal Routing/Mux', feedbackMux, ...
    'Inputs', '2', ...
    'Position', [650 285 675 365]);
add_block('simulink/Discontinuities/Saturation', feedbackLimit, ...
    'UpperLimit', 'computedTorqueConfig.lqiFeedbackForceLimit', ...
    'LowerLimit', '-computedTorqueConfig.lqiFeedbackForceLimit', ...
    'Position', [840 295 940 345]);
add_block('simulink/Math Operations/Sum', antiWindupSum, ...
    'Inputs', '+-', ...
    'Position', [975 365 1005 415]);
add_line(target, 'Mux/1', 'Gain/1');
add_line(target, 'Gain/1', 'LQI Feedback Vector/1');
add_line(target, 'LQI Anti Windup Difference/1', 'LQI Feedback Vector/2');
add_line(target, 'LQI Feedback Vector/1', 'LTI System/1');
add_line(target, 'LTI System/1', 'LQI Feedback Limit/1');
add_line(target, 'LTI System/1', 'LQI Anti Windup Difference/2');
add_line(target, 'LQI Feedback Limit/1', 'LQI Anti Windup Difference/1');
add_line(target, 'LQI Feedback Limit/1', 'u/1');
set_param(errorMux, 'Inputs', '6');
end

function installActuatorVariant(projectRoot)
modelFile = fullfile(projectRoot, 'matlab', 'simscape_subsystems', 'stewart_strut.slx');
load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_strut', 0));

source = 'stewart_strut/Actuator/Classical';
target = 'stewart_strut/Actuator/Nonideal-Force';
if blockExists(target)
    delete_block(target);
end
add_block(source, target, 'CopyOption', 'nolink');
set_param(target, 'VariantControl', 'stewart.actuators.type==7');

converter = find_system(target, 'SearchDepth', 1, ...
    'LookUnderMasks', 'all', 'MaskType', 'Simulink-PS Converter');
joint = find_system(target, 'SearchDepth', 1, ...
    'LookUnderMasks', 'all', 'MaskType', 'Prismatic Joint');
gainBlock = [target, '/Gain'];
converterLines = get_param(converter{1}, 'LineHandles');
if converterLines.Inport ~= -1
    delete_line(converterLines.Inport);
end

add_block('simulink/Continuous/Transport Delay', [target, '/Input Delay'], ...
    'DelayTime', 'stewart.nonidealForceActuator.inputDelay', ...
    'InitialOutput', 'stewart.nonidealForceActuator.initialForce(i)', ...
    'Position', [145 405 240 455]);
add_block('simulink/Discontinuities/Dead Zone', [target, '/Force Dead Zone'], ...
    'LowerValue', '-stewart.nonidealForceActuator.deadzone(i)', ...
    'UpperValue', 'stewart.nonidealForceActuator.deadzone(i)', ...
    'Position', [265 405 365 455]);
add_block('simulink/Discontinuities/Rate Limiter', [target, '/Force Rate Limit'], ...
    'RisingSlewLimit', 'stewart.nonidealForceActuator.forceRateLimit(i)', ...
    'FallingSlewLimit', '-stewart.nonidealForceActuator.forceRateLimit(i)', ...
    'InitialCondition', 'stewart.nonidealForceActuator.initialForce(i)', ...
    'Position', [390 405 500 455]);
add_block('simulink/Continuous/State-Space', [target, '/Force Lag'], ...
    'A', '-1/stewart.nonidealForceActuator.timeConstant', ...
    'B', '1/stewart.nonidealForceActuator.timeConstant', ...
    'C', '1', ...
    'D', '0', ...
    'X0', 'stewart.nonidealForceActuator.initialForce(i)', ...
    'Position', [525 405 625 455]);
add_block('simulink/Discontinuities/Saturation', [target, '/Force Saturation'], ...
    'UpperLimit', 'stewart.nonidealForceActuator.forceLimit(i)', ...
    'LowerLimit', '-stewart.nonidealForceActuator.forceLimit(i)', ...
    'Position', [650 405 760 455]);

if blockExists(gainBlock)
    set_param(gainBlock, 'Gain', '1');
    deleteConnectedLines(gainBlock);
    add_line(target, 'Fi/1', 'Gain/1');
    add_line(target, 'Gain/1', 'Input Delay/1');
else
    add_line(target, 'Fi/1', 'Input Delay/1');
end
add_line(target, 'Input Delay/1', 'Force Dead Zone/1');
add_line(target, 'Force Dead Zone/1', 'Force Rate Limit/1');
add_line(target, 'Force Rate Limit/1', 'Force Lag/1');
add_line(target, 'Force Lag/1', 'Force Saturation/1');
add_line(target, 'Force Saturation/1', [get_param(converter{1}, 'Name'), '/1']);

if ~isempty(joint)
    set_param(joint{1}, ...
        'TorqueActuationMode', 'InputTorque', ...
        'MotionActuationMode', 'ComputedMotion', ...
        'SenseTorqueForce', 'on');
    forceConverter = [target, '/PS-Simulink Converter1'];
    set_param(forceConverter, 'Unit', 'N');
    forcePhysicalLines = get_param(forceConverter, 'LineHandles');
    if forcePhysicalLines.LConn ~= -1
        delete_line(forcePhysicalLines.LConn);
    end
    jointPorts = get_param(joint{1}, 'PortHandles');
    forceConverterPorts = get_param(forceConverter, 'PortHandles');
    add_line(target, jointPorts.RConn(3), forceConverterPorts.LConn(1));
    forceOutput = get_param(forceConverter, 'LineHandles');
    forceInput = get_param([target, '/Fmi'], 'LineHandles');
    if forceInput.Inport == -1
        if forceOutput.Outport ~= -1
            delete_line(forceOutput.Outport);
        end
        add_line(target, 'PS-Simulink Converter1/1', 'Fmi/1');
    end
end

save_system('stewart_strut', modelFile);
clear cleanup;
end

function tf = blockExists(blockPath)
try
    tf = getSimulinkBlockHandle(blockPath) ~= -1;
catch
    tf = false;
end
end

function deleteConnectedLines(blockPath)
lineHandles = get_param(blockPath, 'LineHandles');
fields = fieldnames(lineHandles);
for fieldIndex = 1:numel(fields)
    handles = lineHandles.(fields{fieldIndex});
    handles = handles(handles ~= -1);
    for handleIndex = 1:numel(handles)
        try
            delete_line(handles(handleIndex));
        catch
        end
    end
end
end
