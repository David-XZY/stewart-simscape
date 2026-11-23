function interface = installSimscapeStrictClfCbfQpRuntime(modelName)
% installSimscapeStrictClfCbfQpRuntime - Insert strict QP without saving SLX.
%
% The loaded model is modified only in memory.  The complete computed-
% torque + LQI output is intercepted after "Sum Feedback Feedforward", sent
% through the strict-QP Level-2 S-function, and then routed to the original
% actuator and logging destinations.

arguments
    modelName {mustBeTextScalar} = "stewart_platform_model"
end
modelName = char(modelName);
if ~bdIsLoaded(modelName)
    error('installSimscapeStrictClfCbfQpRuntime:ModelNotLoaded', ...
        'Load and prepare %s before installing the runtime QP.', modelName);
end
if evalin('base', 'exist(''strictQpSimscape'', ''var'')') ~= 1
    error('installSimscapeStrictClfCbfQpRuntime:MissingRuntimeData', ...
        'Base-workspace variable strictQpSimscape is required.');
end

installControllerChoice(modelName);
velocitySignal = installDirectVelocityMeasurement(modelName);
referenceBlocks = installStrictReferenceSources(modelName);
qpBlock = installQpBlock(modelName);

% Let the Level-2 S-function declare its six inputs and five outputs before
% programmatic wiring.  The computed-torque branch remains active here.
set_param(modelName, 'SimulationCommand', 'update');
assignin('base', 'controller', initializeController('type', 'strict-clf-cbf-qp'));
set_param(modelName, 'SimulationCommand', 'update');

wireQpInputsAndCommand(modelName, qpBlock, velocitySignal, referenceBlocks);
loggedSignals = wireDiagnosticLogging(modelName, qpBlock);
set_param(modelName, 'SimulationCommand', 'update');

interface = struct();
interface.modelName = modelName;
interface.nominalForceSource = [modelName, '/Sum Feedback Feedforward'];
interface.qpBlock = qpBlock;
interface.commandDestination = [modelName, '/Stewart Platform'];
interface.poseSignal = [modelName, '/Relative Motion Sensor:1'];
interface.directVelocitySignal = velocitySignal;
interface.referenceBlocks = referenceBlocks;
interface.loggedSignals = loggedSignals;
interface.forceSign = 1;
interface.forceSignStatement = ...
    'F_cmd uses the existing Simscape axial-force sign; no sign inversion is inserted.';
interface.modelSaved = false;
interface.normalModeOnly = true;
interface.controllerType = 11;
end

function installControllerChoice(modelName)
controllerPath = [modelName, '/Controller'];
source = [controllerPath, '/Computed-Torque-Force'];
target = [controllerPath, '/Strict-CLF-CBF-QP-Nominal'];
if ~blockExists(source)
    error('installSimscapeStrictClfCbfQpRuntime:MissingNominalController', ...
        'Computed-Torque-Force variant is not installed.');
end
if blockExists(target)
    delete_block(target);
end
add_block(source, target, 'CopyOption', 'nolink');
set_param(target, 'VariantControl', 'controller.type == 11');
end

function velocitySignal = installDirectVelocityMeasurement(modelName)
sensor = [modelName, '/Relative Motion Sensor'];
transformSensor = [sensor, '/Transform', newline, 'Sensor'];
set_param(transformSensor, ...
    'SenseOmegaX', 'on', 'SenseOmegaY', 'on', 'SenseOmegaZ', 'on', ...
    'SenseXDot', 'on', 'SenseYDot', 'on', 'SenseZDot', 'on');
set_param(modelName, 'SimulationCommand', 'update');

names = {'Strict Omega X', 'Strict Omega Y', 'Strict Omega Z', ...
    'Strict Velocity X', 'Strict Velocity Y', 'Strict Velocity Z'};
units = {'rad/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s'};
sensorPortIndices = [3, 4, 5, 9, 10, 11];
sourceConverter = [sensor, '/PS-Simulink', newline, 'Converter2'];
for index = 1:numel(names)
    path = [sensor, '/', names{index}];
    if blockExists(path)
        delete_block(path);
    end
    row = mod(index-1, 3);
    column = floor((index-1)/3);
    position = [1010+160*column, 70+80*row, 1125+160*column, 110+80*row];
    add_block(sourceConverter, path, 'Position', position);
    set_param(path, 'Unit', units{index});
end

muxPath = [sensor, '/Strict Spatial Velocity'];
outportPath = [sensor, '/VStrict'];
deleteIfPresent(muxPath);
deleteIfPresent(outportPath);
add_block('simulink/Signal Routing/Mux', muxPath, ...
    'Inputs', '6', 'Position', [1390, 90, 1415, 350]);
add_block('simulink/Sinks/Out1', outportPath, ...
    'Port', '2', 'Position', [1480, 205, 1510, 225]);

transformPorts = get_param(transformSensor, 'PortHandles');
if numel(transformPorts.RConn) < max(sensorPortIndices)
    error('installSimscapeStrictClfCbfQpRuntime:VelocityPortsUnavailable', ...
        'Transform Sensor did not expose the requested direct velocity ports.');
end
for index = 1:numel(names)
    converter = [sensor, '/', names{index}];
    converterPorts = get_param(converter, 'PortHandles');
    add_line(sensor, transformPorts.RConn(sensorPortIndices(index)), ...
        converterPorts.LConn(1));
end

% The QP expects [world linear velocity; world angular velocity].
linearOrder = 4:6;
angularOrder = 1:3;
orderedConverters = [linearOrder, angularOrder];
for muxIndex = 1:6
    add_line(sensor, [names{orderedConverters(muxIndex)}, '/1'], ...
        ['Strict Spatial Velocity/', num2str(muxIndex)], 'autorouting', 'on');
end
add_line(sensor, 'Strict Spatial Velocity/1', 'VStrict/1', 'autorouting', 'on');
velocitySignal = [modelName, '/Relative Motion Sensor:2'];
end

function blocks = installStrictReferenceSources(modelName)
specification = { ...
    'Strict Q Reference', 'references.qAbs', [470, 805, 610, 835]; ...
    'Strict Qd Reference', 'references.rd', [470, 850, 610, 880]; ...
    'Strict Qdd Reference', 'references.rdd', [470, 895, 610, 925]};
blocks = strings(3, 1);
for index = 1:size(specification, 1)
    path = [modelName, '/', specification{index, 1}];
    deleteIfPresent(path);
    add_block('simulink/Sources/From Workspace', path, ...
        'VariableName', specification{index, 2}, ...
        'Position', specification{index, 3});
    blocks(index) = string(path);
end
end

function qpBlock = installQpBlock(modelName)
qpBlock = [modelName, '/Strict CLF-CBF-QP Filter'];
deleteIfPresent(qpBlock);
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', qpBlock, ...
    'FunctionName', 'strictClfCbfQpSimscapeSfun', ...
    'Parameters', 'strictQpSimscape', ...
    'Position', [760, 690, 970, 940]);
end

function wireQpInputsAndCommand(modelName, qpBlock, velocitySignal, referenceBlocks)
qpPorts = get_param(qpBlock, 'PortHandles');
if numel(qpPorts.Inport) ~= 6 || numel(qpPorts.Outport) ~= 5
    error('installSimscapeStrictClfCbfQpRuntime:UnexpectedQpPorts', ...
        'Strict QP block must expose 6 inputs and 5 outputs.');
end

posePorts = get_param([modelName, '/Relative Motion Sensor'], 'PortHandles');
add_line(modelName, posePorts.Outport(1), qpPorts.Inport(1), 'autorouting', 'on');
add_line(modelName, posePorts.Outport(2), qpPorts.Inport(2), 'autorouting', 'on');
for index = 1:3
    sourcePorts = get_param(char(referenceBlocks(index)), 'PortHandles');
    add_line(modelName, sourcePorts.Outport(1), qpPorts.Inport(index+2), ...
        'autorouting', 'on');
end

sumPath = [modelName, '/Sum Feedback Feedforward'];
sumPorts = get_param(sumPath, 'PortHandles');
originalLine = get_param(sumPorts.Outport(1), 'Line');
if originalLine == -1
    error('installSimscapeStrictClfCbfQpRuntime:MissingNominalForceLine', ...
        'Nominal-force Sum has no output line to intercept.');
end
destinationPorts = get_param(originalLine, 'DstPortHandle');
destinationPorts = destinationPorts(destinationPorts ~= -1);
delete_line(originalLine);
add_line(modelName, sumPorts.Outport(1), qpPorts.Inport(6), 'autorouting', 'on');
for index = 1:numel(destinationPorts)
    add_line(modelName, qpPorts.Outport(1), destinationPorts(index), ...
        'autorouting', 'on');
end
commandLine = get_param(qpPorts.Outport(1), 'Line');
set_param(commandLine, 'Name', 'u');

% velocitySignal is retained in the interface for human/model inspection.
if strlength(string(velocitySignal)) == 0
    error('installSimscapeStrictClfCbfQpRuntime:MissingVelocitySignal', ...
        'Direct velocity signal was not installed.');
end
end

function names = wireDiagnosticLogging(modelName, qpBlock)
bus = [modelName, '/Bus', newline, 'Creator1'];
set_param(bus, 'Inputs', '11');
busPorts = get_param(bus, 'PortHandles');
qpPorts = get_param(qpBlock, 'PortHandles');
names = ["qQp"; "qdQp"; "FnomQp"; "strictQpDiagnostic"];
for index = 1:4
    line = add_line(modelName, qpPorts.Outport(index+1), ...
        busPorts.Inport(index+7), 'autorouting', 'on');
    set_param(line, 'Name', char(names(index)));
end
end

function deleteIfPresent(path)
if blockExists(path)
    delete_block(path);
end
end

function tf = blockExists(path)
try
    tf = getSimulinkBlockHandle(path) ~= -1;
catch
    tf = false;
end
end
