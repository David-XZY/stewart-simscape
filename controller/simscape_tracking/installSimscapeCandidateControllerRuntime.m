function interface = installSimscapeCandidateControllerRuntime(modelName)
% installSimscapeCandidateControllerRuntime - In-memory candidate controller.
arguments
    modelName {mustBeTextScalar} = "stewart_platform_model"
end
modelName = char(modelName);
if ~bdIsLoaded(modelName)
    error('installSimscapeCandidateControllerRuntime:ModelNotLoaded', ...
        'Load and prepare %s before installing the runtime controller.', modelName);
end
if evalin('base', 'exist(''candidateControllerRuntime'', ''var'')') ~= 1
    error('installSimscapeCandidateControllerRuntime:MissingRuntime', ...
        'candidateControllerRuntime is required in the base workspace.');
end

velocitySignal = installVelocity(modelName);
referenceBlocks = installReferences(modelName);
controllerBlock = [modelName, '/Disturbance Candidate Controller'];
deleteIfPresent(controllerBlock);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    controllerBlock, 'FunctionName', 'candidateDisturbanceControllerSfun', ...
    'Parameters', 'candidateControllerRuntime', ...
    'Position', [760, 680, 980, 950]);
set_param(modelName, 'SimulationCommand', 'update');
wireController(modelName, controllerBlock, velocitySignal, referenceBlocks);
loggedSignals = wireLogging(modelName, controllerBlock);
set_param(modelName, 'SimulationCommand', 'update');
interface = struct('modelName', string(modelName), ...
    'controllerBlock', string(controllerBlock), ...
    'velocitySignal', string(velocitySignal), ...
    'referenceBlocks', referenceBlocks, 'loggedSignals', loggedSignals, ...
    'modelSaved', false, 'normalModeOnly', true);
end

function signal = installVelocity(modelName)
sensor = [modelName, '/Relative Motion Sensor'];
transform = [sensor, '/Transform', newline, 'Sensor'];
set_param(transform, 'SenseOmegaX', 'on', 'SenseOmegaY', 'on', ...
    'SenseOmegaZ', 'on', 'SenseXDot', 'on', 'SenseYDot', 'on', 'SenseZDot', 'on');
set_param(modelName, 'SimulationCommand', 'update');
names = {'Candidate Omega X', 'Candidate Omega Y', 'Candidate Omega Z', ...
    'Candidate Velocity X', 'Candidate Velocity Y', 'Candidate Velocity Z'};
units = {'rad/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s'};
indices = [3, 4, 5, 9, 10, 11];
source = [sensor, '/PS-Simulink', newline, 'Converter2'];
for index = 1:6
    path = [sensor, '/', names{index}];
    deleteIfPresent(path);
    add_block(source, path, 'Position', [1010+160*floor((index-1)/3), ...
        70+80*mod(index-1, 3), 1125+160*floor((index-1)/3), ...
        110+80*mod(index-1, 3)]);
    set_param(path, 'Unit', units{index});
end
mux = [sensor, '/Candidate Spatial Velocity'];
outport = [sensor, '/VCandidate'];
deleteIfPresent(mux); deleteIfPresent(outport);
add_block('simulink/Signal Routing/Mux', mux, 'Inputs', '6', ...
    'Position', [1390, 90, 1415, 350]);
add_block('simulink/Sinks/Out1', outport, 'Port', '2', ...
    'Position', [1480, 205, 1510, 225]);
ports = get_param(transform, 'PortHandles');
for index = 1:6
    converter = get_param([sensor, '/', names{index}], 'PortHandles');
    add_line(sensor, ports.RConn(indices(index)), converter.LConn(1));
end
order = [4, 5, 6, 1, 2, 3];
for index = 1:6
    add_line(sensor, [names{order(index)}, '/1'], ...
        ['Candidate Spatial Velocity/', num2str(index)], 'autorouting', 'on');
end
add_line(sensor, 'Candidate Spatial Velocity/1', 'VCandidate/1', 'autorouting', 'on');
signal = [modelName, '/Relative Motion Sensor:2'];
end

function blocks = installReferences(modelName)
specification = { ...
    'Candidate Q Reference', 'references.qAbs', [470, 805, 610, 835]; ...
    'Candidate Qd Reference', 'references.rd', [470, 850, 610, 880]; ...
    'Candidate Qdd Reference', 'references.rdd', [470, 895, 610, 925]; ...
    'Candidate Feedforward', 'references.uCT', [470, 940, 610, 970]};
blocks = strings(4, 1);
for index = 1:4
    path = [modelName, '/', specification{index, 1}];
    deleteIfPresent(path);
    add_block('simulink/Sources/From Workspace', path, ...
        'VariableName', specification{index, 2}, 'Position', specification{index, 3});
    blocks(index) = string(path);
end
end

function wireController(modelName, blockPath, velocitySignal, references)
ports = get_param(blockPath, 'PortHandles');
posePorts = get_param([modelName, '/Relative Motion Sensor'], 'PortHandles');
add_line(modelName, posePorts.Outport(1), ports.Inport(1), 'autorouting', 'on');
add_line(modelName, posePorts.Outport(2), ports.Inport(2), 'autorouting', 'on');
for index = 1:4
    sourcePorts = get_param(char(references(index)), 'PortHandles');
    add_line(modelName, sourcePorts.Outport(1), ports.Inport(index+2), 'autorouting', 'on');
end
sumPorts = get_param([modelName, '/Sum Feedback Feedforward'], 'PortHandles');
line = get_param(sumPorts.Outport(1), 'Line');
destinations = get_param(line, 'DstPortHandle');
destinations = destinations(destinations ~= -1);
delete_line(line);
terminator = [modelName, '/Unused Baseline Nominal'];
deleteIfPresent(terminator);
add_block('simulink/Sinks/Terminator', terminator, ...
    'Position', [1030, 630, 1050, 650]);
terminatorPorts = get_param(terminator, 'PortHandles');
add_line(modelName, sumPorts.Outport(1), terminatorPorts.Inport(1), ...
    'autorouting', 'on');
for index = 1:numel(destinations)
    add_line(modelName, ports.Outport(1), destinations(index), 'autorouting', 'on');
end
commandLine = get_param(ports.Outport(1), 'Line');
set_param(commandLine, 'Name', 'u');
if strlength(string(velocitySignal)) == 0
    error('installSimscapeCandidateControllerRuntime:MissingVelocity', ...
        'Candidate velocity signal was not installed.');
end
end

function names = wireLogging(modelName, blockPath)
bus = [modelName, '/Bus', newline, 'Creator1'];
set_param(bus, 'Inputs', '11');
busPorts = get_param(bus, 'PortHandles');
controllerPorts = get_param(blockPath, 'PortHandles');
names = ["qCandidate"; "qdCandidate"; "FnomCandidate"; "candidateDiagnostic"];
for index = 1:4
    line = add_line(modelName, controllerPorts.Outport(index+1), ...
        busPorts.Inport(index+7), 'autorouting', 'on');
    set_param(line, 'Name', char(names(index)));
end
end

function deleteIfPresent(path)
try
    if getSimulinkBlockHandle(path) ~= -1, delete_block(path); end
catch
end
end
