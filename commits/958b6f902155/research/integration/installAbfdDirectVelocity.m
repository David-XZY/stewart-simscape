function velocitySignal = installAbfdDirectVelocity(modelName)
sensor = [modelName, '/Relative Motion Sensor'];
transformSensor = [sensor, '/Transform', newline, 'Sensor'];
set_param(transformSensor, ...
    'SenseOmegaX', 'on', 'SenseOmegaY', 'on', 'SenseOmegaZ', 'on', ...
    'SenseXDot', 'on', 'SenseYDot', 'on', 'SenseZDot', 'on');
set_param(modelName, 'SimulationCommand', 'update');

names = {'Abfd Omega X', 'Abfd Omega Y', 'Abfd Omega Z', ...
    'Abfd Velocity X', 'Abfd Velocity Y', 'Abfd Velocity Z'};
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

muxPath = [sensor, '/Abfd Spatial Velocity'];
outportPath = [sensor, '/VAbfd'];
deleteIfPresent(muxPath);
deleteIfPresent(outportPath);
add_block('simulink/Signal Routing/Mux', muxPath, ...
    'Inputs', '6', 'Position', [1390, 90, 1415, 350]);
add_block('simulink/Sinks/Out1', outportPath, ...
    'Port', '2', 'Position', [1480, 205, 1510, 225]);

transformPorts = get_param(transformSensor, 'PortHandles');
if numel(transformPorts.RConn) < max(sensorPortIndices)
    error('installSimscapeAbfdClfCbfQpRuntime:VelocityPortsUnavailable', ...
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
        ['Abfd Spatial Velocity/', num2str(muxIndex)], 'autorouting', 'on');
end
add_line(sensor, 'Abfd Spatial Velocity/1', 'VAbfd/1', 'autorouting', 'on');
velocitySignal = [modelName, '/Relative Motion Sensor:2'];
end

function deleteIfPresent(path)
if blockExists(path), delete_block(path); end
end
function tf=blockExists(path)
tf=getSimulinkBlockHandle(path)~=-1;
end
