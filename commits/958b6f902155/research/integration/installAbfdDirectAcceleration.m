function signalPath=installAbfdDirectAcceleration(modelName)
% Add an independent world-frame acceleration sensor for offline labels.
% Keep existing motion sensing ports and controller inputs unchanged.
subsystem=[modelName,'/Relative Motion Sensor'];
source=[subsystem,'/Transform',newline,'Sensor'];
sensor=[subsystem,'/Abfd Acceleration Sensor'];
assert(getSimulinkBlockHandle(sensor)==-1,'ABFD:AlreadyInstalled','Acceleration sensor already installed.');
add_block(source,sensor,'Position',[730,420,910,570]);
parameters=fieldnames(get_param(sensor,'DialogParameters'));
for k=1:numel(parameters)
    if strncmp(parameters{k},'Sense',5), set_param(sensor,parameters{k},'off'); end
end
set_param(sensor,'MeasurementFrame','Base','SenseAlpha','on','SenseXYZDDot','on');
original=get_param(source,'PortHandles'); ports=get_param(sensor,'PortHandles');
assert(isscalar(ports.LConn) && numel(ports.RConn)==3, ...
    'ABFD:AccelerationPorts','Expected B/F frames and two vector acceleration ports.');
add_line(subsystem,original.LConn(1),ports.LConn(1),'autorouting','on');
add_line(subsystem,original.RConn(1),ports.RConn(1),'autorouting','on');
names={'Abfd Angular Acceleration','Abfd Linear Acceleration'};
units={'rad/s^2','m/s^2'};
converterSource=[subsystem,'/PS-Simulink',newline,'Converter2'];
for k=1:2
    converter=[subsystem,'/',names{k}];
    add_block(converterSource,converter,'Position',[1010,400+80*k,1150,440+80*k]);
    set_param(converter,'Unit',units{k});
    handles=get_param(converter,'PortHandles');
    add_line(subsystem,ports.RConn(k+1),handles.LConn(1),'autorouting','on');
end
add_block('simulink/Signal Routing/Mux',[subsystem,'/Abfd Spatial Acceleration'], ...
    'Inputs','2','Position',[1280,480,1305,640]);
add_block('simulink/Sinks/Out1',[subsystem,'/AAbfd'], ...
    'Port','3','Position',[1370,545,1400,565]);
add_line(subsystem,[names{2},'/1'],'Abfd Spatial Acceleration/1','autorouting','on');
add_line(subsystem,[names{1},'/1'],'Abfd Spatial Acceleration/2','autorouting','on');
add_line(subsystem,'Abfd Spatial Acceleration/1','AAbfd/1','autorouting','on');
bus=[modelName,'/Bus',newline,'Creator1']; count=str2double(get_param(bus,'Inputs'));
set_param(bus,'Inputs',num2str(count+1)); busPorts=get_param(bus,'PortHandles');
sensorPorts=get_param(subsystem,'PortHandles');
line=add_line(modelName,sensorPorts.Outport(3),busPorts.Inport(count+1),'autorouting','on');
set_param(line,'Name','abfdSpatialAcceleration');
set_param(modelName,'SimulationCommand','update');
signalPath=[subsystem,':3'];
end
