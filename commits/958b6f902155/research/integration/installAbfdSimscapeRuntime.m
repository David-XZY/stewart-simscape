function interface=installAbfdSimscapeRuntime(modelName)
% In-memory installation only. Closing without save restores original SLX.
installAbfdDirectVelocity(modelName);
block=[modelName,'/ABFD Controller'];
assert(getSimulinkBlockHandle(block)==-1,'ABFD:AlreadyInstalled','ABFD runtime already installed.');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',block, ...
    'FunctionName','geometricTvlqrSimscapeSfun','Parameters','abfdRuntime','Position',[750,700,950,890]);
selector=[modelName,'/ABFD Actual Force'];
add_block('simulink/Signal Routing/Bus Selector',selector,'OutputSignals','Taum','Position',[400,740,450,790]);
y=get_param([modelName,'/From'],'PortHandles'); sel=get_param(selector,'PortHandles');
add_line(modelName,y.Outport(1),sel.Inport(1),'autorouting','on');
set_param(modelName,'SimulationCommand','update');
p=get_param(block,'PortHandles'); sensor=get_param([modelName,'/Relative Motion Sensor'],'PortHandles');
sel=get_param(selector,'PortHandles');
add_line(modelName,sensor.Outport(1),p.Inport(1),'autorouting','on');
add_line(modelName,sensor.Outport(2),p.Inport(2),'autorouting','on');
add_line(modelName,sel.Outport(1),p.Inport(3),'autorouting','on');
% Continuous integration is strictly causal: its output contains the force
% over the completed interval, with no dependence on the new force command.
impulseBlock=[modelName,'/ABFD Actual Force Integral'];
add_block('simulink/Continuous/Integrator',impulseBlock,'InitialCondition','zeros(6,1)', ...
    'Position',[500,820,560,860]);
ip=get_param(impulseBlock,'PortHandles');
add_line(modelName,sel.Outport(1),ip.Inport(1),'autorouting','on');
add_line(modelName,ip.Outport(1),p.Inport(4),'autorouting','on');
sumPorts=get_param([modelName,'/Sum Feedback Feedforward'],'PortHandles');
line=get_param(sumPorts.Outport(1),'Line'); dest=get_param(line,'DstPortHandle'); delete_line(line);
for k=1:numel(dest), if dest(k)~=-1, add_line(modelName,p.Outport(1),dest(k),'autorouting','on'); end, end
bus=[modelName,'/Bus',newline,'Creator1']; set_param(bus,'Inputs','14'); bp=get_param(bus,'PortHandles');
names={'abfdQ','abfdQd','abfdDiagnostics','abfdFeedforward','abfdFeedback','abfdEstimatedState'};
for k=1:6
    h=add_line(modelName,p.Outport(k+1),bp.Inport(k+7),'autorouting','on'); set_param(h,'Name',names{k});
end
h=add_line(modelName,ip.Outport(1),bp.Inport(14),'autorouting','on'); set_param(h,'Name','abfdForceIntegral');
set_param(get_param(p.Outport(1),'Line'),'Name','u');
set_param(modelName,'SimulationCommand','update');
interface=struct('modelSaved',false,'controllerBlock',block,'forceSource','measured-actuator-output', ...
    'forcePath','y.Taum <- stewart_strut/Fmi <- Nonideal-Force/PS-Simulink Converter1 <- prismatic actuation force', ...
    'forceUnit','N','forceSign',1,'velocityFrame','world','diagnosticWidth',151, ...
    'observerForceSource','time integral of measured-actuator-output','observerForceUnit','N s', ...
    'observerTiming','posterior at previous tick -> measured completed-interval impulse -> prediction at current tick -> pose correction');
end
