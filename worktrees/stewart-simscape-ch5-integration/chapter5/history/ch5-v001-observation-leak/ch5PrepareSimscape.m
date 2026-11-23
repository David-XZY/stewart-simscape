function runtime=ch5PrepareSimscape(task,mode,method,seed,c)
persistent cachedSetup cachedMode cachedPlant
[physical,scene,ad,strict]=ch5Physics(c);collision=ch5CollisionModel(physical,scene,c);
mode=string(mode);method=string(method);
if isempty(cachedSetup)||cachedMode~=mode||~bdIsLoaded('stewart_platform_model')
    if bdIsLoaded('stewart_platform_model'),close_system('stewart_platform_model',0);end
    cachedSetup=prepareSimscapeStrictClfCbfQp('',struct('actuatorMode',mode,'sampleTime',c.Ts, ...
        'baselineOverrides',struct('forceRateLimit',8000/c.Ts,'lqiControlScale',c.lqi_control_scale)));
    cachedMode=mode;p=ch5ActuatorProfile(mode,c,physical);
    cachedPlant=ch5PlantModel(physical,scene,p,struct('Ts',c.Ts,'substeps',ceil(c.Ts/c.internal_step),'Np',c.H,'integralGridTs',0));
end
setup=cachedSetup;plant=cachedPlant;
[d,G]=ad.dynamics(task.q0,task.qd0);initial=full(G)\(-full(d));
stewart=setup.simscapeData.stewart;
stewart=initializeStewartPose(stewart,'AP',task.q0(1:3)-physical.qHome(1:3),'ARB',ch5Rotation(task.q0(4:6)));
if mode=="nonideal-force"
    actuator=makeNonidealForceActuator(physical,struct('forceRateLimit',8000/c.Ts,'initialForce',initial));
    stewart.nonidealForceActuator=actuator;assignin('base','nonidealForceActuator',actuator);
end
assignin('base','stewart',stewart);assignin('base','controller',initializeController('type','open-loop'));
lqi=ch5DiscreteLqi(physical,ad,c);
runtime=struct('c',c,'physical',physical,'scene',scene,'ad',ad,'strict',strict,'collision',collision, ...
    'plant',plant,'task',task,'mode',mode,'method',method,'seed',seed,'initialForce',initial, ...
    'framePose',physical.qHome,'lqi',lqi,'modelName',setup.modelName);
if method=="diffusion"
    [~,root]=ch5Init();file=fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed));
    assert(isfile(file),'ch5:MissingModel','A trained, validated model is required.');runtime.weights=load(file);
end
assignin('base','ch5Runtime',runtime);
block=[setup.modelName,'/Strict CLF-CBF-QP Filter'];
set_param(block,'FunctionName','ch5SimscapeSfun','Parameters','ch5Runtime');
set_param(setup.modelName,'StopTime',num2str(c.episode_seconds,16),'MaxStep',num2str(c.internal_step,16), ...
    'ReturnWorkspaceOutputs','on','SimscapeLogType','none');
bus=[setup.modelName,'/Bus',newline,'Creator1'];
if str2double(get_param(bus,'Inputs'))==11
    set_param(bus,'Inputs','12');bp=get_param(bus,'PortHandles');
    sp=get_param([setup.modelName,'/Relative Motion Sensor'],'PortHandles');
    line=add_line(setup.modelName,sp.Outport(2),bp.Inport(12),'autorouting','on');
    set_param(line,'Name','ch5SpatialVelocity');
end
% Mechanical visualization otherwise launches a visible window each episode.
try,set_param(setup.modelName,'SimMechanicsOpenEditorOnUpdate','off');catch,end
set_param(setup.modelName,'SimulationCommand','update');
end
