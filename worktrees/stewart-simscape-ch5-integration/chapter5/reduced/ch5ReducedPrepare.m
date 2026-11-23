function r=ch5ReducedPrepare(index,mode)
% Reuse the already installed model; actuator type is a variant parameter.
% Avoid rebuilding the unused legacy tracking design / Simscape linearize
% when switching actuator mode. Each episode still resets physical state.
[c,root]=ch5Init();mode=string(mode);saved=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');task=saved.tasks{index};
persistent plantCache plantKey
key=jsonencode(c);
if isempty(plantKey)||~strcmp(plantKey,key),plantCache=containers.Map('KeyType','char','ValueType','any');plantKey=key;end
if ~bdIsLoaded('stewart_platform_model')||evalin('base','exist(''ch5Runtime'',''var'')')~=1
    r=ch5SpeedPrepare(index,mode,'reduced-v1',0);return;
end
timer=tic;r=evalin('base','ch5Runtime');
assert(startsWith(r.c.version,c.version)&&r.c.Ts==c.Ts&&r.c.H==c.H,'ch5:InvalidTemplate','Loaded runtime belongs to a different control configuration.');
plantCache(char(r.mode))=r.plant;
assert(isequal(r.physical.g,[0;0;-9.81])||all(isfinite(r.physical.g)),'ch5:InvalidTemplate','Invalid loaded physical model.');
r.task=task;r.method="diffusion";r.mode=mode;r.seed=5101;r.c=c;r.c.version=[c.version,'-speed-reduced-v1'];
for name={'journalDirectory','reduced_identity'}
    if isfield(r,name{1}),r=rmfield(r,name{1});end
end
[d,G]=r.ad.dynamics(task.q0,task.qd0);r.initialForce=full(G)\(-full(d));
stewart=evalin('base','stewart');stewart=initializeStewartPose(stewart,'AP',task.q0(1:3)-r.physical.qHome(1:3),'ARB',ch5Rotation(task.q0(4:6)));
if mode=="nonideal-force"
    stewart.actuators.type=7;
    actuator=makeNonidealForceActuator(r.physical,struct('forceRateLimit',c.actuator_slew_rate,'forceLagTimeConstant',c.actuator_time_constant, ...
        'forceInputDelay',c.actuator_delay,'forceDeadzone',ones(6,1)*c.actuator_deadzone,'initialForce',r.initialForce));
    stewart.nonidealForceActuator=actuator;assignin('base','nonidealForceActuator',actuator);
else,stewart.actuators.type=1;end
assignin('base','stewart',stewart);assignin('base','controller',initializeController('type','open-loop'));
if isKey(plantCache,char(mode)),r.plant=plantCache(char(mode));
else
    profile=ch5ActuatorProfile(mode,c,r.physical);r.plant=ch5PlantModel(r.physical,r.scene,profile, ...
        struct('Ts',c.Ts,'substeps',ceil(c.Ts/c.internal_step),'Np',c.H,'integralGridTs',0));plantCache(char(mode))=r.plant;
end
r.speed=struct('stop_on_unsafe',true);assignin('base','ch5Runtime',r);
r.preparation=struct('model_reused',true,'cold_build_seconds',0,'task_reset_seconds',toc(timer),'model_update_seconds',0, ...
    'legacy_linearize_skipped',true,'actuator_variant',stewart.actuators.type);
end
