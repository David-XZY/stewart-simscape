function [runtime,detail]=ch5PrepareCachedSimscape(task,mode,method,seed,c)
% Same preparation operations and model, with task-invariant objects reused.
% A new task still resets mechanical pose, actuator equilibrium and S-function
% states. Disabling physical initialization would change the experiment.
persistent template stewartTemplate cacheKey
mode=string(mode);method=string(method);key=char(mode+"|"+jsonencode(c));
detail=struct('model_reused',false,'cold_build_seconds',0,'task_reset_seconds',0,'model_update_seconds',0);
if isempty(template)||~strcmp(cacheKey,key)||~bdIsLoaded(template.modelName)
    timer=tic;runtime=ch5PrepareSimscape(task,mode,method,seed,c);
    detail.cold_build_seconds=toc(timer);
    template=runtime;
    for field={'task','weights','weights_file'}
        if isfield(template,field{1}),template=rmfield(template,field{1});end
    end
    stewartTemplate=evalin('base','stewart');cacheKey=key;return;
end
timer=tic;runtime=template;runtime.task=task;runtime.method=method;runtime.seed=seed;
[d,G]=runtime.ad.dynamics(task.q0,task.qd0);initial=full(G)\(-full(d));runtime.initialForce=initial;
stewart=initializeStewartPose(stewartTemplate,'AP',task.q0(1:3)-runtime.physical.qHome(1:3),'ARB',ch5Rotation(task.q0(4:6)));
if mode=="nonideal-force"
    actuator=makeNonidealForceActuator(runtime.physical,struct('forceRateLimit',c.actuator_slew_rate, ...
        'forceLagTimeConstant',c.actuator_time_constant,'forceInputDelay',c.actuator_delay, ...
        'forceDeadzone',ones(6,1)*c.actuator_deadzone,'initialForce',initial));
    stewart.nonidealForceActuator=actuator;assignin('base','nonidealForceActuator',actuator);
end
assignin('base','stewart',stewart);assignin('base','controller',initializeController('type','open-loop'));
if method=="diffusion"
    [~,root]=ch5Init();file=fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed));
    assert(isfile(file),'ch5:MissingModel','A trained, validated model is required.');
    runtime.weights=load(file);runtime.weights_file=file;
end
assignin('base','ch5Runtime',runtime);
block=[runtime.modelName,'/Strict CLF-CBF-QP Filter'];
set_param(block,'FunctionName','ch5SimscapeSfun','Parameters','ch5Runtime');
set_param(runtime.modelName,'StopTime',num2str(c.episode_seconds,16),'MaxStep',num2str(c.internal_step,16), ...
    'ReturnWorkspaceOutputs','on','SimscapeLogType','none');
detail.model_reused=true;detail.task_reset_seconds=toc(timer);timer=tic;
set_param(runtime.modelName,'SimulationCommand','update');detail.model_update_seconds=toc(timer);
end
