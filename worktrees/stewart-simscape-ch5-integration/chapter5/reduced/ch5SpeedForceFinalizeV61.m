function report=ch5SpeedForceFinalizeV61()
% Verify the persistent template was restored, release only already-saved episode data.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v57');
r=evalin('base','ch5Runtime');name=get_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName');
report=struct('pid',feature('getpid'),'template_function',name,'template_mode',char(r.mode),'horizon',r.c.H,'control_step',r.c.Ts, ...
    'simulation_status',get_param(r.modelName,'SimulationStatus'),'formal_ready',false,'formal_runs',0,'no_restart',true);
assert(strcmp(name,'ch5ReducedV33Sfun')&&r.c.H==60&&abs(r.c.Ts-.02)<1e-12,'ch5:TemplateNotRestored','Original persistent template must be restored.');
assert(isfile(fullfile(directory,'pilot','closed_loop_v60','001_nonideal_force.mat'))&& ...
    isfile(fullfile(directory,'pilot','closed_loop_v60','004_nonideal_force.mat')),'ch5:MissingEvidence','Raw Simscape results must exist before release.');
for key={'ch5Records','ch5Predictions','ch5Plans','ch5Audit','ch5State','ch5ReducedExecution','ch5ReducedSensitivity'}
    if isappdata(0,key{1}),rmappdata(0,key{1});end
end
report.episode_appdata_released=true;
fid=fopen(fullfile(directory,'finalized.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
