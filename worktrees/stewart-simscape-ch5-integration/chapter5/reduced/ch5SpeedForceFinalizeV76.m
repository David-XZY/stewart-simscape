function report=ch5SpeedForceFinalizeV76()
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v76');
for p={'conditioning_audit.json','../full_v76/summary.json'}
    a=jsondecode(fileread(fullfile(out,p{1})));assert(a.completed,'ch5:IncompleteArtifact','Required complete record missing.');
end
r=evalin('base','ch5Runtime');model=r.modelName;block=[model,'/Strict CLF-CBF-QP Filter'];assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'ch5:WorkerBusy','Model must be stopped.');
assert(strcmp(get_param(block,'FunctionName'),'ch5ReducedV33Sfun'),'ch5:RestoreMismatch','Restore original worker controller.');
assert(r.c.H==60&&abs(r.c.Ts-.02)<1e-12,'ch5:RestoreMismatch','Physical configuration must be restored.');
keys={'ch5Records','ch5Predictions','ch5Plans','ch5Audit','ch5ReducedExecution','ch5ReducedSensitivity','ch5State','ch5Random','ch5DiffusionWarm','ch5ControlRuntime'};
for i=1:numel(keys),if isappdata(0,keys{i}),rmappdata(0,keys{i});end;end
m=memory;report=struct('completed',true,'pid',feature('getpid'),'model_status',get_param(model,'SimulationStatus'),'controller_function',get_param(block,'FunctionName'), ...
    'H',r.c.H,'Ts',r.c.Ts,'memory_used_matlab_bytes',m.MemUsedMATLAB,'matlab_restarted',false,'formal_ready',false,'formal_runs',0,'formal_required',1440);
fid=fopen(fullfile(out,'finalized.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
