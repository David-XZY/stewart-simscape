function report=ch5SpeedForceFinalizeV72()
% Release saved episode records only after verifying their durable artifacts.
% Keep the original MATLAB session and cached planning graph alive.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
for name={'teacher_full_v72','terminal_full_v70','full_v71','fault_v71'}
    file=fullfile(directory,name{1},'summary.json');assert(isfile(file),'ch5:MissingArtifact','Required raw run ledger missing.');
    data=jsondecode(fileread(file));assert(data.completed,'ch5:IncompleteArtifact','Required development ledger incomplete.');
end
runtime=evalin('base','ch5Runtime');model=runtime.modelName;block=[model,'/Strict CLF-CBF-QP Filter'];
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'ch5:WorkerBusy','Do not clear data during simulation.');
assert(strcmp(get_param(block,'FunctionName'),'ch5ReducedV33Sfun'),'ch5:RestoreMismatch','Original worker controller must be restored.');
assert(runtime.c.H==60&&abs(runtime.c.Ts-.02)<1e-12,'ch5:RestoreMismatch','Original horizon and step must be restored.');
keys={'ch5Records','ch5Predictions','ch5Plans','ch5Audit','ch5ReducedExecution','ch5ReducedSensitivity','ch5State','ch5Random','ch5DiffusionWarm','ch5ControlRuntime'};
cleared={};for i=1:numel(keys),if isappdata(0,keys{i}),rmappdata(0,keys{i});cleared{end+1}=keys{i};end;end
memoryInfo=memory;
report=struct('completed',true,'pid',feature('getpid'),'model_status',get_param(model,'SimulationStatus'), ...
    'controller_function',get_param(block,'FunctionName'),'H',runtime.c.H,'Ts',runtime.c.Ts, ...
    'memory_used_matlab_bytes',memoryInfo.MemUsedMATLAB,'cleared_keys',{cleared},'matlab_restarted',false, ...
    'formal_ready',false,'formal_runs',0,'formal_required',1440);
fid=fopen(fullfile(directory,'finalized_v72.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
