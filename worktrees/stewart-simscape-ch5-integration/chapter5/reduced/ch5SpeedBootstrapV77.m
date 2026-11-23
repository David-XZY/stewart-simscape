function report=ch5SpeedBootstrapV77()
% One initialization after the previous process exited; all jobs reuse this session.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','online_v77');if ~isfolder(out),mkdir(out);end
assert(~isfile(fullfile(out,'bootstrap.json')),'ch5:ExistingAttempt','Preserve worker boot attempts.');timer=tic;
r=ch5ReducedPrepareV48(4,"nonideal-force");assignin('base','ch5Runtime',r);
block=[r.modelName,'/Strict CLF-CBF-QP Filter'];set_param(block,'FunctionName','ch5ReducedV33Sfun','Parameters','ch5Runtime');
configuration=mex.getCompilerConfigurations('C++','Selected');compiler={};for j=1:numel(configuration),compiler{end+1}=configuration(j).Name;end %#ok<AGROW>
report=struct('completed',true,'pid',feature('getpid'),'seconds',toc(timer),'H',r.c.H,'Ts',r.c.Ts,'compiler',{compiler}, ...
    'reason','previous persistent MATLAB process was no longer running','formal',false);
fid=fopen(fullfile(out,'bootstrap.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
