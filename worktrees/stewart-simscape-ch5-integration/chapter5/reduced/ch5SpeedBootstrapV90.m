function report=ch5SpeedBootstrapV90()
% One cold initialization; all V90 jobs share this persistent MATLAB process.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_repair_v90');
if ~isfolder(out),mkdir(out);end
file=fullfile(out,'bootstrap.json');assert(~isfile(file),'ch5:ExistingAttempt','Preserve boot evidence.');
timer=tic;r=ch5ReducedPrepareV48(4,"nonideal-force");assignin('base','ch5Runtime',r);
block=[r.modelName,'/Strict CLF-CBF-QP Filter'];
set_param(block,'FunctionName','ch5ReducedV33Sfun','Parameters','ch5Runtime');
report=struct('completed',true,'formal',false,'pid',feature('getpid'),'seconds',toc(timer), ...
    'reason','previous MATLAB process 20004 is no longer running; reuse this process for every V90 job');
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
