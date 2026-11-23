function report=ch5SpeedTransientReplay()
clear ch5ReducedTransient ch5ReducedRecoveryFast
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','transient_replay_v2');
if ~isfolder(directory),mkdir(directory);end
saved=load(fullfile(root,'chapter5','artifacts','reduced_probe','closed_latest_v2','003_nonideal_force.mat'),'records');
r=ch5ReducedPrepare(3,"nonideal-force");k=ch5ReducedKernels(r);record=saved.records{8};
timer=tic;extra=ch5ReducedTransient(record.x,record.u,record.observed,r);coldSeconds=toc(timer);
timer=tic;extra=ch5ReducedTransient(record.x,record.u,record.observed,r);hotSeconds=toc(timer);
[u,recovery]=ch5ReducedRecoveryFast(record.x,record.previous_u,record.observed,r,k,record.time,record.q);
report=struct('time',record.time,'transient',extra,'cold_seconds',coldSeconds,'hot_seconds',hotSeconds, ...
    'recovery',recovery,'identity',ch5ReducedIdentity(),'pid',feature('getpid'));
save(fullfile(directory,'replay.mat'),'record','extra','recovery','u','report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Startup transient replay t=%.2fs: peak %.6f violation %.6f rejected=%d hot %.6fs recovery valid=%d\n', ...
    record.time,extra.maximum_acceleration,extra.maximum_violation,~extra.accepted,hotSeconds,recovery.interval_valid);
assert(~extra.accepted&&extra.maximum_acceleration>1.2,'ch5:TransientMiss','Refined points must reject the previously missed physical peak.');
end
