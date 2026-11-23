function report=ch5SpeedTailClosedLoop()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','performance_probe','tail_closed_loop');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'),'scope','Development: fixed/switch goals, ideal actuator, blocking Simscape');
for index=[1,3]
    label='braking_one_sqp_30_polish';r=ch5SpeedPrepare(index,"ideal-force",label,30);r.c.episode_seconds=15;r.speed.stop_on_unsafe=true;r.speed.repair_held_terminal=true;
    ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);r.speed_identity=ch5SpeedIdentity();
    setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',100000+5101+r.task.seed+r.task.index));
    build=tic;[~,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];
    preview=ch5NativePreview(r);ch5NativeNodes(r);ch5PredictiveOptimizer(r,find(o(22,:)>.5));ch5PredictiveOptimizer(r,find(o(22,:)>.5),true);
    [~,~,trace]=preview(x,repmat(r.initialForce,1,r.c.H));ch5NativeCollisionAudit(full(trace),o,r);ch5SpeedTailBackup(r);buildSeconds=toc(build);
    assignin('base','ch5Runtime',r);
    set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5SpeedTailSfun','Parameters','ch5Runtime');
    r.journalDirectory=fullfile(directory,sprintf('%03d_journal',index));if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
    assignin('base','ch5Runtime',r);set_param(r.modelName,'SimulationCommand','update');timer=tic;
    output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
    records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
    file=fullfile(directory,sprintf('%03d_ideal_force.mat',index));
    save(file,'records','predictions','plans','output','r','-v7');
    metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
    refresh=cellfun(@(s)s.prediction.refresh,predictions);allTime=cellfun(@(s)s.controller_seconds,records);
    row=struct('mode',char(r.mode),'task',index,'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds, ...
        'simulation_error',output.ErrorMessage,'refresh_seconds',allTime(refresh),'execution_check_seconds',allTime(~refresh), ...
        'refresh_deadline_misses',nnz(allTime(refresh)>.1),'execution_deadline_misses',nnz(allTime(~refresh)>.02), ...
        'code_identity',r.speed_identity,'variant','tail_continuation');
    save(file,'row','audit','-append');report.rows{end+1}=row;save(fullfile(directory,'summary.mat'),'report','-v7');
    fid=fopen(strrep(file,'.mat','.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(row));fclose(fid);
    fprintf('Tail Simscape task%d wall %.2fs arrived=%d safe=%d final %.6fm fallback %d\n',index,wallSeconds,metrics.arrival,metrics.all_constraints_valid,metrics.final_position,metrics.fallback_count);
end
report.completed=true;save(fullfile(directory,'summary.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
