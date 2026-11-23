function report=ch5SpeedReducedLatestClosedLoop()
clear ch5ReducedBrakeSequence
% Development only. Leave formal generation/config/models unchanged.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','closed_latest_v2');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'),'formal_ready',false);
for mode=["ideal-force","nonideal-force"]
    for index=[1,3,4]
        r=ch5ReducedPrepare(index,mode);r.c.episode_seconds=15;r.reduced=struct('iterations',1,'backtracking',5);
        r.speed.stop_on_unsafe=true;r.reduced_identity=ch5ReducedIdentity();
        ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);timer=tic;k=ch5ReducedKernels(r);ch5ReducedFeedbackRollout(r);ch5ReducedBrakeSequence(r);buildSeconds=toc(timer);
        r.journalDirectory=fullfile(directory,sprintf('%03d_%s_journal',index,strrep(mode,'-','_')));
        if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
        assignin('base','ch5Runtime',r);
        set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5ReducedLatestSfun','Parameters','ch5Runtime');
        updateTimer=tic;set_param(r.modelName,'SimulationCommand','update');r.preparation.model_update_seconds=toc(updateTimer);timer=tic;
        % Stop immediately on a physical-domain violation; preserve failed runs.
        fprintf('Latest Simscape start %s task%d reset %.3fs update %.3fs build %.3fs\n',mode,index,r.preparation.task_reset_seconds,r.preparation.model_update_seconds,buildSeconds);
        output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));save(file,'records','predictions','plans','output','r','-v7');
        metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
        refresh=cellfun(@(s)s.prediction.refresh,predictions);allTime=cellfun(@(s)s.controller_seconds,records);
        row=struct('mode',char(mode),'task',index,'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds, ...
            'preparation',r.preparation,'simulation_error',output.ErrorMessage,'refresh_seconds',allTime(refresh),'execution_seconds',allTime(~refresh), ...
            'refresh_deadline_misses',nnz(allTime(refresh)>.1),'execution_deadline_misses',nnz(allTime(~refresh)>.02), ...
            'code_identity',r.reduced_identity,'variant','latest_matlab_active_condensed_increment');
        save(file,'row','audit','-append');report.rows{end+1}=row;save(fullfile(directory,'summary.mat'),'report','-v7');
        fid=fopen(strrep(file,'.mat','.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(row));fclose(fid);
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
        fprintf('Reduced Simscape %s task%d wall %.2fs arrived=%d safe=%d final %.6fm fallback %d\n',mode,index,wallSeconds,metrics.arrival,metrics.all_constraints_valid,metrics.final_position,metrics.fallback_count);
    end
end
report.completed=true;
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
