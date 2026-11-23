function report=ch5SpeedReducedFeedbackClosedLoop()
% Development only. Leave formal generation/config/models unchanged.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','closed_feedback_v1');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'),'formal_ready',false);
for mode=["nonideal-force","ideal-force"]
    for index=[1,3]
        r=ch5SpeedPrepare(index,mode,'reduced-v1',0);r.c.episode_seconds=15;r.reduced=struct('iterations',1,'backtracking',5);
        r.speed.stop_on_unsafe=true;r.reduced_identity=ch5ReducedIdentity();
        ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);timer=tic;k=ch5ReducedKernels(r);ch5ReducedFeedbackRollout(r);buildSeconds=toc(timer);
        r.journalDirectory=fullfile(directory,sprintf('%03d_%s_journal',index,strrep(mode,'-','_')));
        if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
        assignin('base','ch5Runtime',r);
        set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5ReducedFeedbackSfun','Parameters','ch5Runtime');
        set_param(r.modelName,'SimulationCommand','update');timer=tic;
        % Stop immediately on a physical-domain violation; preserve failed runs.
        output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));save(file,'records','predictions','plans','output','r','-v7');
        metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
        refresh=cellfun(@(s)s.prediction.refresh,predictions);allTime=cellfun(@(s)s.controller_seconds,records);
        row=struct('mode',char(mode),'task',index,'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds, ...
            'preparation',r.preparation,'simulation_error',output.ErrorMessage,'refresh_seconds',allTime(refresh),'execution_seconds',allTime(~refresh), ...
            'refresh_deadline_misses',nnz(allTime(refresh)>.1),'execution_deadline_misses',nnz(allTime(~refresh)>.02), ...
            'code_identity',r.reduced_identity,'variant','feedback_condensed_force_increment');
        save(file,'row','audit','-append');report.rows{end+1}=row;save(fullfile(directory,'summary.mat'),'report','-v7');
        fid=fopen(strrep(file,'.mat','.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(row));fclose(fid);
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
        fprintf('Reduced Simscape %s task%d wall %.2fs arrived=%d safe=%d final %.6fm fallback %d\n',mode,index,wallSeconds,metrics.arrival,metrics.all_constraints_valid,metrics.final_position,metrics.fallback_count);
    end
end
report.completed=true;
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
