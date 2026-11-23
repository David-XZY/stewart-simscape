function report=ch5SpeedClosedLoop()
[~,root]=ch5Init();addpath(fullfile(root,'chapter5','performance'));directory=fullfile(root,'chapter5','artifacts','performance_probe','closed_loop');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'),'scope','Independent development; blocking Simscape with separately measured deadlines');
for mode=["ideal-force","nonideal-force"]
    for index=[1,3]
        label='continued_one_sqp_3_polish';r=ch5SpeedPrepare(index,mode,label,3);r.c.episode_seconds=15;
        ch5ReleaseEpisodeData(mode,'diffusion',r.task);
        setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',100000+5101+r.task.seed+r.task.index));
        if isappdata(0,'ch5SpeedWarm'),rmappdata(0,'ch5SpeedWarm');end
        if isappdata(0,'ch5SpeedExecution'),rmappdata(0,'ch5SpeedExecution');end
        if isappdata(0,'ch5SpeedIterate'),rmappdata(0,'ch5SpeedIterate');end
        build=tic;[~,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];if mode=="nonideal-force",x=[x;repmat(r.initialForce,3,1)];end
        preview=ch5NativePreview(r);ch5NativeNodes(r);ch5PredictiveOptimizer(r,find(o(22,:)>.5));ch5PredictiveOptimizer(r,find(o(22,:)>.5),true);
        [~,~,trace]=preview(x,repmat(r.initialForce,1,r.c.H));ch5NativeCollisionAudit(full(trace),o,r);buildSeconds=toc(build);
        assignin('base','ch5Runtime',r);set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5SpeedSfun','Parameters','ch5Runtime');
        set_param(r.modelName,'SimulationCommand','update');timer=tic;
        output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        assert(isempty(output.ErrorMessage),'ch5:SpeedSimulation','Independent speed simulation failed: %s',output.ErrorMessage);
        metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
        refresh=cellfun(@(s)s.prediction.refresh,predictions);allTime=cellfun(@(s)s.controller_seconds,records);
        refreshTime=allTime(refresh);holdTime=allTime(~refresh);
        goal0=r.task.goals(1:3,1);initialError=norm(r.task.q0(1:3)-goal0);
        row=struct('mode',char(mode),'task',index,'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds, ...
            'refresh_seconds',refreshTime,'execution_check_seconds',holdTime,'refresh_deadline_misses',nnz(refreshTime>.1), ...
            'execution_deadline_misses',nnz(holdTime>.02),'initial_position_error',initialError, ...
            'maximum_measured_displacement',max(vecnorm(audit.absolute_pose(:,1:3)'-r.task.q0(1:3))), ...
            'code_identity',ch5SpeedIdentity());
        file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));
        save(file,'row','records','predictions','plans','output','audit','-v7');
        fid=fopen(strrep(file,'.mat','.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(row));fclose(fid);
        report.rows{end+1}=row;save(fullfile(directory,'summary.mat'),'report','-v7');
        fprintf('Speed Simscape %s task %d: wall %.2fs arrived=%d safe=%d final %.6fm misses %d/%d\n',mode,index,wallSeconds,metrics.arrival,metrics.all_constraints_valid,metrics.final_position,row.refresh_deadline_misses,numel(refreshTime));
    end
end
report.completed=true;save(fullfile(directory,'summary.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
