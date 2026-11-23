function report=ch5SpeedReducedPrepareCheck()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','prepare_check_v1');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'));
for mode=["ideal-force","nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(1,mode);r.method="hold";r.task.probe_force=[1;zeros(5,1)];r.reduced_identity=ch5ReducedIdentity();
    ch5ReleaseEpisodeData(r.mode,r.method,r.task);assignin('base','ch5Runtime',r);
    set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5ReducedFeedbackSfun','Parameters','ch5Runtime');
    timer=tic;set_param(r.modelName,'SimulationCommand','update');updateSeconds=toc(timer);timer=tic;
    output=sim(r.modelName,'SimulationMode','normal','StopTime','.1','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
    records=getappdata(0,'ch5Records');predicted=zeros(12,numel(records)-1);measured=predicted;
    for j=1:numel(records)-1
        yn=full(r.plant.fd(records{j}.x,records{j}.u,zeros(6,1)));predicted(:,j)=yn(1:12);measured(:,j)=records{j+1}.x(1:12);
    end
    difference=predicted-measured;
    row=struct('mode',char(mode),'episode',numel(report.rows)+1,'preparation',r.preparation,'update_seconds',updateSeconds,'wall_seconds',wallSeconds, ...
        'position_step_error_max',max(abs(difference(1:3,:)),[],'all'),'velocity_step_error_max',max(abs(difference(7:12,:)),[],'all'), ...
        'simulation_error',output.ErrorMessage,'control_samples',numel(records),'code_identity',r.reduced_identity);
    row.accepted=isempty(output.ErrorMessage)&&numel(records)==6&&row.position_step_error_max<1e-6&&row.velocity_step_error_max<1e-4;
    report.rows{end+1}=row;save(fullfile(directory,sprintf('%d_%s.mat',row.episode,strrep(mode,'-','_'))),'records','output','r','row','predicted','measured','-v7');
    fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    fprintf('Reuse mode %s passed=%d reset %.3fs update %.3fs impulse %.3fs step position %.3g velocity %.3g\n',mode,row.accepted,r.preparation.task_reset_seconds,updateSeconds,wallSeconds,row.position_step_error_max,row.velocity_step_error_max);
end
assert(all(cellfun(@(s)s.accepted,report.rows)),'ch5:ReuseMismatch','In-memory actuator switching changed the physical response.');
end
