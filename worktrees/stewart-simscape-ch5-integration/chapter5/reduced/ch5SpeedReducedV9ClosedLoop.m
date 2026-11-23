function report=ch5SpeedReducedV9ClosedLoop()
clear ch5ReducedControlV9 ch5ReducedStepV9 ch5ReducedAuditFastV8 ch5ReducedAuditV8 ch5ReducedV9Sfun
% Separate version; reuse MATLAB/model, reset each episode's physical state.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','closed_tapered_v9');
if ~isfolder(directory),mkdir(directory);end
admitted=load(fullfile(root,'chapter5','artifacts','reduced_probe','factor_stage_audit_v1','audit.mat'),'report');
assert(admitted.report.accepted,'ch5:FactorUnverified','Run exact stage and prior-cost verification first.');
contract=load(fullfile(root,'chapter5','artifacts','reduced_probe','prior_contract_v9','audit.mat'),'report');
assert(contract.report.accepted,'ch5:PriorContract','Run consecutive control calls before full simulation.');
currentIdentity=ch5ReducedIdentity();
for name={'ch5ReducedControlV9_m','ch5ReducedStepV9_m','ch5ReducedAuditFastV8_m','ch5ReducedAuditV8_m'}
    assert(strcmp(contract.report.identity.(name{1}),currentIdentity.(name{1})),'ch5:StaleContract','Contract was checked against different controller source.');
end
mainDirectory=directory;report=struct('rows',{{}},'pid',feature('getpid'),'formal_ready',false);
for mode=["nonideal-force","ideal-force"]
    indices=[4,3,1];if mode=="nonideal-force",indices=[indices,2];end
    for caseIndex=indices
        fault=caseIndex==2;index=caseIndex;directory=mainDirectory;
        if fault,index=1;directory=fullfile(root,'chapter5','artifacts','reduced_probe','recovery_tapered_v9');if ~isfolder(directory),mkdir(directory);end;end
        r=ch5ReducedPrepare(index,mode);r.c.episode_seconds=15;r.reduced=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
            'factor_derivatives',median(admitted.report.factor_seconds)<.95*median(admitted.report.original_seconds));
        if fault,r.reduced.force_rejection_window=[.4,.6];end
        r.speed.stop_on_unsafe=true;r.reduced_identity=ch5ReducedIdentity();
        ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);timer=tic;k=ch5ReducedKernelsV4(r);
        ch5ReducedFeedbackRollout(r);ch5ReducedBrakeSequence(r);
        x=[r.task.q0;r.task.qd0];if r.plant.n>12,x=[x;repmat(r.initialForce,3,1)];end
        [~,o]=ch5Observe(r.task,0);ch5ReducedTransient(x,r.initialForce,o,r);buildSeconds=toc(timer);
        r.journalDirectory=fullfile(directory,sprintf('%03d_%s_journal',index,strrep(mode,'-','_')));
        if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
        assignin('base','ch5Runtime',r);
        set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5ReducedV9Sfun','Parameters','ch5Runtime');
        updateTimer=tic;set_param(r.modelName,'SimulationCommand','update');r.preparation.model_update_seconds=toc(updateTimer);
        fprintf('Fast+transient Simscape start %s task%d reset %.3fs update %.3fs build %.3fs\n',mode,index,r.preparation.task_reset_seconds,r.preparation.model_update_seconds,buildSeconds);
        timer=tic;output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));save(file,'records','predictions','plans','output','r','-v7');
        auditTimer=tic;metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');metricsSeconds=toc(auditTimer);
        refresh=cellfun(@(s)s.prediction.refresh,predictions);allTime=cellfun(@(s)s.controller_seconds,records);
        row=struct('mode',char(mode),'task',index,'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds, ...
            'fault_injection',fault,'factored_derivatives_used',r.reduced.factor_derivatives,'metrics_seconds',metricsSeconds,'preparation',r.preparation,'simulation_error',output.ErrorMessage,'refresh_seconds',allTime(refresh),'execution_seconds',allTime(~refresh), ...
            'refresh_deadline_misses',nnz(allTime(refresh)>.1),'execution_deadline_misses',nnz(allTime(~refresh)>.02), ...
            'reduced_parameters',r.reduced,'code_identity',r.reduced_identity,'variant','time_tapered_collision_reserve_and_velocity_cost');
        save(file,'row','audit','-append');if ~fault,report.rows{end+1}=row;end;save(fullfile(directory,'summary.mat'),'report','-v7');
        fid=fopen(strrep(file,'.mat','.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(row));fclose(fid);
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
        fprintf('Fast Simscape %s task%d wall %.2fs arrived=%d safe=%d final %.6fm fallback %d\n',mode,index,wallSeconds,metrics.arrival,metrics.all_constraints_valid,metrics.final_position,metrics.fallback_count);
        if ~isempty(output.ErrorMessage),error('ch5:ClosedLoopRuntimeFailure','Saved startup/runtime failure; stop this development batch.');end
    end
end
report.completed=true;fid=fopen(fullfile(mainDirectory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
