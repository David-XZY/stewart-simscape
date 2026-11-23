function report=ch5SpeedReducedV28ClosedLoop(phaseName)
% The moving pair is admitted first; all results use one frozen source set.
if nargin<1,phaseName="moving";end
clear ch5ReducedControlV28 ch5ReducedStepV28 ch5ReducedSolveV28 ch5ReducedV28Sfun
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');mainDirectory=fullfile(base,'closed_generic_v28');
if ~isfolder(mainDirectory),mkdir(mainDirectory);end
contract=load(fullfile(base,'support_audit_v28','audit.mat'),'report');
assert(contract.report.accepted,'ch5:SupportUnverified','Independent support-projection and nonlinear gates must pass.');
identity=ch5ReducedIdentity();required={'ch5ReducedControlV28_m','ch5ReducedStepV28_m','ch5ReducedSolveV28_m','ch5ReducedSupportQPv28_m', ...
    'ch5ReducedV28Sfun_m','ch5ReducedTerminalV23_m','ch5ReducedWarmV23_m','ch5ReducedAuditV23_m','ch5ReducedJacobianV17_m', ...
    'ch5ReducedPriorV21_m','ch5ReducedCollisionAuditV22_m','ch5ReducedCheckRolloutV22_m','ch5ReducedHoldAuditV22_m','ch5ReducedRecoveryFast_m'};
for j=1:numel(required)
    assert(strcmp(contract.report.identity.(required{j}),identity.(required{j})),'ch5:StaleContract','Controller changed since its independent audit.');
end
if string(phaseName)=="moving"
    cases={4,"nonideal-force",false;4,"ideal-force",false};
elseif string(phaseName)=="remaining"
    for mode=["nonideal-force","ideal-force"]
        row=jsondecode(fileread(fullfile(mainDirectory,sprintf('004_%s.json',strrep(mode,'-','_')))));
        assert(row.metrics.safe_success&&row.metrics.simulated_end_seconds>=14.999,'ch5:MovingNotAdmitted','The complete moving pair must pass before expanding validation.');
    end
    cases={3,"nonideal-force",false;1,"nonideal-force",false;1,"nonideal-force",true;3,"ideal-force",false;1,"ideal-force",false};
else,error('ch5:InvalidPhase','Unknown development phase.');end
report=struct('rows',{{}},'pid',feature('getpid'),'formal_ready',false,'phase',char(phaseName),'completed',false);
files=dir(fullfile(mainDirectory,'00*force.json'));
for j=1:numel(files),report.rows{end+1}=jsondecode(fileread(fullfile(files(j).folder,files(j).name)));end
for ci=1:size(cases,1)
    index=cases{ci,1};mode=cases{ci,2};fault=cases{ci,3};directory=mainDirectory;
    if fault,directory=fullfile(base,'recovery_generic_v28');if ~isfolder(directory),mkdir(directory);end;end
    file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));jsonFile=strrep(file,'.mat','.json');
    if isfile(jsonFile)
        cached=jsondecode(fileread(jsonFile));
        for j=1:numel(required),assert(strcmp(cached.code_identity.(required{j}),identity.(required{j})),'ch5:StaleEpisode','Existing episode has different source.');end
        assert(isfile(file),'ch5:MissingRaw','Resume requires the raw MAT file.');
        assert(cached.metrics.safe_success&&cached.metrics.simulated_end_seconds>=14.999&&isempty(cached.simulation_error), ...
            'ch5:FailedEpisode','Retain the failed episode and use a new revision for an algorithm repair.');
        fprintf('Resume: retained %s task%d fault%d\n',mode,index,fault);continue;
    end
    assert(~isfile(file),'ch5:PartialEpisode','Preserve and audit a partial raw episode before rerunning.');
    r=ch5ReducedPrepare(index,mode);r.c.episode_seconds=15;
    r.reduced=struct('iterations',2,'backtracking',2,'linearization_collision_reserve',.0002,'velocity_weight',1,'factor_derivatives',false,'retain_numerical',false,'restoration_attempts',0,'reuse_sensitivities',true,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
    if fault,r.reduced.force_rejection_window=[.4,.6];end
    r.speed.stop_on_unsafe=true;r.reduced_identity=identity;r.algorithm_revision='reduced-v28';
    ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);timer=tic;ch5ReducedKernelsV4(r);
    ch5ReducedFeedbackRollout(r);ch5ReducedBrakeSequence(r);
    ch5ReducedTerminalV23([r.task.q0;r.task.qd0;repmat(r.initialForce,(r.plant.n-12)/6,1)],repmat(r.initialForce,1,r.c.H),r,ch5ReducedKernelsV4(r));
    x=[r.task.q0;r.task.qd0];if r.plant.n>12,x=[x;repmat(r.initialForce,3,1)];end
    [~,o]=ch5Observe(r.task,0);ch5ReducedHoldAuditV22(x,r.initialForce,r.initialForce,o,r);
    [~,~,trace]=r.plant.rollout(x,repmat(r.initialForce,1,r.c.H));ch5ReducedCollisionAuditV22(full(trace),o,r);buildSeconds=toc(timer);
    r.journalDirectory=fullfile(directory,sprintf('%03d_%s_journal',index,strrep(mode,'-','_')));
    if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
    assignin('base','ch5Runtime',r);
    set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5ReducedV28Sfun','Parameters','ch5Runtime');
    updateTimer=tic;set_param(r.modelName,'SimulationCommand','update');r.preparation.model_update_seconds=toc(updateTimer);
    fprintf('Conditioned Simscape start %s task%d fault%d reset %.3fs update %.3fs build %.3fs\n',mode,index,fault,r.preparation.task_reset_seconds,r.preparation.model_update_seconds,buildSeconds);
    timer=tic;output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
    records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
    save(file,'records','predictions','plans','output','r','-v7');
    auditTimer=tic;metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');metricsSeconds=toc(auditTimer);
    metrics.task_predictive_solves=metrics.predictive_solves;metrics.task_predictive_solver_failures=metrics.predictive_solver_failures;
    metrics.restoration_solves=0;metrics.restoration_solver_failures=0;
    for pi=1:numel(predictions)
        for ai=1:numel(predictions{pi}.prediction.restoration_attempts)
            attempt=predictions{pi}.prediction.restoration_attempts{ai};
            metrics.restoration_solves=metrics.restoration_solves+1;
            metrics.restoration_solver_failures=metrics.restoration_solver_failures+double(~attempt.solver_success);
        end
    end
    metrics.predictive_solves=metrics.predictive_solves+metrics.restoration_solves;
    metrics.predictive_solver_failures=metrics.predictive_solver_failures+metrics.restoration_solver_failures;
    refresh=cellfun(@(s)s.prediction.refresh,predictions);allTime=cellfun(@(s)s.controller_seconds,records);
    row=struct('mode',char(mode),'task',index,'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds, ...
        'fault_injection',fault,'factored_derivatives_used',false,'metrics_seconds',metricsSeconds,'preparation',r.preparation, ...
        'simulation_error',output.ErrorMessage,'refresh_seconds',allTime(refresh),'execution_seconds',allTime(~refresh), ...
        'refresh_deadline_misses',nnz(allTime(refresh)>.1),'execution_deadline_misses',nnz(allTime(~refresh)>.02), ...
        'full_horizon_refreshes',nnz(refresh),'intermediate_execution_samples',nnz(~refresh), ...
        'objective_screens',sum(cellfun(@(s)s.prediction.objective_screens,predictions)), ...
        'full_candidate_audits',sum(cellfun(@(s)s.prediction.full_candidate_audits,predictions)), ...
        'full_prediction_rejections',sum(cellfun(@(s)s.prediction.refresh&&~s.prediction.chosen,predictions)), ...
        'retained_numerical_reuses',sum(cellfun(@(s)s.prediction.used_retained_numerical,predictions)), ...
        'reduced_parameters',r.reduced,'code_identity',identity,'variant','bounded_support_projection_row_scaling_and_cached_terminal_tail');
    save(file,'row','audit','-append');writeJSON(jsonFile,row);
    if ~fault,report.rows{end+1}=row;end
    report.completed=numel(report.rows)==6;save(fullfile(directory,'summary.mat'),'report','-v7');
    writeJSON(fullfile(directory,'summary.json'),report);writeJSON(fullfile(mainDirectory,'summary.json'),report);
    fprintf('Conditioned Simscape %s task%d fault%d wall %.2fs arrived=%d safe=%d final %.6fm fallback%d QPfail%d\n', ...
        mode,index,fault,wallSeconds,metrics.arrival,metrics.all_constraints_valid,metrics.final_position,metrics.fallback_count,metrics.predictive_solver_failures);
    assert(isempty(output.ErrorMessage),'ch5:ClosedLoopRuntimeFailure','Saved Simscape runtime failure; stop this batch.');
    assert(metrics.safe_success,'ch5:ClosedLoopNotAdmitted','Saved unsafe or non-arriving episode; stop expansion of this revision.');
end
report.completed=numel(report.rows)==6;writeJSON(fullfile(mainDirectory,'summary.json'),report);
end

function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
