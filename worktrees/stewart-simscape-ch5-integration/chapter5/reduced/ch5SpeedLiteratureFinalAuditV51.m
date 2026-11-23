function report=ch5SpeedLiteratureFinalAuditV51()
% Data/source integrity audit, including failures; not controller acceptance.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'literature_final_audit_v51');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous final audits.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'syntax_rows',{{}},'episodes',{{}}, ...
    'accepted',false,'formal_ready',false,'error','','scope','Syntax, dependency identity, complete raw time series and explicit failure status');
try
    files=dir(fullfile(root,'chapter5','reduced','*.m'));
    for j=1:numel(files)
        issues=checkcode(fullfile(files(j).folder,files(j).name),'-id');syntax=any(strcmp({issues.id},'SYNER'));
        report.syntax_rows{end+1}=struct('file',files(j).name,'syntax_error',syntax);assert(~syntax,'ch5:SyntaxFailure','Retained source has a syntax error.');
    end
    common={'ch5ReducedSolveV24_m','ch5ReducedTerminalV23_m','ch5ReducedWarmV23_m','ch5ReducedAuditV23_m', ...
        'ch5ReducedJacobianV17_m','ch5ReducedCollisionAuditV22_m','ch5ReducedCheckRolloutV22_m', ...
        'ch5ReducedHoldAuditV22_m','ch5ReducedRecoveryFast_m'};
    inspectGroup('closed_short_v45','horizon_audit_v45',[common,{'ch5ReducedControlV45_m','ch5ReducedStepV33_m', ...
        'ch5ReducedV45Sfun_m','ch5ReducedPrepareV45_m','ch5ReducedKernelsV45_m','ch5ReducedPreviewV43_m','ch5ReducedPriorV43_m'}],false);
    inspectGroup('closed_blocked_v48','blocking_audit_v48',[common,{'ch5ReducedControlV48_m','ch5ReducedStepV48_m', ...
        'ch5ReducedFeedbackRolloutV48_m','ch5ReducedV48Sfun_m','ch5ReducedPrepareV48_m','ch5ReducedKernelsV4_m','ch5ReducedPriorV21_m'}],false);
    required=[common,{'ch5ReducedControlV50_m','ch5ReducedStepV50_m','ch5ReducedFeedbackRolloutV50_m', ...
        'ch5ReducedV50Sfun_m','ch5ReducedPrepareV48_m','ch5ReducedKernelsV4_m','ch5ReducedPriorV21_m'}];
    inspectGroup('closed_blocked_v50','blocking_audit_v50',required,false);
    if isfolder(fullfile(base,'recovery_blocked_v50')),inspectGroup('recovery_blocked_v50','blocking_audit_v50',required,true);end
    inspectGroup('closed_preheated_v54','blocking_audit_v50',required,false);
    hot=jsondecode(fileread(fullfile(base,'closed_preheated_v54','004_nonideal_force.json')));
    assert(hot.preparation.execution_warmup_error==0&&strcmp(hot.preparation.execution_warmup_entry_sha256, ...
        report.identity.ch5SpeedReducedV54ClosedLoop_m),'ch5:WarmupMismatch','Preheated episode must record its exact preparation source and consistency check.');
    retained=jsondecode(fileread(fullfile(base,'refactor_final_audit_v37','summary.json')));
    assert(retained.accepted&&numel(retained.episodes)==7,'ch5:OriginalRegression','The retained V33 development regression is required.');
    for name=common,assert(strcmp(retained.identity.(name{1}),report.identity.(name{1})),'ch5:SourceDrift','Shared accepted source changed.');end
    [userMemory,systemMemory]=memory;
    current=evalin('base','ch5Runtime');report.active_template=struct('H',current.c.H,'Ts',current.c.Ts, ...
        'controller',get_param([current.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName'));
    report.memory_snapshot=struct('matlab_used_bytes',userMemory.MemUsedMATLAB,'available_physical_bytes',systemMemory.PhysicalMemory.Available, ...
        'measurement','Final snapshot only; no continuous peak measurement');report.accepted=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();fprintf('Literature final integrity audit: %d sources and %d new episodes, PID%d\n',numel(files),numel(report.episodes),report.pid);
    function inspectGroup(group,contractName,required,fault)
        contract=jsondecode(fileread(fullfile(base,contractName,'summary.json')));assert(contract.accepted,'ch5:ContractFailure','Independent contract failed.');
        episodes=dir(fullfile(base,group,'00*force.json'));assert(~isempty(episodes),'ch5:MissingEpisode','At least one recorded episode is required.');
        for ej=1:numel(episodes)
            path=fullfile(episodes(ej).folder,episodes(ej).name);row=jsondecode(fileread(path));raw=strrep(path,'.json','.mat');
            assert(isfile(raw),'ch5:MissingRaw','Raw MAT is required.');data=load(raw,'records','predictions');
            assert(numel(data.records)==751&&numel(data.predictions)==751&&abs(data.records{end}.time-15)<1e-9, ...
                'ch5:IncompleteRaw','Each reported 15-second episode must retain all controls and predictions.');
            assert(row.metrics.simulated_end_seconds==15&&isempty(row.simulation_error)&&row.fault_injection==fault, ...
                'ch5:EpisodeMismatch','Keep task failure distinct from a missing or errored simulation.');
            for rj=1:numel(required)
                name=required{rj};assert(strcmp(row.code_identity.(name),report.identity.(name))&& ...
                    strcmp(contract.identity.(name),report.identity.(name)),'ch5:SourceDrift','Episode source differs from its independent contract.');
            end
            report.episodes{end+1}=struct('path',path,'safe_success',row.metrics.safe_success,'arrival',row.metrics.arrival, ...
                'all_constraints_valid',row.metrics.all_constraints_valid,'fault',fault,'qp_failures',row.metrics.predictive_solver_failures, ...
                'fallbacks',row.metrics.fallback_count,'complete_records',true,'identity_matched',true);
        end
    end
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
