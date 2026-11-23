function report=ch5SpeedRefactorFinalAuditV37()
% Read-only final audit; does not change the formal campaign or controller.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');
directory=fullfile(base,'refactor_final_audit_v37');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve prior audit results.');mkdir(directory);
report=struct('pid',feature('getpid'),'accepted',false,'formal_ready',false, ...
    'identity',ch5ReducedIdentity(),'syntax_rows',{{}},'episodes',{{}},'error','');
try
    files=dir(fullfile(root,'chapter5','reduced','*.m'));
    for j=1:numel(files)
        issues=checkcode(fullfile(files(j).folder,files(j).name),'-id');
        syntax=any(strcmp({issues.id},'SYNER'));
        report.syntax_rows{end+1}=struct('file',files(j).name,'syntax_error',syntax);
        assert(~syntax,'ch5:SyntaxFailure','A retained development source has a syntax error.');
    end
    contract=jsondecode(fileread(fullfile(base,'numerics_audit_v33c','summary.json')));
    assert(contract.accepted,'ch5:ContractFailure','V33 independent numerical checks failed.');
    required={'ch5ReducedControlV33_m','ch5ReducedStepV33_m','ch5ReducedSolveV24_m', ...
        'ch5ReducedV33Sfun_m','ch5ReducedTerminalV23_m','ch5ReducedWarmV23_m','ch5ReducedAuditV23_m', ...
        'ch5ReducedJacobianV17_m','ch5ReducedPriorV21_m','ch5ReducedCollisionAuditV22_m', ...
        'ch5ReducedCheckRolloutV22_m','ch5ReducedHoldAuditV22_m','ch5ReducedRecoveryFast_m'};
    for task=[1,3,4]
        for mode=["ideal-force","nonideal-force"]
            inspect(fullfile(base,'closed_generic_v33',sprintf('%03d_%s.json',task,strrep(mode,'-','_'))),false);
        end
    end
    inspect(fullfile(base,'recovery_generic_v33','001_nonideal_force.json'),true);
    [userMemory,systemMemory]=memory;
    report.memory_snapshot=struct('matlab_used_bytes',userMemory.MemUsedMATLAB, ...
        'available_physical_bytes',systemMemory.PhysicalMemory.Available, ...
        'measurement','One final snapshot; not a continuous peak.');
    report.accepted=true;
catch ex
    report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);
end
write();fprintf('Final development audit: %d source files, %d complete episodes, PID %d\n',numel(files),numel(report.episodes),report.pid);

    function inspect(path,fault)
        row=jsondecode(fileread(path));
        assert(row.metrics.safe_success&&row.metrics.all_constraints_valid&&row.metrics.simulated_end_seconds==15, ...
            'ch5:IncompleteRegression','All complete V33 development runs must reach safely.');
        assert(isempty(row.simulation_error)&&row.fault_injection==fault,'ch5:EpisodeError','Runtime error or incorrect probe identity.');
        assert(isfile(strrep(path,'.json','.mat')),'ch5:MissingRaw','Raw episode is required.');
        for ri=1:numel(required)
            name=required{ri};
            assert(strcmp(row.code_identity.(name),report.identity.(name))&&strcmp(contract.identity.(name),report.identity.(name)), ...
                'ch5:SourceDrift','Selected dependency changed after independent numerical checks.');
        end
        report.episodes{end+1}=struct('path',path,'safe_success',true,'fault_injection',fault, ...
            'qp_nonconvergence',row.metrics.predictive_solver_failures,'fallbacks',row.metrics.fallback_count);
    end
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
