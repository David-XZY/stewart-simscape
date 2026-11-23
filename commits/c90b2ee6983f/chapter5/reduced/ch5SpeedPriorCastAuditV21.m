function report=ch5SpeedPriorCastAuditV21()
clear ch5ReducedPriorV21 ch5ReducedControlV21 ch5ReducedV21Sfun
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'prior_cast_audit_v21');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve attempts.');mkdir(directory);
contract=jsondecode(fileread(fullfile(base,'condition_audit_v20','summary.json')));identity=ch5ReducedIdentity();
required={'ch5ReducedJacobianV17_m','ch5ReducedSolveV19_m','ch5ReducedStepV20_m','ch5ReducedControlV20_m','ch5ReducedAuditV12_m','ch5ReducedHoldAudit_m','ch5ReducedHoldKernel_m','ch5ReducedTransient_m','ch5ReducedRecoveryFast_m'};
assert(contract.accepted,'ch5:Unverified','The conditioning/refresh/physical contracts must pass.');
for j=1:numel(required),assert(strcmp(identity.(required{j}),contract.identity.(required{j})),'ch5:StaleAudit','Changed after conditioning acceptance.');end
old=fileread(fullfile(root,'chapter5','reduced','ch5ReducedControlV20.m'));fresh=fileread(fullfile(root,'chapter5','reduced','ch5ReducedControlV21.m'));
expected=strrep(strrep(old,'ch5ReducedControlV20','ch5ReducedControlV21'),'ch5ReducedPrior(x,previous','ch5ReducedPriorV21(x,previous');
assert(strcmp(fresh,expected),'ch5:ControllerMismatch','The combined controller differs beyond audited prior conversion.');
report=struct('accepted',true,'identity',identity,'pid',feature('getpid'),'condition_contract_accepted',contract.accepted,'rows',{{}});
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=struct('factor_derivatives',false);k=ch5ReducedKernelsV4(r);
    source=load(fullfile(base,'closed_warm_v11',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records');sample=source.records{276};
    for seed=5101:5105
        r.weights=load(fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed)));r.seed=seed;
        g=RandStream('mt19937ar','Seed',7100+seed);state=g.State;setappdata(0,'ch5Random',g);timer=tic;
        [reference,refInfo]=ch5ReducedPrior(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);oldSeconds=toc(timer);
        g.State=state;timer=tic;[prior,info]=ch5ReducedPriorV21(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);newSeconds=toc(timer);
        error=norm(reference-prior,inf);scoreError=norm(info.scores-refInfo.scores,inf);
        row=struct('mode',char(mode),'seed',seed,'maximum_prior_error',error,'score_error',scoreError,'selected',info.selected, ...
            'old_seconds',oldSeconds,'new_seconds',newSeconds,'accepted',error==0&&scoreError==0&&info.selected==refInfo.selected);
        report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
        fprintf('Prior cast %s seed%d error%.3g %.1f -> %.1fms\n',mode,seed,error,1000*oldSeconds,1000*newSeconds);
    end
end
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:PriorCastMismatch','Keep all inference mismatches.');
end
