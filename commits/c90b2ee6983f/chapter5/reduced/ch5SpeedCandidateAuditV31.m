function report=ch5SpeedCandidateAuditV31()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'candidate_audit_v31');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'completed',false,'error','');
try
    source=load(fullfile(base,'closed_generic_v28','004_nonideal_force.mat'),'records','predictions');
    for seed=5101:5105
        r=ch5ReducedPrepare(4,"nonideal-force");r.seed=seed;r.weights=load(fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed)));
        r.reduced=struct('velocity_weight',1,'factor_derivatives',false,'reuse_sensitivities',true,'linearization_collision_reserve',.0002,'qp_row_scaling',true,'trust_radius',1);
        k=ch5ReducedKernelsV4(r);
        for time=[.6,9.3,13]
            idx=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{idx};
            generator=RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index));
            count=sum(cellfun(@(v)v.prediction.refresh,source.predictions(1:idx-1)));
            for j=1:count,randn(generator,6*r.c.H,r.c.candidates);end;setappdata(0,'ch5Random',generator);
            [prior,priorInfo,candidates]=ch5ReducedPriorCandidatesV31(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);
            warm=source.predictions{idx}.prediction.candidates{1}.U;
            [~,warmCheck,~,warmCost]=ch5ReducedAuditV23(sample.x,sample.previous_u,warm,sample.goal,sample.observed,r,k,prior);
            row=struct('seed',seed,'time',time,'warm_cost',warmCost,'warm_accepted',warmCheck.accepted,'prior',priorInfo,'candidates',{{}},'best_valid_cost',inf,'best_valid_candidate',0);
            for ci=1:numel(candidates)
                timer=tic;[U,check,trajectory,cost,audit]=ch5ReducedAuditV23(sample.x,sample.previous_u,candidates{ci}.U,sample.goal,sample.observed,r,k,prior);seconds=toc(timer);
                row.candidates{end+1}=struct('candidate',ci,'U',U,'X',trajectory.X,'check',check,'cost',cost,'audit',audit,'seconds',seconds);
                if check.accepted&&cost<row.best_valid_cost,row.best_valid_cost=cost;row.best_valid_candidate=ci;end
            end
            report.rows{end+1}=row;
            fprintf('Direct candidate seed%d t%.1f valid%d/4 bestcost %.5g warm %.5g\n',seed,time, ...
                sum(cellfun(@(v)v.check.accepted,row.candidates)),row.best_valid_cost,warmCost);
            write(directory,report);
        end
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
for j=1:numel(summary.rows)
    for t=1:numel(summary.rows{j}.candidates),summary.rows{j}.candidates{t}=rmfield(summary.rows{j}.candidates{t},{'U','X'});end
end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
