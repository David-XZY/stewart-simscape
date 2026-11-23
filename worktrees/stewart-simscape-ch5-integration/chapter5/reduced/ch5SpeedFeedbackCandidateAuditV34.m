function report=ch5SpeedFeedbackCandidateAuditV34()
% Test learned candidates through the same internally coupled feedback rollout.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'feedback_candidate_audit_v34');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'executed_force_replays',{{}},'completed',false,'error','');
try
    source=load(fullfile(base,'closed_generic_v28','004_nonideal_force.mat'),'records','predictions');
    for seed=5101:5105
        r=ch5ReducedPrepare(4,"nonideal-force");r.seed=seed;r.weights=load(fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed)));
        r.reduced=struct('velocity_weight',1,'factor_derivatives',false,'reuse_sensitivities',true,'linearization_collision_reserve',.0002,'qp_row_scaling',true,'trust_radius',1);k=ch5ReducedKernelsV4(r);feedback=ch5ReducedFeedbackRollout(r);
        for time=[.6,9.3,13]
            idx=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{idx};
            generator=RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index));count=sum(cellfun(@(v)v.prediction.refresh,source.predictions(1:idx-1)));
            for j=1:count,randn(generator,6*r.c.H,r.c.candidates);end;setappdata(0,'ch5Random',generator);
            [prior,priorInfo,candidates]=ch5ReducedPriorCandidatesV31(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);
            warm=source.predictions{idx}.prediction.candidates{1};gain=ch5ReducedFeedbackGain(sample.x,r,k);
            [~,warmCheck,~,warmCost]=ch5ReducedAuditV23(sample.x,sample.previous_u,warm.U,sample.goal,sample.observed,r,k,prior);
            for blend=[1,.1,.01]
                row=struct('seed',seed,'time',time,'blend',blend,'warm_cost',warmCost,'warm_accepted',warmCheck.accepted,'prior',priorInfo,'candidates',{{}},'best_valid_cost',inf,'best_valid_candidate',0);
                for ci=1:numel(candidates)
                    baseU=warm.U+blend*(candidates{ci}.U-warm.U);
                    proposal=full(feedback(sample.x,warm.X,baseU,gain,zeros(6,r.c.H/5)));
                    timer=tic;[U,check,trajectory,cost,audit]=ch5ReducedAuditV23(sample.x,sample.previous_u,proposal,sample.goal,sample.observed,r,k,prior);seconds=toc(timer);
                    row.candidates{end+1}=struct('candidate',ci,'U',U,'X',trajectory.X,'check',check,'cost',cost,'audit',audit,'seconds',seconds);
                    if check.accepted&&cost<row.best_valid_cost,row.best_valid_cost=cost;row.best_valid_candidate=ci;end
                end
                report.rows{end+1}=row;
                fprintf('Feedback candidate seed%d t%.1f blend%.2g valid%d/4 improvement %.5g\n',seed,time,blend, ...
                    sum(cellfun(@(v)v.check.accepted,row.candidates)),warmCost-row.best_valid_cost);
            end
        end
        write(directory,report);
    end
    r=ch5ReducedPrepare(4,"nonideal-force");r.reduced=struct('factor_derivatives',false);k=ch5ReducedKernelsV4(r);
    for label=["closed_generic_v22","closed_generic_v28"]
        source=load(fullfile(base,label,'004_nonideal_force.mat'),'records');
        for time=[.6,9.3,13]
            idx=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{idx};
            U=cell2mat(cellfun(@(v)v.u,source.records(idx:idx+r.c.H-1),'UniformOutput',false));predicted=full(k.nodes(sample.x,U));
            actual=cell2mat(cellfun(@(v)v.x,source.records(idx:idx+r.c.H),'UniformOutput',false));
            position=vecnorm(predicted(1:3,:)-actual(1:3,:));rotation=zeros(1,r.c.H+1);
            for j=1:r.c.H+1,rotation(j)=acos(min(1,max(-1,(trace(ch5Rotation(actual(4:6,j))'*ch5Rotation(predicted(4:6,j)))-1)/2)));end
            row=struct('source',char(label),'time',time,'position_errors',position,'rotation_errors',rotation, ...
                'maximum_position_error',max(position),'maximum_rotation_error',max(rotation),'U',U,'predicted',predicted,'actual',actual);
            report.executed_force_replays{end+1}=row;
            fprintf('Executed force replay %s t%.1f max %.6fmm %.6fdeg\n',label,time,1000*max(position),rad2deg(max(rotation)));
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
for j=1:numel(summary.executed_force_replays),summary.executed_force_replays{j}=rmfield(summary.executed_force_replays{j},{'U','predicted','actual'});end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
