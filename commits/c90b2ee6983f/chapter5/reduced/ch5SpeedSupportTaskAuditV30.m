function report=ch5SpeedSupportTaskAuditV30()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'support_task_audit_v30');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'accepted',true,'error','');
try
    original=load(fullfile(base,'failure_replay_v23','audit.mat'),'report');
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
        mi=find(cellfun(@(v)strcmp(v.mode,char(mode)),original.report.rows),1);s=original.report.rows{mi}.replay;
        [proposal,qp]=ch5ReducedStepV30(s.x,s.previous,s.warm,s.goal,s.observed,s.prior,r,k,.6);
        [U,check,trajectory,cost,audit]=ch5ReducedAuditV23(s.x,s.previous,proposal,s.goal,s.observed,r,k,s.prior);
        independent=ch5ReducedAuditGuardedV10(s.x,s.previous,U,s.goal,s.observed,r,k,s.prior);
        row=struct('mode',char(mode),'time',.6,'solver',qp,'check',check,'cost',cost,'audit',audit,'U',U,'X',trajectory.X,'accepted',check.accepted&&independent.accepted);
        report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
        fprintf('Task support %s t.6 accepted%d violation %.6fmm QP%d/%dit\n',mode,check.accepted,1000*check.inequality_violation,qp.exitflag,qp.iterations);
        if mode~="nonideal-force",continue;end
        source=load(fullfile(base,'closed_generic_v28','004_nonideal_force.mat'),'records','predictions');
        for time=[1,4.6,9.3,13,14]
            idx=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{idx};U=source.predictions{idx}.prediction.candidates{1}.U;
            generator=RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index));
            for j=1:sum(cellfun(@(v)v.prediction.refresh,source.predictions(1:idx-1))),randn(generator,6*r.c.H,r.c.candidates);end
            setappdata(0,'ch5Random',generator);prior=ch5ReducedPriorV21(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);
            [U,oldCheck,~,oldCost]=ch5ReducedAuditV23(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
            timer=tic;[proposal,qp]=ch5ReducedStepV30(sample.x,sample.previous_u,U,sample.goal,sample.observed,prior,r,k,time);
            [newU,check,trajectory,newCost,audit]=ch5ReducedAuditV23(sample.x,sample.previous_u,proposal,sample.goal,sample.observed,r,k,prior,struct('accepted',oldCheck.accepted,'cost',oldCost));seconds=toc(timer);
            row=struct('mode',char(mode),'time',time,'solver',qp,'check',check,'old_cost',oldCost,'cost',newCost,'cost_decrease',oldCost-newCost, ...
                'seconds',seconds,'audit',audit,'U',newU,'X',trajectory.X,'accepted',check.accepted);report.rows{end+1}=row;
            fprintf('Task support %s t%.1f accepted%d decrease %.6g %.1fms QP%d/%dit\n',mode,time,check.accepted,oldCost-newCost,1000*seconds,qp.exitflag,qp.iterations);
        end
        write(directory,report);
    end
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function s=settings()
s=struct('iterations',2,'backtracking',2,'linearization_collision_reserve',.0002,'velocity_weight',1,'factor_derivatives',false, ...
    'reuse_sensitivities',false,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
for j=1:numel(summary.rows),summary.rows{j}=rmfield(summary.rows{j},{'U','X'});end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
