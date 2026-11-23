function report=ch5SpeedConditionAuditV20()
clear ch5ReducedSolveV19 ch5ReducedJacobianV17 ch5ReducedStepV20 ch5ReducedControlV20 ch5ReducedV20Sfun
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'condition_audit_v20');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve every acceptance attempt.');mkdir(directory);
report=struct('accepted',true,'pid',feature('getpid'),'identity',ch5ReducedIdentity(),'control_rows',{{}},'matrix_rows',{{}},'steady_rows',{{}},'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
        source=load(fullfile(base,'closed_warm_v11',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        clearSensitivity();
        for index=[1,26,31,191,276,281,286,471]
            sample=source.records{index};before=reconstruct(source,index,r,k);count=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)));
            install(before,r,count);timer=tic;[u,d,p]=ch5ReducedControlV20(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);seconds=toc(timer);
            independent=false;if p.chosen,gate=ch5ReducedAuditGuardedV10(sample.x,sample.previous_u,p.candidates{1}.U,sample.goal,sample.observed,r,k,zeros(6,r.c.H));independent=gate.accepted;end
            a=p.denoise_attempts{1};row=struct('mode',char(mode),'time',sample.time,'seconds',seconds,'chosen',p.chosen,'fallback',d.usedFallback, ...
                'solver',a,'independent_gate_accepted',independent,'accepted',(p.chosen&&independent)||(~p.chosen&&d.usedFallback));
            report.control_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
            fprintf('Condition control %s t%.1f %.1fms flag%d reuse%d/%d chosen%d\n',mode,sample.time,1000*seconds,a.exitflag,a.sensitivity.reused_stages,r.c.H,p.chosen);
        end
        sample=source.records{276};U=source.predictions{276}.prediction.candidates{1}.U;prior=16*ones(6,r.c.H);
        exact=r;exact.reduced.reuse_sensitivities=false;[~,oldMatrix]=ch5ReducedStepV9(sample.x,sample.previous_u,U,sample.goal,sample.observed,prior,exact,k,true);
        [~,newMatrix]=ch5ReducedStepV20(sample.x,sample.previous_u,U,sample.goal,sample.observed,prior,exact,k,sample.time,true);
        errors=[];for field={'Q','f','A','b','E','e','M','nodes'},name=field{1};errors(end+1)=norm(oldMatrix.(name)-newMatrix.(name),inf);end
        row=struct('mode',char(mode),'errors',errors,'accepted',max(errors)<1e-8);report.matrix_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
        % At an exact equilibrium, shift/reuse must reproduce the fresh Jacobians.
        ua=full(k.equilibrium(r.task.q0));x=[r.task.q0;zeros(6,1)];if r.plant.n>12,x=[x;repmat(ua,(r.plant.n-12)/6,1)];end
        U=repmat(ua,1,r.c.H);nodes=full(k.nodes(x,U));clearSensitivity();
        ch5ReducedJacobianV17(nodes,U,r.task.q0,0,r,k);[A,B,detail]=ch5ReducedJacobianV17(nodes,U,r.task.q0,.1,r,k);
        [~,freshA,freshB]=k.stages(nodes(:,1:r.c.H),U);error=max(norm(A-full(freshA),inf),norm(B-full(freshB),inf));
        [~,~,changed]=ch5ReducedJacobianV17(nodes,U,r.task.q0+[.001;zeros(5,1)],.2,r,k);
        row=struct('mode',char(mode),'reused',detail.reused_stages,'fresh',detail.fresh_stages,'maximum_age',detail.maximum_age_seconds, ...
            'derivative_error',error,'changed_goal_reused',changed.reused_stages,'accepted',detail.reused_stages==55&&error<1e-8&&changed.reused_stages==0);
        report.steady_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
    end
    r=ch5ReducedPrepare(1,"nonideal-force");r.reduced=settings();r.reduced.force_rejection_window=[0,.1];install([],r,0);clearSensitivity();
    x=[r.task.q0;r.task.qd0;repmat(r.initialForce,3,1)];previous=r.initialForce;refresh=false(1,6);fallback=refresh;cacheCleared=refresh;
    for step=0:5
        time=step*r.c.Ts;[goal,o]=ch5Observe(r.task,time);[u,d,p]=ch5ReducedControlV20(x,previous,goal,o,r,time);
        refresh(step+1)=p.refresh;fallback(step+1)=d.usedFallback;cacheCleared(step+1)=~isappdata(0,'ch5ReducedSensitivity');
        x=full(r.plant.fd(x,u,zeros(6,1)));previous=u;
    end
    report.failure_clock=struct('refresh',refresh,'fallback',fallback,'cache_cleared',cacheCleared,'accepted', ...
        isequal(refresh,logical([1,0,0,0,0,1]))&&all(fallback(1:5))&&all(cacheCleared(1:5))&&~fallback(6));
    report.accepted=report.accepted&&report.failure_clock.accepted;
    bad=load(fullfile(base,'closed_latest_v2','003_nonideal_force.mat'),'records');sample=bad.records{8};
    check=ch5ReducedHoldAudit(sample.x,sample.previous_u,sample.u,sample.observed,r);report.old_transient_rejected=~check.accepted;report.accepted=report.accepted&&~check.accepted;
catch ex
    report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);
end
write(directory,report);assert(report.accepted,'ch5:UnverifiedConditioning','Preserve failed independent acceptance.');
end

function saved=reconstruct(source,index,r,k)
saved=[];if index==1,return;end
last=find(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)),1,'last');
p=source.predictions{last}.prediction;sample=source.records{last};trajectory=[];
anchorIndex=find(cellfun(@(p)p.prediction.chosen,source.predictions(1:last)),1,'last');
anchor=r.task.q0;if ~isempty(anchorIndex),anchor=source.records{anchorIndex}.x(1:6);end
if p.chosen,trajectory=p.candidates{1};end
saved=struct('time',sample.time,'refresh_time',sample.time,'trajectory_time',sample.time, ...
    'goal',sample.goal,'observed',sample.observed,'anchor',anchor,'trajectory',trajectory);
for j=last+1:index-1
    sample=source.records{j};p=source.predictions{j}.prediction;
    if ~isempty(saved.trajectory)
        warm=ch5ReducedWarmV11(sample.x,sample.previous_u,saved,r,k,sample.time);
        if p.chosen,saved.trajectory=warm;saved.anchor=sample.x(1:6);else,saved.trajectory=[];end
    end
    saved.time=sample.time;saved.trajectory_time=sample.time;saved.goal=sample.goal;saved.observed=sample.observed;
end
end
function install(saved,r,refreshCount)
if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
if ~isempty(saved),setappdata(0,'ch5ReducedExecution',saved);end
generator=RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index));
for j=1:refreshCount,randn(generator,6*r.c.H,r.c.candidates);end
setappdata(0,'ch5Random',generator);
end
function options=settings()
options=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
    'factor_derivatives',false,'retain_numerical',false,'restoration_attempts',0,'reuse_sensitivities',true);
end
function clearSensitivity()
if isappdata(0,'ch5ReducedSensitivity'),rmappdata(0,'ch5ReducedSensitivity');end
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
