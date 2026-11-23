function report=ch5SpeedScreenAuditV13()
clear ch5ReducedAuditV12 ch5ReducedControlV13 ch5ReducedRestoreV12 ch5ReducedV13Sfun
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');
assert(~isfile(fullfile(base,'screen_audit_v13','summary.json')),'ch5:ExistingAudit','Retain the admitted audit.');
attempt=1;while isfolder(fullfile(base,sprintf('screen_audit_v13_attempt%02d',attempt))),attempt=attempt+1;end
directory=fullfile(base,sprintf('screen_audit_v13_attempt%02d',attempt));mkdir(directory);
report=struct('accepted',true,'pid',feature('getpid'),'identity',ch5ReducedIdentity(), ...
    'full_gate_rows',{{}},'control_rows',{{}},'restoration_rows',{{}},'failure_clock',[],'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
        source=load(fullfile(base,'closed_warm_v11',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        for index=[1,26,31,276,321,401,701]
            sample=source.records{index};before=reconstruct(source,index,r,k);refreshCount=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)));
            exact=r;exact.reduced.retain_numerical=false;exact.reduced.restoration_attempts=0;
            install(before,r,refreshCount);timer=tic;
            [reference,referenceD,referenceP]=ch5ReducedControlV11(sample.x,sample.previous_u,sample.goal,sample.observed,exact,sample.time);oldSeconds=toc(timer);
            install(before,r,refreshCount);timer=tic;
            [u,d,p]=ch5ReducedControlV13(sample.x,sample.previous_u,sample.goal,sample.observed,exact,sample.time);newSeconds=toc(timer);
            sequenceError=0;costError=0;
            if referenceP.chosen&&referenceP.refresh
                sequenceError=norm(p.candidates{1}.U-referenceP.candidates{1}.U,inf);costError=abs(p.cost-referenceP.cost);
            end
            row=struct('mode',char(mode),'sample',index,'time',sample.time,'force_error',norm(u-reference,inf), ...
                'sequence_error',sequenceError,'cost_error',costError,'old_seconds',oldSeconds,'new_seconds',newSeconds, ...
                'screens',p.objective_screens,'full_audits',p.full_candidate_audits,'accepted', ...
                norm(u-reference,inf)<1e-8&&sequenceError<1e-8&&costError<1e-12&&p.chosen==referenceP.chosen&&d.usedFallback==referenceD.usedFallback);
            report.control_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
            fprintf('Screen replay %s t%.2f accepted=%d screened%d %.1f -> %.1fms\n',mode,sample.time,row.accepted,p.objective_screens,1000*oldSeconds,1000*newSeconds);
            prior=ones(6,r.c.H)*16;candidate=source.predictions{index}.prediction.candidates{1};
            if ~isempty(candidate)&&size(candidate.U,2)==r.c.H
                U=candidate.U;[old,oldT,oldCost]=ch5ReducedAuditGuardedV10(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
                if old.equality_tolerance_ratio>1
                    U=ch5ReducedTerminal(sample.x,U,r,k);
                    [old,oldT,oldCost]=ch5ReducedAuditGuardedV10(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
                end
                [newU,new,newT,newCost]=ch5ReducedAuditV12(sample.x,sample.previous_u,candidate.U,sample.goal,sample.observed,r,k,prior);
                errors=[norm(newU-U,inf),norm(newT.X-oldT.X,inf),abs(newCost-oldCost), ...
                    abs(new.equality_tolerance_ratio-old.equality_tolerance_ratio),abs(new.inequality_violation-old.inequality_violation)];
                gateRow=struct('mode',char(mode),'sample',index,'errors',errors,'accepted',max(errors)<1e-10&&old.accepted==new.accepted);
                report.full_gate_rows{end+1}=gateRow;report.accepted=report.accepted&&gateRow.accepted;
                if new.accepted
                    incumbent=struct('accepted',true,'cost',newCost-1);
                    [~,screen,~,screenCost,screenInfo]=ch5ReducedAuditV12(sample.x,sample.previous_u,newU,sample.goal,sample.observed,r,k,prior,incumbent);
                    assert(screenInfo.screened_for_cost&&~screenInfo.full_audit_performed&&~screen.accepted&&screenCost==newCost, ...
                        'ch5:ObjectiveScreen','A nonimproving trial must be screened without claiming physical admission.');
                end
            end
        end
        index=31;sample=source.records{index};before=reconstruct(source,index,r,k);
        refreshCount=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)));install(before,r,refreshCount);
        [u,d,p]=ch5ReducedControlV13(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);
        independentAccepted=false;
        if p.chosen
            independent=ch5ReducedAuditGuardedV10(sample.x,sample.previous_u,p.candidates{1}.U,sample.goal,sample.observed,r,k,zeros(6,r.c.H));
            independentAccepted=independent.accepted;
        end
        restoreRow=struct('mode',char(mode),'time',sample.time,'chosen',logical(p.chosen),'fallback',d.usedFallback, ...
            'attempts',{p.restoration_attempts},'independent_gate_accepted',independentAccepted,'numerical_retained',p.numerical_retained, ...
            'accepted',(p.chosen&&independentAccepted)||(~p.chosen&&d.usedFallback&&p.numerical_retained));
        report.restoration_rows{end+1}=restoreRow;report.accepted=report.accepted&&restoreRow.accepted;
        fprintf('Restoration replay %s chosen%d fallback%d attempts%d\n',mode,p.chosen,d.usedFallback,numel(p.restoration_attempts));
    end
    r=ch5ReducedPrepare(1,"nonideal-force");r.reduced=settings();r.reduced.force_rejection_window=[0,.1];
    install([],r,0);x=[r.task.q0;r.task.qd0;repmat(r.initialForce,3,1)];previous=r.initialForce;
    refresh=false(1,6);fallback=refresh;retained=refresh;failed=refresh;used=refresh;
    for step=0:5
        time=step*r.c.Ts;[goal,o]=ch5Observe(r.task,time);
        [u,d,p]=ch5ReducedControlV13(x,previous,goal,o,r,time);
        refresh(step+1)=p.refresh;fallback(step+1)=d.usedFallback;retained(step+1)=p.numerical_retained;
        failed(step+1)=p.full_horizon_failure_active;used(step+1)=p.used_retained_numerical;
        x=full(r.plant.fd(x,u,zeros(6,1)));previous=u;
    end
    report.failure_clock=struct('refresh',refresh,'fallback',fallback,'numerical_retained',retained, ...
        'full_failure_active',failed,'used_retained_numerical',used,'accepted', ...
        isequal(refresh,logical([1,0,0,0,0,1]))&&all(fallback(1:5))&&all(retained)&&all(failed(1:5))&&~fallback(6)&&used(6));
    report.accepted=report.accepted&&report.failure_clock.accepted;
    bad=load(fullfile(base,'closed_latest_v2','003_nonideal_force.mat'),'records');sample=bad.records{8};
    oldCheck=ch5ReducedHoldAudit(sample.x,sample.previous_u,sample.u,sample.observed,r);
    report.old_transient_rejected=~oldCheck.accepted;report.accepted=report.accepted&&report.old_transient_rejected;
catch ex
    report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);
end
write(directory,report);
assert(report.accepted,'ch5:ScreenUnverified','Save the failed audit and fix it before closed-loop expansion.');
directory=fullfile(base,'screen_audit_v13');mkdir(directory);write(directory,report);
fprintf('Screen audit accepted=%d controls%d full gates%d\n',report.accepted,numel(report.control_rows),numel(report.full_gate_rows));
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
    'factor_derivatives',false,'retain_numerical',true,'restoration_attempts',1);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
