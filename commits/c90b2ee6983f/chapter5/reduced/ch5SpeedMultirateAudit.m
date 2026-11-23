function report=ch5SpeedMultirateAudit()
clear ch5ReducedControlV10 ch5ReducedHoldAudit ch5ReducedHoldKernel ch5ReducedAuditGuardedV10 ch5ReducedV10Sfun
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','multirate_audit_v10');
assert(~isfile(fullfile(directory,'summary.json')),'ch5:ExistingAudit','Do not overwrite the admitted multirate audit.');
if ~isfolder(directory),mkdir(directory);end
report=struct('hold_rows',{{}},'control_rows',{{}},'replay_rows',{{}},'identity',ch5ReducedIdentity(), ...
    'pid',feature('getpid'),'accepted',true);
for mode=["ideal-force","nonideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
    saved=load(fullfile(root,'chapter5','artifacts','reduced_probe','closed_tapered_v9',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
    for index=[1,26,300,450,700]
        sample=saved.records{index};[check,interval]=ch5ReducedHoldAudit(sample.x,sample.previous_u,sample.u,sample.observed,r);
        [refNext,refTrace]=r.plant.fd(sample.x,sample.u,zeros(6,1));refNext=full(refNext);refTrace=full(refTrace);
        values=zeros(25,size(refTrace,2));gap=inf;
        for j=1:size(refTrace,2)
            values(:,j)=full(r.plant.mechanicalValues(refTrace(:,j),sample.u));
            for oi=find(sample.observed(22,:)>.5)
                ot=sample.observed(:,oi);ot(1:3)=ot(1:3)+r.plant.traceTime(j)*ot(16:18);
                gap=min(gap,full(k.collision(refTrace(1:6,j),ot)));
            end
        end
        native=ch5ReducedHoldKernel(r);[~,nativeTrace,nativeValues]=native(sample.x,sample.u);
        error=[norm(interval.X(:,end)-refNext,inf),norm(full(nativeTrace)-refTrace,inf),norm(full(nativeValues)-values,inf)];
        gapError=0;if isfinite(gap),gapError=abs(check.internal_check.minimum_gap-gap);end
        lower=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
        upper=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
        mechanical=max([lower-values;values-upper;zeros(1,size(values,2))],[],'all');
        collision=max(0,r.c.safe_distance+r.c.sample_hold_margin-gap);
        extra=ch5ReducedTransient(sample.x,sample.u,sample.observed,r);
        force=max([r.strict.forceMin-sample.u;sample.u-r.strict.forceMax;0]);
        rate=max([abs(sample.u-sample.previous_u)/r.c.Ts-r.strict.forceRateLimit;0]);
        expected=all(isfinite(values),'all')&&extra.accepted&&max([mechanical,collision,extra.maximum_violation,force])<=1e-6&&rate<=1e-6;
        row=struct('mode',char(mode),'sample',index,'errors',error,'gap_error',gapError, ...
            'reference_accepted',expected,'native_accepted',check.accepted,'accepted',max(error)<1e-10&&gapError<1e-10&&expected==check.accepted);
        report.hold_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
    end
    invalid=sample.u;invalid(1)=r.strict.forceMax(1)+1;
    invalidCheck=ch5ReducedHoldAudit(sample.x,sample.previous_u,invalid,sample.observed,r);
    report.accepted=report.accepted&&~invalidCheck.accepted;
    for task=[1,4]
        r=ch5ReducedPrepare(task,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
        if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
        setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',5101));
        x=[r.task.q0;r.task.qd0];if r.plant.n>12,x=[x;repmat(r.initialForce,3,1)];end
        previous=r.initialForce;refresh=false(1,6);feasible=refresh;durations=zeros(1,6);scopes=cell(1,6);
        for step=0:5
            time=step*r.c.Ts;[goal,o]=ch5Observe(r.task,time);timer=tic;
            [u,d,p]=ch5ReducedControlV10(x,previous,goal,o,rmfield(r,'task'),time);durations(step+1)=toc(timer);
            refresh(step+1)=p.refresh;feasible(step+1)=d.feasible;scopes{step+1}=p.validation_scope;
            if step==0
                first=p.candidates{1};prior=ones(size(first.U))*16;
                [a,~,c]=ch5ReducedAuditGuardedV10(x,previous,first.U,goal,o,r,k,prior);
                [b,~,cr]=ch5ReducedAuditFastV8(x,previous,first.U,goal,o,r,k,prior);
                assert(a.accepted==b.accepted&&abs(c-cr)<1e-10,'ch5:ChangedFullGate','Refresh acceptance and nonzero prior cost must match v9.');
                bad=first.U;bad(1,1)=r.strict.forceMax(1)+1;
                badCheck=ch5ReducedAuditGuardedV10(x,previous,bad,goal,o,r,k,prior);
                assert(~badCheck.accepted&&badCheck.cheap_reject,'ch5:InputGuard','Invalid force must be rejected algebraically.');
            end
            x=full(r.plant.fd(x,u,zeros(6,1)));previous=u;
        end
        row=struct('mode',char(mode),'task',task,'refresh',refresh,'feasible',feasible,'control_seconds',durations, ...
            'scopes',{scopes},'accepted',isequal(refresh,logical([1,0,0,0,0,1]))&&all(feasible)&& ...
            strcmp(scopes{1},'full_horizon')&&strcmp(scopes{2},'executed_interval')&&strcmp(scopes{6},'full_horizon'));
        report.control_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
        fprintf('Multirate contract %s task%d accepted=%d interval %.2fms\n',mode,task,row.accepted,1000*median(durations(2:5)));
    end
    if mode=="nonideal-force"
        r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
        for index=1:numel(saved.records)
            sample=saved.records{index};p=saved.predictions{index}.prediction;
            if p.chosen||isempty(p.candidates{1}),continue;end
            candidate=p.candidates{1};if isempty(candidate.U),continue;end
            [check,~]=ch5ReducedHoldAudit(sample.x,sample.previous_u,candidate.U(:,1),sample.observed,r);
            [~,~,trace]=k.preview(sample.x,candidate.U);trace=full(trace);
            times=reshape(r.plant.traceTime(:)+(0:r.c.H-1)*r.c.Ts,1,[]);minimumGap=inf;minimumTime=nan;obstacle=0;
            mapped=k.collision.map(numel(times));
            for oi=find(sample.observed(22,:)>.5)
                ot=repmat(sample.observed(:,oi),1,numel(times));ot(1:3,:)=ot(1:3,:)+ot(16:18,1)*times;
                gaps=full(mapped(trace(1:6,:),ot));[value,where]=min(gaps);
                if value<minimumGap,minimumGap=value;minimumTime=times(where);obstacle=oi;end
            end
            report.replay_rows{end+1}=struct('sample',index,'time',sample.time,'refresh',p.refresh, ...
                'executed_interval_accepted',check.accepted,'interval_check',check,'predicted_minimum_gap',minimumGap, ...
                'predicted_minimum_gap_time',minimumTime,'obstacle',obstacle, ...
                'minimum_between_control_nodes',abs(minimumTime/r.c.Ts-round(minimumTime/r.c.Ts))>1e-6);
        end
        bad=load(fullfile(root,'chapter5','artifacts','reduced_probe','closed_latest_v2','003_nonideal_force.mat'),'records');
        sample=bad.records{8};check=ch5ReducedHoldAudit(sample.x,sample.previous_u,sample.u,sample.observed,r);
        report.old_transient_rejected=~check.accepted;report.accepted=report.accepted&&report.old_transient_rejected;
        r=ch5ReducedPrepare(1,mode);r.reduced=settings();r.reduced.force_rejection_window=[.02,.08];
        if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
        setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',5101));
        x=[r.task.q0;r.task.qd0;repmat(r.initialForce,3,1)];previous=r.initialForce;refresh=false(1,6);fallback=refresh;
        for step=0:5
            time=step*r.c.Ts;[goal,o]=ch5Observe(r.task,time);
            [u,d,p]=ch5ReducedControlV10(x,previous,goal,o,rmfield(r,'task'),time);
            refresh(step+1)=p.refresh;fallback(step+1)=d.usedFallback;x=full(r.plant.fd(x,u,zeros(6,1)));previous=u;
        end
        report.failure_clock=struct('refresh',refresh,'fallback',fallback,'accepted',isequal(refresh,logical([1,0,0,0,0,1]))&&all(fallback(2:4)));
        report.accepted=report.accepted&&report.failure_clock.accepted;
    end
end
report.full_horizon_rejections_replayed=numel(report.replay_rows);
report.replayed_first_holds_valid=sum(cellfun(@(row)row.executed_interval_accepted,report.replay_rows));
save(fullfile(directory,'audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Multirate audit accepted=%d; old rejected first holds valid %d/%d\n',report.accepted,report.replayed_first_holds_valid,report.full_horizon_rejections_replayed);
assert(report.accepted,'ch5:MultirateUnverified','Independent physical hold and clock/refresh contracts must pass.');
end

function options=settings()
options=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002,'velocity_weight',1,'factor_derivatives',false);
end
