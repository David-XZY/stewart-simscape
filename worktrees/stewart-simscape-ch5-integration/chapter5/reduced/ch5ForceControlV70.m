function [u,d,prediction]=ch5ForceControlV70(x,previous,goal,o,r,time)
% Four v-force candidates share one physical linearization. Admission uses exact rollouts.
k=ch5ForceKernelsV64(r);saved=[];if isappdata(0,'ch5ReducedExecution'),saved=getappdata(0,'ch5ReducedExecution');end
[terminalUsed,u,d,prediction,saved]=ch5ForceTerminalExecuteV70(x,previous,goal,o,r,saved,time);
if terminalUsed,setappdata(0,'ch5ReducedExecution',saved);return;end
if ~isempty(saved)&&isfield(saved,'terminal_active')&&saved.terminal_active
    saved.terminal_active=false;saved.trajectory=[];saved.refresh_time=-inf;
end
eventChanged=false;
if ~isempty(saved)
    dt=time-saved.observation_time;predictedCenter=saved.observed(1:3,:)+saved.observed(16:18,:)*dt;
    active=(o(22,:)>.5)|(saved.observed(22,:)>.5);
    eventChanged=any(abs(o(4:15,active)-saved.observed(4:15,active))>1e-10,'all')|| ...
        any(abs(o(16:18,active)-saved.observed(16:18,active))>1e-4,'all')|| ...
        any(abs(o(1:3,active)-predictedCenter(:,active))>.001,'all');
end
refresh=isempty(saved)||eventChanged||time-saved.refresh_time>=.1-1e-10|| ...
    norm(goal-saved.goal)>1e-10||any(o(22,:)~=saved.observed(22,:));
attempts={};rawChecks={};checks={};trajectory=[];cost=inf;priorInfo=[];acceptedRaw=0;acceptedProjected=0;
rawAudited=0;anchor=x(1:6);if ~isempty(saved),anchor=saved.anchor;end
phases=struct('warm_start',0,'prior',0,'incumbent_check',0,'prediction_step',0,'trial_checks',0,'recovery',0);timer=tic;
if ~refresh&&isempty(saved.trajectory)
    [u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);
    prediction=struct('refresh',false,'chosen',0,'candidates',{{}},'checks',{{}},'raw_checks',{{}},'raw_audited',0,'raw_accepted',0, ...
        'projected_accepted',0,'denoise_attempts',{{}},'final_attempts',{{}},'restoration_attempts',{{}},'goal',goal,'observed',o, ...
        'cost',inf,'phases',phases,'full_horizon_checked',false,'validation_scope','recovery_executed_interval', ...
        'full_horizon_failure_active',true,'recovery_active',true,'terminal_certified',false,'algorithm_revision','force-v70-terminal-execution-development');
    saved.goal=goal;saved.observed=o;saved.observation_time=time;setappdata(0,'ch5ReducedExecution',saved);return;
end
if ~refresh
    shift=round((time-saved.trajectory_time)/r.c.Ts);old=saved.trajectory;
    K=ch5ReducedFeedbackGain(x,r,k);u=old.U(:,shift+1)-K*(x-old.X(:,shift+1));phases.warm_start=toc(timer);timer=tic;
    [check,trajectory]=ch5ReducedHoldAuditV22(x,previous,u,o,r);phases.incumbent_check=toc(timer);
else
    ch5ForceTerminalV62();
    if ~isempty(saved)&&~isempty(saved.trajectory)
        [U,qs]=ch5ForceWarmV57(x,previous,saved,r,k,time);
    elseif ~isempty(saved)&&isfield(saved,'numerical_trajectory')&&~isempty(saved.numerical_trajectory)
        shift=min(r.c.H,max(0,round((time-saved.numerical_time)/r.c.Ts)));old=saved.numerical_trajectory;
        U=[old.U(:,shift+1:end),repmat(old.U(:,end),1,shift)];qs=old.artificial_pose;
    else
        brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));qs=[];
    end
    phases.warm_start=toc(timer);timer=tic;
    % A numerical SQP iterate needs no admission until selected for execution.
    nodes=full(k.nodes(x,U));if isempty(qs),qs=nodes(1:6,end);end
    trajectory=struct('X',nodes,'U',U,'artificial_pose',qs);
    check=struct('accepted',false,'scope','unaudited_numerical_initializer');cost=inf;
    phases.incumbent_check=toc(timer);incumbent=trajectory;timer=tic;
    generator=getappdata(0,'ch5Random');noise=randn(generator,360,4);
    [prior,~,priorInfo]=ch5ForceSampleV65(ch5Context(x,previous,goal,o,r.mode),previous,r.force_weights,noise);phases.prior=toc(timer);
    timer=tic;
    % Raw generation is recorded before input clipping or any physical projection.
    doRaw=~isfield(r.reduced,'audit_raw_candidates')||r.reduced.audit_raw_candidates;
    if doRaw
    rawAudited=4;
    for b=1:4
        [rawCheck,rawTrajectory,rawCost]=ch5ForceAuditV62(x,previous,prior(:,:,b),goal,o,r,k,[]);
        rawChecks{b}=rawCheck;acceptedRaw=acceptedRaw+double(rawCheck.accepted);
        if rawCheck.accepted&&(~check.accepted||rawCost<cost),check=rawCheck;trajectory=rawTrajectory;cost=rawCost;end
    end
    end
    phases.trial_checks=toc(timer);timer=tic;
    [proposals,poses,solves]=ch5ForceStepV66(x,previous,incumbent.U,goal,o,prior,r,k,time,incumbent.artificial_pose);
    phases.prediction_step=toc(timer);timer=tic;
    ranking=inf(1,4);for b=1:4,if isfield(solves{b},'predicted_task_cost'),ranking(b)=solves{b}.predicted_task_cost;end;end
    [~,order]=sort(ranking);limit=4;if isfield(r.reduced,'max_verified_candidates'),limit=r.reduced.max_verified_candidates;end
    verified=0;
    for b=order
        attempt=solves{b};
        if verified>=limit,attempt.retained=false;attempt.trials={};attempt.audit_skipped_by_budget=true;attempts{b}=attempt;continue;end
        verified=verified+1;attempt.retained=false;attempt.trials={};
        if attempt.finite
            for line=0:3
                a=2^(-line);trial=incumbent.U+a*(proposals(:,:,b)-incumbent.U);qs=incumbent.artificial_pose+a*(poses(:,b)-incumbent.artificial_pose);
                [trial,trialCheck,trialTrajectory,trialCost,retraction]=ch5ForceRetractV62(x,previous,trial,goal,o,r,k,qs);
                attempt.trials{end+1}=struct('line',line,'check',trialCheck,'cost',trialCost,'terminal_retraction',retraction);
                if trialCheck.accepted
                    acceptedProjected=acceptedProjected+1;
                    if ~check.accepted||trialCost<cost,check=trialCheck;trajectory=trialTrajectory;cost=trialCost;attempt.retained=true;end
                    break;
                end
            end
        end
        attempts{b}=attempt;
    end
    phases.trial_checks=phases.trial_checks+toc(timer);
    if ~check.accepted
        timer=tic;[ic,it,icost]=ch5ForceAuditV62(x,previous,incumbent.U,goal,o,r,k,incumbent.artificial_pose);
        if ~ic.accepted
            [restored,~]=ch5ReducedTerminalV23(x,incumbent.U,r,k,incumbent.X);
            [ic,it,icost]=ch5ForceAuditV62(x,previous,restored,goal,o,r,k,[]);
        end
        phases.incumbent_check=phases.incumbent_check+toc(timer);
        if ic.accepted,check=ic;trajectory=it;cost=icost;end
    end
    u=trajectory.U(:,1);
end
if isfield(r.reduced,'force_rejection_window')&&time>=r.reduced.force_rejection_window(1)&&time<r.reduced.force_rejection_window(2)
    check.accepted=false;check.rejection_reason='development_fault_injection';
end
if check.accepted
    d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);
    if refresh,saved=struct('trajectory',trajectory,'trajectory_time',time,'refresh_time',time,'anchor',x(1:6));end
else
    timer=tic;[u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);phases.recovery=toc(timer);
    if isempty(saved),saved=struct('trajectory',[],'trajectory_time',time,'refresh_time',time,'anchor',anchor);else,saved.trajectory=[];end
end
if refresh,saved.refresh_time=time;saved.numerical_time=time;saved.numerical_trajectory=trajectory;end
saved.goal=goal;saved.observed=o;saved.observation_time=time;setappdata(0,'ch5ReducedExecution',saved);
prediction=struct('refresh',refresh,'chosen',double(check.accepted),'candidates',{{trajectory}},'checks',{{check}}, ...
    'raw_audited',rawAudited,'raw_checks',{rawChecks},'raw_accepted',acceptedRaw,'projected_accepted',acceptedProjected,'denoise_attempts',{attempts}, ...
    'final_attempts',{{}},'restoration_attempts',{{}},'terminal_repair',[],'prior',priorInfo,'goal',goal,'observed',o, ...
    'cost',cost,'phases',phases,'full_horizon_checked',refresh,'validation_scope',check.scope,'full_horizon_failure_active',~check.accepted, ...
    'recovery_active',logical(d.usedFallback),'terminal_certified',false,'algorithm_revision','force-v70-terminal-execution-development');
end
