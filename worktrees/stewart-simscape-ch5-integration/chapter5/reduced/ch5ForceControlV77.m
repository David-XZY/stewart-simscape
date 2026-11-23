function [u,d,prediction]=ch5ForceControlV77(x,previous,goal,o,r,time)
% Generate two priors, solve/audit lazily, expand to four only after failure.
% Offline raw diagnostics never change the selected input.
k=ch5ForceKernelsV64(r);saved=[];if isappdata(0,'ch5ReducedExecution'),saved=getappdata(0,'ch5ReducedExecution');end
[terminalUsed,u,d,prediction,saved]=ch5ForceTerminalExecuteV77(x,previous,goal,o,r,saved,time);
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
attempts={};rawChecks={};trajectory=[];cost=inf;priorInfo={};acceptedRaw=0;acceptedProjected=0;
rawAudited=0;anchor=x(1:6);if ~isempty(saved),anchor=saved.anchor;end
rawGenerated=[];noise=[];context=[];selectionSource='none';chosenCandidate=0;prepared=[];ranking=[];
phases=struct('warm_start',0,'prior',0,'incumbent_check',0,'prediction_step',0,'trial_checks',0,'recovery',0);timer=tic;
if ~refresh&&isempty(saved.trajectory)
    [u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);
    prediction=struct('refresh',false,'chosen',0,'candidates',{{}},'checks',{{}},'raw_checks',{{}},'raw_audited',0,'raw_accepted',0, ...
        'projected_accepted',0,'denoise_attempts',{{}},'final_attempts',{{}},'restoration_attempts',{{}},'goal',goal,'observed',o, ...
        'cost',inf,'phases',phases,'full_horizon_checked',false,'validation_scope','recovery_executed_interval', ...
        'full_horizon_failure_active',true,'recovery_active',true,'terminal_certified',false,'algorithm_revision',r.algorithm_revision);
    saved.goal=goal;saved.observed=o;saved.observation_time=time;setappdata(0,'ch5ReducedExecution',saved);return;
end
if ~refresh
    shift=round((time-saved.trajectory_time)/r.c.Ts);old=saved.trajectory;
    K=ch5ReducedFeedbackGain(x,r,k);u=old.U(:,shift+1)-K*(x-old.X(:,shift+1));phases.warm_start=toc(timer);timer=tic;
    [check,trajectory]=ch5ReducedHoldAuditV77(x,previous,u,o,r);phases.incumbent_check=toc(timer);selectionSource='accepted_feedback_interval';
else
    ch5ForceTerminalV62();
    if ~isempty(saved)&&~isempty(saved.trajectory),[U,qs]=ch5ForceWarmV57(x,previous,saved,r,k,time);
    else,brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));qs=[];end
    phases.warm_start=toc(timer);timer=tic;
    nodes=full(k.nodes(x,U));if isempty(qs),qs=nodes(1:6,end);end
    incumbent=struct('X',nodes,'U',U,'artificial_pose',qs);trajectory=incumbent;
    check=struct('accepted',false,'scope','unaudited_numerical_initializer');phases.incumbent_check=toc(timer);
    generator=getappdata(0,'ch5Random');noise=randn(generator,360,4);context=ch5Context(x,previous,goal,o,r.mode);
    rawOnly=isfield(r.reduced,'raw_only')&&r.reduced.raw_only;warm=[];
    for batchStart=[1,3]
        timer=tic;ids=batchStart:batchStart+1;
        [batch,~,sampleInfo]=ch5ForceSampleV76(context,previous,r.force_weights,noise(:,ids),r,k);
        phases.prior=phases.prior+toc(timer);priorInfo{end+1}=sampleInfo;rawGenerated(:,:,ids)=batch;
        timer=tic;batchRanking=rankPriors(x,batch,goal,r,k);ranking(ids)=batchRanking;
        if ~rawOnly&&isempty(prepared)
            prepared=ch5ForcePrepareStepV77(x,previous,incumbent.U,goal,o,rawGenerated(:,:,1),r,k,time,incumbent.artificial_pose);
        end
        phases.prediction_step=phases.prediction_step+toc(timer);[~,order]=sort(batchRanking);
        for bi=order
            b=ids(bi);
            if rawOnly
                timer=tic;[rawCheck,rawTrajectory,rawCost]=ch5ForceAuditV77(x,previous,rawGenerated(:,:,b),goal,o,r,k,[]);
                phases.trial_checks=phases.trial_checks+toc(timer);rawChecks{b}=rawCheck;
                if rawCheck.accepted,check=rawCheck;trajectory=rawTrajectory;cost=rawCost;selectionSource='raw_prior';chosenCandidate=b;break;end
                continue;
            end
            timer=tic;[proposal,pose,attempt,warm]=ch5ForceSolveStepV77(prepared,rawGenerated(:,:,b),warm);
            phases.prediction_step=phases.prediction_step+toc(timer);attempt.candidate_index=b;attempt.retained=false;attempt.trials={};
            timer=tic;
            if attempt.finite
                for line=0:3
                    a=2^(-line);trial=incumbent.U+a*(proposal-incumbent.U);trialPose=incumbent.artificial_pose+a*(pose-incumbent.artificial_pose);
                    [~,trialCheck,trialTrajectory,trialCost,retraction]=ch5ForceRetractV77(x,previous,trial,goal,o,r,k,trialPose);
                    attempt.trials{end+1}=struct('line',line,'check',trialCheck,'cost',trialCost,'terminal_retraction',retraction);
                    if trialCheck.accepted
                        acceptedProjected=acceptedProjected+1;check=trialCheck;trajectory=trialTrajectory;cost=trialCost;
                        attempt.retained=true;selectionSource='qp_projection';if retraction.attempted,selectionSource='qp_projection_and_terminal_retraction';end
                        chosenCandidate=b;break;
                    end
                end
            end
            phases.trial_checks=phases.trial_checks+toc(timer);attempts{end+1}=attempt;
            if check.accepted,break;end
        end
        if check.accepted,break;end
    end
    if ~check.accepted&&~rawOnly
        timer=tic;[~,order]=sort(ranking);
        for b=order
            [rawCheck,rawTrajectory,rawCost]=ch5ForceAuditV77(x,previous,rawGenerated(:,:,b),goal,o,r,k,[]);rawChecks{b}=rawCheck;
            if rawCheck.accepted,check=rawCheck;trajectory=rawTrajectory;cost=rawCost;selectionSource='raw_prior';chosenCandidate=b;break;end
        end
        phases.trial_checks=phases.trial_checks+toc(timer);
    end
    if ~check.accepted&&~rawOnly
        timer=tic;[ic,it,icost]=ch5ForceAuditV77(x,previous,incumbent.U,goal,o,r,k,incumbent.artificial_pose);
        if ~ic.accepted
            [restored,~]=ch5ReducedTerminalV23(x,incumbent.U,r,k,incumbent.X);
            [ic,it,icost]=ch5ForceAuditV77(x,previous,restored,goal,o,r,k,[]);
        end
        phases.incumbent_check=phases.incumbent_check+toc(timer);
        if ic.accepted,check=ic;trajectory=it;cost=icost;selectionSource='incumbent_restoration';end
    end
    % Diagnostic work follows the completed decision. The flag cannot select a prior.
    if isfield(r.reduced,'audit_raw_candidates')&&r.reduced.audit_raw_candidates
        timer=tic;
        for b=1:size(rawGenerated,3)
            if b<=numel(rawChecks)&&~isempty(rawChecks{b}),continue;end
            rawChecks{b}=ch5ForceAuditV77(x,previous,rawGenerated(:,:,b),goal,o,r,k,[]);
        end
        phases.trial_checks=phases.trial_checks+toc(timer);
    end
    for b=1:numel(rawChecks)
        if ~isempty(rawChecks{b}),rawAudited=rawAudited+1;acceptedRaw=acceptedRaw+double(rawChecks{b}.accepted);end
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
    timer=tic;[u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);phases.recovery=toc(timer);selectionSource='safety_qp_recovery';
    if isempty(saved),saved=struct('trajectory',[],'trajectory_time',time,'refresh_time',time,'anchor',anchor);else,saved.trajectory=[];end
end
if refresh,saved.refresh_time=time;saved.numerical_time=time;saved.numerical_trajectory=trajectory;end
saved.goal=goal;saved.observed=o;saved.observation_time=time;setappdata(0,'ch5ReducedExecution',saved);
prediction=struct('refresh',refresh,'chosen',double(check.accepted),'chosen_candidate',chosenCandidate,'selection_source',selectionSource, ...
    'candidates',{{trajectory}},'checks',{{check}},'raw_audited',rawAudited,'raw_checks',{rawChecks},'raw_accepted',acceptedRaw, ...
    'raw_generated',rawGenerated,'raw_noise',noise,'raw_context',context,'raw_state',x,'raw_previous',previous, ...
    'raw_generated_count',size(rawGenerated,3),'ranking',ranking,'projected_accepted',acceptedProjected, ...
    'denoise_attempts',{attempts},'final_attempts',{{}},'restoration_attempts',{{}},'terminal_repair',[],'prior',{priorInfo}, ...
    'goal',goal,'observed',o,'cost',cost,'phases',phases,'full_horizon_checked',refresh,'validation_scope',check.scope, ...
    'full_horizon_failure_active',~check.accepted,'recovery_active',logical(d.usedFallback),'terminal_certified',false,'algorithm_revision',r.algorithm_revision);
end
function cost=rankPriors(x,U,goal,r,k)
% Dynamics-only ranking is not an admission check or a substitute for full gates.
ps=[.1*ones(3,1);deg2rad(10)*ones(3,1)];vs=[.1*ones(3,1);.2*ones(3,1)];cost=inf(1,size(U,3));
for b=1:size(U,3)
    if ~all(isfinite(U(:,:,b)),'all'),continue;end
    nodes=full(k.nodes(x,U(:,:,b)));
    if all(isfinite(nodes),'all')
        cost(b)=r.c.Ts*(sum(((nodes(1:6,2:end)-goal)./ps).^2,'all')+sum((nodes(7:12,2:end)./vs).^2,'all'))+ ...
            100*sum(((nodes(1:6,end)-goal)./ps).^2);
    end
end
end
