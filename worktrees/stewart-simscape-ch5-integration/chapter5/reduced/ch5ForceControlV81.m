function [u,d,prediction]=ch5ForceControlV81(x,previous,goal,o,r,time)
% Generate two priors, rank their optimized task costs, audit on demand.
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
rawGenerated=[];noise=[];context=[];selectionSource='none';chosenCandidate=0;prepared=[];ranking=[];rawRanking=[];rawPrechecks={};
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
        [batch,~,sampleInfo]=ch5ForceSampleV81(context,previous,r.force_weights,noise(:,ids),r,k);
        phases.prior=phases.prior+toc(timer);priorInfo{end+1}=sampleInfo;rawGenerated(:,:,ids)=batch;
        timer=tic;batchRanking=inf(1,2);
        for bi=1:2
            [rawRanking(ids(bi)),rawPrechecks{ids(bi)}]=ch5ForceCandidateCostV80(x,previous,batch(:,:,bi),goal,r,k);
        end
        if ~rawOnly&&isempty(prepared)
            prepared=ch5ForcePrepareStepV81(x,previous,incumbent.U,goal,o,rawGenerated(:,:,1),r,k,time,incumbent.artificial_pose);
        end
        proposals=cell(1,2);poses=cell(1,2);solveGroup=cell(1,2);
        if ~rawOnly
            for bi=1:2
                [proposals{bi},poses{bi},attempt,warm]=ch5ForceSolveStepV77(prepared,batch(:,:,bi),warm);
                attempt.candidate_index=ids(bi);attempt.retained=false;attempt.trials={};
                if isfield(attempt,'predicted_task_cost'),batchRanking(bi)=attempt.predicted_task_cost;end
                solveGroup{bi}=attempt;
            end
        end
        ranking(ids)=batchRanking;phases.prediction_step=phases.prediction_step+toc(timer);
        [~,order]=sort(batchRanking);timer=tic;
        if ~rawOnly
            for bi=order
                attempt=solveGroup{bi};b=ids(bi);
                if attempt.finite
                    for line=0:3
                        a=2^(-line);trial=incumbent.U+a*(proposals{bi}-incumbent.U);trialPose=incumbent.artificial_pose+a*(poses{bi}-incumbent.artificial_pose);
                        [~,trialCheck,trialTrajectory,trialCost,retraction]=ch5ForceRetractV80(x,previous,trial,goal,o,r,k,trialPose);
                        attempt.trials{end+1}=struct('line',line,'check',trialCheck,'cost',trialCost,'terminal_retraction',retraction);
                        if trialCheck.accepted
                            acceptedProjected=acceptedProjected+1;check=trialCheck;trajectory=trialTrajectory;cost=trialCost;
                            attempt.retained=true;selectionSource='qp_projection';if retraction.attempted,selectionSource='qp_projection_and_terminal_retraction';end
                            chosenCandidate=b;break;
                        end
                    end
                end
                solveGroup{bi}=attempt;
                if check.accepted,break;end
            end
            for bi=1:2,attempts{end+1}=solveGroup{bi};end
        end
        [~,rawOrder]=sort(rawRanking(ids));
        for bi=rawOrder
            b=ids(bi);
            if ~rawPrechecks{b}.terminal_region_possible||rawRanking(b)>=cost,continue;end
            [rawCheck,rawTrajectory,rawCost]=ch5ForceAuditV80(x,previous,rawGenerated(:,:,b),goal,o,r,k,[]);rawChecks{b}=rawCheck;
            if rawCheck.accepted
                check=rawCheck;trajectory=rawTrajectory;cost=rawCost;selectionSource='raw_prior';chosenCandidate=b;break;
            end
        end
        phases.trial_checks=phases.trial_checks+toc(timer);
        if check.accepted,break;end
    end
    if ~check.accepted&&~rawOnly
        timer=tic;[ic,it,icost]=ch5ForceAuditV80(x,previous,incumbent.U,goal,o,r,k,incumbent.artificial_pose);
        if ~ic.accepted
            [restored,~]=ch5ReducedTerminalV23(x,incumbent.U,r,k,incumbent.X);
            [ic,it,icost]=ch5ForceAuditV80(x,previous,restored,goal,o,r,k,[]);
        end
        phases.incumbent_check=phases.incumbent_check+toc(timer);
        if ic.accepted,check=ic;trajectory=it;cost=icost;selectionSource='incumbent_restoration';end
    end
    % Diagnostic work follows the completed decision. The flag cannot select a prior.
    if isfield(r.reduced,'audit_raw_candidates')&&r.reduced.audit_raw_candidates
        timer=tic;
        for b=1:size(rawGenerated,3)
            if b<=numel(rawChecks)&&~isempty(rawChecks{b}),continue;end
            rawChecks{b}=ch5ForceAuditV80(x,previous,rawGenerated(:,:,b),goal,o,r,k,[]);
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
    'raw_generated_count',double(~isempty(rawGenerated))*size(rawGenerated,3),'ranking',ranking,'raw_cost_ranking',rawRanking,'raw_prechecks',{rawPrechecks},'projected_accepted',acceptedProjected, ...
    'denoise_attempts',{attempts},'final_attempts',{{}},'restoration_attempts',{{}},'terminal_repair',[],'prior',{priorInfo}, ...
    'goal',goal,'observed',o,'cost',cost,'phases',phases,'full_horizon_checked',refresh,'validation_scope',check.scope, ...
    'full_horizon_failure_active',~check.accepted,'recovery_active',logical(d.usedFallback),'terminal_certified',false,'algorithm_revision',r.algorithm_revision);
end
