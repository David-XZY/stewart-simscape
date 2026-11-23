function [u,d,prediction]=ch5ForceControlV60(x,previous,goal,o,r,time)
% Four v-force candidates share one physical linearization. Admission uses exact rollouts.
k=ch5ReducedKernelsV4(r);saved=[];if isappdata(0,'ch5ReducedExecution'),saved=getappdata(0,'ch5ReducedExecution');end
refresh=isempty(saved)||isempty(saved.trajectory)||time-saved.refresh_time>=.1-1e-10|| ...
    norm(goal-saved.goal)>1e-10||any(o(22,:)~=saved.observed(22,:));
attempts={};rawChecks={};checks={};trajectory=[];cost=inf;priorInfo=[];acceptedRaw=0;acceptedProjected=0;
anchor=x(1:6);if ~isempty(saved),anchor=saved.anchor;end
phases=struct('warm_start',0,'prior',0,'incumbent_check',0,'prediction_step',0,'trial_checks',0,'recovery',0);timer=tic;
if ~refresh
    shift=round((time-saved.trajectory_time)/r.c.Ts);old=saved.trajectory;
    K=ch5ReducedFeedbackGain(x,r,k);u=old.U(:,shift+1)-K*(x-old.X(:,shift+1));phases.warm_start=toc(timer);timer=tic;
    [check,trajectory]=ch5ReducedHoldAuditV22(x,previous,u,o,r);phases.incumbent_check=toc(timer);
else
    if ~isempty(saved)&&~isempty(saved.trajectory)
        [U,qs]=ch5ForceWarmV57(x,previous,saved,r,k,time);
    else
        brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));qs=[];
    end
    phases.warm_start=toc(timer);timer=tic;
    [check,trajectory,cost]=ch5ForceAuditV58(x,previous,U,goal,o,r,k,qs);phases.incumbent_check=toc(timer);
    if ~check.accepted
        % Numerical terminal initializer, followed by the NEW gate; it is never an executed reference.
        [U,repair]=ch5ReducedTerminalV23(x,U,r,k,trajectory.X); %#ok<ASGLU>
        [check,trajectory,cost]=ch5ForceAuditV58(x,previous,U,goal,o,r,k,[]);
    end
    incumbent=trajectory;incumbentCheck=check;incumbentCost=cost;timer=tic;
    generator=getappdata(0,'ch5Random');noise=randn(generator,360,4);
    [prior,~,priorInfo]=ch5ForceSampleV57(ch5Context(x,previous,goal,o,r.mode),previous,r.force_weights,noise);phases.prior=toc(timer);
    timer=tic;
    % Raw generation is recorded before input clipping or any physical projection.
    for b=1:4
        [rawCheck,rawTrajectory,rawCost]=ch5ForceAuditV58(x,previous,prior(:,:,b),goal,o,r,k,[]);
        rawChecks{b}=rawCheck;acceptedRaw=acceptedRaw+double(rawCheck.accepted);
        if rawCheck.accepted&&(~check.accepted||rawCost<cost),check=rawCheck;trajectory=rawTrajectory;cost=rawCost;end
    end
    phases.trial_checks=toc(timer);timer=tic;
    [proposals,poses,solves]=ch5ForceStepV59(x,previous,incumbent.U,goal,o,prior,r,k,time,incumbent.artificial_pose);
    phases.prediction_step=toc(timer);timer=tic;
    for b=1:4
        attempt=solves{b};attempt.retained=false;attempt.trials={};
        if attempt.finite
            for line=0:3
                a=2^(-line);trial=incumbent.U+a*(proposals(:,:,b)-incumbent.U);qs=incumbent.artificial_pose+a*(poses(:,b)-incumbent.artificial_pose);
                [trial,trialCheck,trialTrajectory,trialCost,retraction]=ch5ForceRetractV60(x,previous,trial,goal,o,r,k,qs);
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
    if ~check.accepted&&incumbentCheck.accepted,check=incumbentCheck;trajectory=incumbent;cost=incumbentCost;end
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
saved.goal=goal;saved.observed=o;setappdata(0,'ch5ReducedExecution',saved);
prediction=struct('refresh',refresh,'chosen',double(check.accepted),'candidates',{{trajectory}},'checks',{{check}}, ...
    'raw_checks',{rawChecks},'raw_accepted',acceptedRaw,'projected_accepted',acceptedProjected,'denoise_attempts',{attempts}, ...
    'final_attempts',{{}},'restoration_attempts',{{}},'terminal_repair',[],'prior',priorInfo,'goal',goal,'observed',o, ...
    'cost',cost,'phases',phases,'full_horizon_checked',refresh,'validation_scope',check.scope,'full_horizon_failure_active',~check.accepted, ...
    'recovery_active',logical(d.usedFallback),'terminal_certified',false,'algorithm_revision','force-v60-development');
end
