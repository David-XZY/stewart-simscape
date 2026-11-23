function [u,d,prediction]=ch5ReducedControlV45(x,previous,goal,o,r,time)
% True-rollout admission with selective sensitivities and bounded QP solve.
phase=tic;k=ch5ReducedKernelsV45(r);saved=[];
if isappdata(0,'ch5ReducedExecution'),saved=getappdata(0,'ch5ReducedExecution');end
retainWarm=false;restoreCount=0; % Rejected warm-seed variants remain disabled.
if ~isempty(saved)&&~isfield(saved,'numerical')
    saved.numerical=saved.trajectory;saved.numerical_time=saved.trajectory_time;saved.full_horizon_failure_active=false;
end
refresh=isempty(saved)||time-saved.refresh_time>=.1-1e-10||norm(goal-saved.goal)>1e-10||any(o(22,:)~=saved.observed(22,:));
anchor=x(1:6);if ~isempty(saved),anchor=saved.anchor;end
phases=struct('warm_start',0,'prior',0,'incumbent_check',0,'prediction_step',0,'trial_checks',0,'restoration',0,'recovery',0);
attempts={};restoration={};repair=[];priorInfo=[];cost=nan;screened=0;fullAudits=0;duplicateAudits=0;usedNumerical=false;
if ~refresh
    executable=saved.trajectory;trajectory=[];numerical=saved.numerical;numericalTime=saved.numerical_time;
    check=struct('accepted',false,'finite',true,'scope','executed_interval','terminal_checked',false, ...
        'equality_tolerance_ratio',0,'rejection_reason','no_retained_plan');
    if ~isempty(executable)
        [executable,repair]=ch5ReducedWarmV23(x,previous,saved,r,k,time);
        numerical=executable;numericalTime=time;proposed=executable.U(:,1);
        phases.warm_start=toc(phase);phase=tic;
        [check,trajectory]=ch5ReducedHoldAuditV22(x,previous,proposed,o,r);phases.incumbent_check=toc(phase);
    else,phases.warm_start=toc(phase);end
    if forcedRejection(r,time),check.accepted=false;check.rejection_reason='development_fault_injection';end
    if check.accepted
        u=proposed;d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);anchor=x(1:6);
    else
        phase=tic;[u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);phases.recovery=toc(phase);executable=[];
        if ~retainWarm,numerical=[];end
    end
    saved.time=time;saved.trajectory_time=time;saved.observed=o;saved.goal=goal;saved.anchor=anchor;
    saved.trajectory=executable;saved.numerical=numerical;saved.numerical_time=numericalTime;
else
    U=[];
    if ~isempty(saved)
        old=saved.trajectory;origin=saved.trajectory_time;
        if isempty(old)&&retainWarm,old=saved.numerical;origin=saved.numerical_time;usedNumerical=~isempty(old);end
        shift=round((time-origin)/r.c.Ts);
        if ~isempty(old)&&shift>=1&&shift<=r.c.H
            U=[old.U(:,shift+1:end),repmat(old.artificial_force,1,shift)];
            steady=[old.artificial_pose;zeros(6,1)];if r.plant.n>12,steady=[steady;repmat(old.artificial_force,(r.plant.n-12)/6,1)];end
            warmNodes=[old.X(:,shift+1:end),repmat(steady,1,shift)];
            feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(x,r,k);
            U=full(feedback(x,warmNodes,U,gain,zeros(6,r.c.H/5)));
        end
    end
    if isempty(U),brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));end
    phases.warm_start=toc(phase);phase=tic;
    [prior,priorInfo]=ch5ReducedPriorV43(x,previous,goal,o,r,k);phases.prior=toc(phase);phase=tic;
    [U,check,trajectory,cost,auditInfo]=ch5ReducedAuditV23(x,previous,U,goal,o,r,k,prior);
    repair=auditInfo.terminal_repair;fullAudits=fullAudits+auditInfo.full_audit_performed;
    phases.incumbent_check=toc(phase);best=store(U,check,trajectory,cost);
    for iteration=1:r.reduced.iterations
        phase=tic;[proposed,info]=ch5ReducedStepV33(x,previous,U,goal,o,prior,r,k,time);
        phases.prediction_step=phases.prediction_step+toc(phase);phase=tic;
        [U,check,trajectory,cost,best,search]=searchTrial(x,previous,U,proposed,goal,o,prior,r,k,check,trajectory,cost,best);
        phases.trial_checks=phases.trial_checks+toc(phase);
        info.retained=search.retained;info.proposal_check=search.proposal_check;info.terminal_repair=search.terminal_repair;
        info.line=search.line;info.trials=search.trials;attempts{end+1}=info;
        screened=screened+search.screened;fullAudits=fullAudits+search.full_audits;duplicateAudits=duplicateAudits+search.duplicates;
        if ~search.retained,break;end
    end
    taskWarm=best; % A failed feasibility search must not replace this task iterate.
    if ~check.accepted&&~forcedRejection(r,time)
        for iteration=1:restoreCount
            if isempty(best),break;end
            phase=tic;[proposed,info]=ch5ReducedRestoreV12(x,previous,best.U,goal,o,prior,r,k);
            [candidateU,candidateCheck,candidateTrajectory,candidateCost,best,search]=searchTrial( ...
                x,previous,best.U,proposed,goal,o,prior,r,k,best.check,best.trajectory,best.cost,best);
            info.retained=search.retained;info.trials=search.trials;info.proposal_check=search.proposal_check;
            restoration{end+1}=info;phases.restoration=phases.restoration+toc(phase);
            screened=screened+search.screened;fullAudits=fullAudits+search.full_audits;duplicateAudits=duplicateAudits+search.duplicates;
            if search.retained,U=candidateU;check=candidateCheck;trajectory=candidateTrajectory;cost=candidateCost;break;end
        end
        if ~check.accepted&&~isempty(best),U=best.U;check=best.check;trajectory=best.trajectory;cost=best.cost;end
    end
    if forcedRejection(r,time),check.accepted=false;check.rejection_reason='development_fault_injection';end
    if check.accepted
        u=U(:,1);d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);anchor=x(1:6);
        executable=trajectory;numerical=trajectory;
    else
        phase=tic;[u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);phases.recovery=toc(phase);executable=[];numerical=[];
        if retainWarm&&~isempty(taskWarm),numerical=taskWarm.trajectory;end
    end
    saved=struct('time',time,'trajectory_time',time,'refresh_time',time,'goal',goal,'observed',o, ...
        'trajectory',executable,'numerical',numerical,'numerical_time',time,'anchor',anchor,'full_horizon_failure_active',~check.accepted);
end
if ~check.accepted&&isappdata(0,'ch5ReducedSensitivity'),rmappdata(0,'ch5ReducedSensitivity');end
setappdata(0,'ch5ReducedExecution',saved);
prediction=struct('refresh',refresh,'chosen',double(check.accepted),'candidates',{{trajectory}},'checks',{{check}}, ...
    'denoise_attempts',{attempts},'final_attempts',{{}},'restoration_attempts',{restoration},'terminal_repair',repair, ...
    'prior',priorInfo,'goal',goal,'observed',o,'cost',cost,'phases',phases,'full_horizon_checked',refresh, ...
    'validation_scope',check.scope,'duplicate_audits_skipped',duplicateAudits,'objective_screens',screened, ...
    'full_candidate_audits',fullAudits,'numerical_retained',~isempty(saved.numerical),'used_retained_numerical',usedNumerical, ...
    'full_horizon_failure_active',saved.full_horizon_failure_active,'recovery_active',logical(d.usedFallback));
end

function [U,check,trajectory,cost,best,info]=searchTrial(x,previous,U,proposed,goal,o,prior,r,k,check,trajectory,cost,best)
info=struct('retained',false,'proposal_check',check,'terminal_repair',[],'line',0,'trials',{{}},'screened',0,'full_audits',0,'duplicates',0);
for line=0:r.reduced.backtracking
    trial=U+2^(-line)*(proposed-U);info.line=line;
    if isequal(trial,U),info.duplicates=info.duplicates+1;break;end
    incumbent=struct('accepted',check.accepted,'cost',cost);
    [trial,trialCheck,trialTrajectory,trialCost,audit]=ch5ReducedAuditV23(x,previous,trial,goal,o,r,k,prior,incumbent);
    info.screened=info.screened+audit.screened_for_cost;info.full_audits=info.full_audits+audit.full_audit_performed;
    info.proposal_check=trialCheck;info.terminal_repair=audit.terminal_repair;
    info.trials{end+1}=struct('line',line,'U',trial,'cost',trialCost,'check',trialCheck,'audit',audit);
    candidate=store(trial,trialCheck,trialTrajectory,trialCost);
    if ~isempty(candidate)&&(isempty(best)||candidate.score<best.score),best=candidate;end
    if trialCheck.accepted&&(~check.accepted||trialCost<cost)
        U=trial;check=trialCheck;trajectory=trialTrajectory;cost=trialCost;info.retained=true;break;
    end
end
end
function best=store(U,check,trajectory,cost)
best=[];
if isempty(trajectory.X)||~all(isfinite([trajectory.X(:);U(:)]))|| ...
        (isfield(check,'skipped_for_cost')&&check.skipped_for_cost),return;end
ratio=max([check.equality_tolerance_ratio-1,check.inequality_violation/1e-6-1,check.command_rate_violation/1e-6-1,0]);
best=struct('U',U,'check',check,'trajectory',trajectory,'cost',cost,'score',ratio);
end
function value=option(r,name,default)
value=default;if isfield(r.reduced,name),value=r.reduced.(name);end
end
function rejected=forcedRejection(r,time)
rejected=isfield(r.reduced,'force_rejection_window')&&time>=r.reduced.force_rejection_window(1)&&time<r.reduced.force_rejection_window(2);
end
