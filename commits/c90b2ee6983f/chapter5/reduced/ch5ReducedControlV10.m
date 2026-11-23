function [u,d,prediction]=ch5ReducedControlV10(x,previous,goal,o,r,time)
% 10-Hz full prediction admission; 50-Hz execution checks one physical hold.
phase=tic;k=ch5ReducedKernelsV4(r);saved=[];
if isappdata(0,'ch5ReducedExecution'),saved=getappdata(0,'ch5ReducedExecution');end
refresh=isempty(saved)||time-saved.refresh_time>=.1-1e-10||norm(goal-saved.goal)>1e-10||any(o(22,:)~=saved.observed(22,:));
anchor=x(1:6);if ~isempty(saved),anchor=saved.anchor;end
phases=struct('warm_start',0,'prior',0,'incumbent_check',0,'prediction_step',0,'trial_checks',0,'recovery',0);
attempts={};repair=[];priorInfo=[];cost=nan;duplicateAudits=0;
if ~refresh
    executable=saved.trajectory;trajectory=[];
    check=struct('accepted',false,'finite',true,'scope','executed_interval', ...
        'terminal_checked',false,'equality_tolerance_ratio',0,'rejection_reason','no_retained_plan');
    if ~isempty(executable)
        shift=round((time-saved.trajectory_time)/r.c.Ts);
        assert(shift>=1&&shift<r.c.H,'ch5:RetainedPlanAge','Retained plan is outside its execution horizon.');
        gain=ch5ReducedFeedbackGain(x,r,k);
        proposed=executable.U(:,shift+1)-gain*(x-executable.X(:,shift+1));
        phases.warm_start=toc(phase);phase=tic;
        [check,trajectory]=ch5ReducedHoldAudit(x,previous,proposed,o,r);
        phases.incumbent_check=toc(phase);
    else,phases.warm_start=toc(phase);end
    if forcedRejection(r,time),check.accepted=false;check.rejection_reason='development_fault_injection';end
    if check.accepted
        u=proposed;d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);anchor=x(1:6);
    else
        phase=tic;[u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);phases.recovery=toc(phase);executable=[];
    end
    saved.time=time;saved.observed=o;saved.goal=goal;saved.anchor=anchor;saved.trajectory=executable;
    setappdata(0,'ch5ReducedExecution',saved);
else
    U=[];
    if ~isempty(saved)&&~isempty(saved.trajectory)
        shift=round((time-saved.trajectory_time)/r.c.Ts);
        if shift>=1&&shift<=r.c.H
            old=saved.trajectory;U=[old.U(:,shift+1:end),repmat(old.artificial_force,1,shift)];
            steady=[old.artificial_pose;zeros(6,1)];
            if r.plant.n>12,steady=[steady;repmat(old.artificial_force,(r.plant.n-12)/6,1)];end
            warmNodes=[old.X(:,shift+1:end),repmat(steady,1,shift)];
            feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(x,r,k);
            U=full(feedback(x,warmNodes,U,gain,zeros(6,r.c.H/5)));
        end
    end
    if isempty(U),brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));end
    phases.warm_start=toc(phase);phase=tic;
    [prior,priorInfo]=ch5ReducedPrior(x,previous,goal,o,r,k);phases.prior=toc(phase);phase=tic;
    [check,trajectory,cost]=ch5ReducedAuditGuardedV10(x,previous,U,goal,o,r,k,prior);
    if check.equality_tolerance_ratio>1
        [U,repair]=ch5ReducedTerminal(x,U,r,k);
        [check,trajectory,cost]=ch5ReducedAuditGuardedV10(x,previous,U,goal,o,r,k,prior);
    end
    phases.incumbent_check=toc(phase);
    for iteration=1:r.reduced.iterations
        phase=tic;[proposed,info]=ch5ReducedStepV9(x,previous,U,goal,o,prior,r,k);retained=false;
        phases.prediction_step=phases.prediction_step+toc(phase);phase=tic;
        lastTrial=[];lastCheck=[];lastTrajectory=[];lastCost=[];lastRepair=[];
        for line=0:r.reduced.backtracking
            trial=U+2^(-line)*(proposed-U);trialRepair=[];
            if isequal(trial,U)
                trialCheck=check;trialTrajectory=trajectory;trialCost=cost;duplicateAudits=duplicateAudits+1;
                break; % All shorter steps are the already checked incumbent.
            elseif isequal(trial,lastTrial)
                trialCheck=lastCheck;trialTrajectory=lastTrajectory;trialCost=lastCost;trialRepair=lastRepair;
                duplicateAudits=duplicateAudits+1;
            else
                [trialCheck,trialTrajectory,trialCost]=ch5ReducedAuditGuardedV10(x,previous,trial,goal,o,r,k,prior);
                if trialCheck.equality_tolerance_ratio>1
                    [trial,trialRepair]=ch5ReducedTerminal(x,trial,r,k);
                    [trialCheck,trialTrajectory,trialCost]=ch5ReducedAuditGuardedV10(x,previous,trial,goal,o,r,k,prior);
                end
                lastTrial=trial;lastCheck=trialCheck;lastTrajectory=trialTrajectory;lastCost=trialCost;lastRepair=trialRepair;
            end
            if trialCheck.accepted&&(~check.accepted||trialCost<cost)
                U=trial;check=trialCheck;trajectory=trialTrajectory;cost=trialCost;retained=true;break;
            end
        end
        phases.trial_checks=phases.trial_checks+toc(phase);
        info.retained=retained;info.proposal_check=trialCheck;info.terminal_repair=trialRepair;info.line=line;
        info.duplicate_audits_skipped=duplicateAudits;attempts{end+1}=info;
        if ~retained,break;end
    end
    if forcedRejection(r,time),check.accepted=false;check.rejection_reason='development_fault_injection';end
    if check.accepted
        u=U(:,1);d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);anchor=x(1:6);executable=trajectory;
    else
        phase=tic;[u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);phases.recovery=toc(phase);executable=[];
    end
    setappdata(0,'ch5ReducedExecution',struct('time',time,'trajectory_time',time,'refresh_time',time, ...
        'goal',goal,'observed',o,'trajectory',executable,'anchor',anchor));
end
prediction=struct('refresh',refresh,'chosen',double(check.accepted),'candidates',{{trajectory}},'checks',{{check}}, ...
    'denoise_attempts',{attempts},'final_attempts',{{}},'terminal_repair',repair,'prior',priorInfo,'goal',goal,'observed',o, ...
    'cost',cost,'phases',phases,'full_horizon_checked',refresh,'validation_scope',check.scope,'duplicate_audits_skipped',duplicateAudits);
end

function rejected=forcedRejection(r,time)
rejected=isfield(r.reduced,'force_rejection_window')&&time>=r.reduced.force_rejection_window(1)&&time<r.reduced.force_rejection_window(2);
end
