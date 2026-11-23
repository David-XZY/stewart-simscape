function [u,d,prediction]=ch5ReducedControlLatest(x,previous,goal,o,r,time)
k=ch5ReducedKernels(r);saved=[];if isappdata(0,'ch5ReducedExecution'),saved=getappdata(0,'ch5ReducedExecution');end
refresh=isempty(saved)||time-saved.refresh_time>=.1-1e-10||norm(goal-saved.goal)>1e-10||any(o(22,:)~=saved.observed(22,:));
anchor=x(1:6);brake=ch5ReducedBrakeSequence(r);U=[];prior=zeros(size(U));priorInfo=[];attempts={};repair=[];
if ~isempty(saved)
    anchor=saved.anchor;shift=round((time-saved.time)/r.c.Ts);
    if ~isempty(saved.trajectory)&&shift>=1&&shift<=r.c.H
        U=[saved.trajectory.U(:,shift+1:end),repmat(saved.trajectory.artificial_force,1,shift)];
        steady=[saved.trajectory.artificial_pose;zeros(6,1)];
        if r.plant.n>12,steady=[steady;repmat(saved.trajectory.artificial_force,(r.plant.n-12)/6,1)];end
        warmNodes=[saved.trajectory.X(:,shift+1:end),repmat(steady,1,shift)];
        feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(x,r,k);
        U=full(feedback(x,warmNodes,U,gain,zeros(6,r.c.H/5)));
    else
        U=full(brake(x,previous));
    end
end
if isempty(U),U=full(brake(x,previous));end
if refresh,[prior,priorInfo]=ch5ReducedPrior(x,previous,goal,o,r,k);end
[check,trajectory,cost]=ch5ReducedAudit(x,previous,U,goal,o,r,k,prior);
if check.equality_tolerance_ratio>1
    [U,repair]=ch5ReducedTerminal(x,U,r,k);
    [check,trajectory,cost]=ch5ReducedAudit(x,previous,U,goal,o,r,k,prior);
end
if refresh
    for iteration=1:r.reduced.iterations
        [proposed,info]=ch5ReducedStepLatest(x,previous,U,goal,o,prior,r,k);retained=false;
        for line=0:r.reduced.backtracking
            trial=U+2^(-line)*(proposed-U);trialRepair=[];
            [trialCheck,trialTrajectory,trialCost]=ch5ReducedAudit(x,previous,trial,goal,o,r,k,prior);
            if trialCheck.equality_tolerance_ratio>1
                [trial,trialRepair]=ch5ReducedTerminal(x,trial,r,k);
                [trialCheck,trialTrajectory,trialCost]=ch5ReducedAudit(x,previous,trial,goal,o,r,k,prior);
            end
            if trialCheck.accepted&&(~check.accepted||trialCost<cost)
                U=trial;check=trialCheck;trajectory=trialTrajectory;cost=trialCost;retained=true;break;
            end
        end
        info.retained=retained;info.proposal_check=trialCheck;info.terminal_repair=trialRepair;info.line=line;attempts{end+1}=info;
        if ~retained,break;end
    end
end
if isfield(r.reduced,'force_rejection_window')&&time>=r.reduced.force_rejection_window(1)&&time<r.reduced.force_rejection_window(2)
    check.accepted=false;check.rejection_reason='development_fault_injection';
end
if check.accepted
    u=U(:,1);d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);anchor=x(1:6);
    executable=trajectory;
else
    [u,d]=ch5ReducedRecovery(x,previous,o,r,k,time,anchor);executable=[];
end
refreshTime=time;if ~refresh,refreshTime=saved.refresh_time;end
setappdata(0,'ch5ReducedExecution',struct('time',time,'refresh_time',refreshTime,'goal',goal,'observed',o,'trajectory',executable,'anchor',anchor));
prediction=struct('refresh',refresh,'chosen',double(check.accepted),'candidates',{{trajectory}},'checks',{{check}}, ...
    'denoise_attempts',{attempts},'final_attempts',{{}},'terminal_repair',repair,'prior',priorInfo,'goal',goal,'observed',o,'cost',cost);
end
