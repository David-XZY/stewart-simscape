function [u,d,prediction]=ch5SpeedTailDispatch(x,previous,goal,o,r,time)
% Refresh a force sequence at 10 Hz and execute its successive 50 Hz inputs.
% Every intermediate application checks the full updated nonlinear rollout;
% failed checks use the existing explicitly uncertified safety-QP fallback.
saved=[];if isappdata(0,'ch5SpeedExecution'),saved=getappdata(0,'ch5SpeedExecution');end
refresh=isempty(saved)||time<=saved.time||norm(goal-saved.goal)>1e-10|| ...
    any(o(22,:)~=saved.observed(22,:))||time-saved.time>=r.speed.update_seconds-1e-10;
if refresh
    [u,d,prediction]=ch5SpeedDiffusionControl(x,previous,goal,o,r,time);
    prediction.refresh=true;
    trajectory=[];if prediction.chosen>0,trajectory=prediction.candidates{prediction.chosen};end
    % Preserve the refresh clock on rejection too. Otherwise failures turn
    % a declared 10 Hz planner into repeated optimization at every 50 Hz tick.
    setappdata(0,'ch5SpeedExecution',struct('time',time,'goal',goal,'observed',o,'trajectory',trajectory));
else
    if isempty(saved.trajectory)
        [u,d]=ch5SafetyQp(x(1:6),x(7:12),x(1:6),zeros(6,1),zeros(6,1),previous,previous,o,r,time);
        d.usedFallback=true;d.fallbackSafetyCertified=false;
        prediction=struct('chosen',0,'candidates',{{}},'checks',{{struct('accepted',false,'rejection_reason','refresh_unavailable')}}, ...
            'denoise_attempts',{{}},'final_attempts',{{}},'refresh',false,'goal',goal,'observed',o);
        return;
    end
    shift=round((time-saved.time)/r.c.Ts);
    warm=struct('U',saved.trajectory.U,'artificial_force',saved.trajectory.artificial_force,'shift_steps',shift);
    [U,nodes]=ch5ControlInitialGuess(x,previous,warm,r,ch5NativeNodes(r));
    [drift,G]=r.ad.dynamics(nodes(1:6,end),zeros(6,1));ua=full(G)\(-full(drift));
    opt=ch5PredictiveOptimizer(r,find(o(22,:)>.5));D=diff([previous,U],1,2);
    decision=[D(:);U(:);reshape(nodes(opt.shooting_rows,2:end),[],1);nodes(1:6,end);ua];
    parameter=[x;previous;goal;o(:);D(:)];
    [~,check,trajectory]=ch5InspectCandidate(decision,parameter,x,previous,o,r,opt,ch5NativePreview(r));
    continuation=false;
    if ~check.accepted&&isfield(r.speed,'repair_held_terminal')&&r.speed.repair_held_terminal
        try
            backup=ch5SpeedTailBackup(r);brakeU=full(backup(x,U(:,1)));nodesFunction=ch5NativeNodes(r);brakeNodes=full(nodesFunction(x,brakeU));
            brakeD=diff([previous,brakeU],1,2);
            brakeDecision=[brakeD(:);brakeU(:);reshape(brakeNodes(opt.shooting_rows,2:end),[],1);brakeNodes(1:6,end);previous];
            [~,brakeCheck,brakeTrajectory]=ch5InspectCandidate(brakeDecision,parameter,x,previous,o,r,opt,ch5NativePreview(r));
            if brakeCheck.accepted,U=brakeU;check=brakeCheck;trajectory=brakeTrajectory;continuation=true;end
        catch ex
            check.continuation_error=ex.message;
        end
    end
    if check.accepted
        u=U(:,1);d=struct('feasible',true,'usedFallback',false,'exitflag',1,'chosen',1,'solver_failure',false);
    else
        [u,d]=ch5SafetyQp(x(1:6),x(7:12),x(1:6),zeros(6,1),zeros(6,1),previous,previous,o,r,time);
        d.usedFallback=true;d.fallbackSafetyCertified=false;
    end
    prediction=struct('chosen',double(check.accepted),'candidates',{{trajectory}},'checks',{{check}}, ...
        'denoise_attempts',{{}},'final_attempts',{{}},'refresh',false,'goal',goal,'observed',o,'repaired_continuation',continuation);
end
end
