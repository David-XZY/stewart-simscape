function [u,diagnostic,prediction]=ch5SpeedDiffusionControl(x,previous,goal,o,r,time)
% Development-only test: denoise once, then a bounded number of joint solves.
% Force increments remain the decision, and the original nonlinear physical
% rollout/terminal gate is mandatory before any newly optimized input executes.
timer=tic;w=r.weights;H=r.c.H;generator=getappdata(0,'ch5Random');p=r.speed;
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
noisy=randn(generator,6*H,r.c.candidates);
steps=unique(round(linspace(r.c.diffusion_steps-1,0,r.c.ddim_steps)),'stable');
for step=1:numel(steps)
    t=steps(step);alpha=double(w.alpha_bar(t+1));noise=ch5Denoise(noisy,condition,t,w);
    clean=(noisy-sqrt(1-alpha)*noise)/sqrt(alpha);
    if step<numel(steps)
        next=double(w.alpha_bar(steps(step+1)+1));noisy=sqrt(next)*clean+sqrt(1-next)*noise;
    end
end
neuralSeconds=toc(timer);timer=tic;
preview=ch5NativePreview(r);nodesPreview=ch5NativeNodes(r);
r.optimizer=ch5PredictiveOptimizer(r,find(o(22,:)>.5));opt=r.optimizer;
warm=[];
if isappdata(0,'ch5SpeedWarm')
    saved=getappdata(0,'ch5SpeedWarm');age=(time-saved.time)/r.c.Ts;shift=round(age);
    if saved.mode==r.mode&&shift>=1&&shift<=H&&abs(age-shift)<1e-6
        warm=saved;warm.shift_steps=shift;
    end
end
[U,nodes,initial]=ch5ControlInitialGuess(x,previous,warm,r,nodesPreview);
[drift,G]=r.ad.dynamics(nodes(1:6,end),zeros(6,1));ua=full(G)\(-full(drift));
state=[reshape(diff([previous,U],1,2),[],1);U(:);reshape(nodes(opt.shooting_rows,2:end),[],1);nodes(1:6,end);ua];
% Rank learned proposals through the same dynamics before joint correction.
scores=inf(1,r.c.candidates);priors=cell(1,r.c.candidates);
for b=1:r.c.candidates
    prior=reshape(clean(:,b).*double(w.action_std(:))+double(w.action_mean(:)),6,H);
    proposal=previous+cumsum(prior,2);last=previous;
    for j=1:H
        proposal(:,j)=min(max(proposal(:,j),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)),min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));
        last=proposal(:,j);
    end
    priors{b}=diff([previous,proposal],1,2);
    predicted=full(nodesPreview(x,proposal));
    if all(isfinite(predicted),'all')
        scale=[.1;.1;.1;deg2rad(10)*ones(3,1)];
        scores(b)=sum(((predicted(1:6,2:end)-goal)./scale).^2,'all')+.1*sum(predicted(7:12,2:end).^2,'all');
    end
end
[~,selected]=min(scores);prior=priors{selected};parameter=[x;previous;goal;o(:);prior(:)];
[state,check,trajectory,cost]=ch5InspectCandidate(state,parameter,x,previous,o,r,opt,preview);
backup=[];backupCheck=[];backupTrajectory=[];backupCost=inf;
if check.accepted,backup=state;backupCheck=check;backupTrajectory=trajectory;backupCost=cost;end
setupSeconds=toc(timer);attempts=cell(p.corrections,1);lambda=[];
for iteration=1:p.corrections
    timer=tic;solution=ch5SolveProjection(opt,state,parameter,lambda);
    [proposed,proposedCheck,proposedTrajectory,proposedCost]=ch5InspectCandidate(solution.x,parameter,x,previous,o,r,opt,preview);
    penalty=max(r.c.projection_merit_penalty_floor,1.1*norm(solution.lam_g,inf));
    oldMerit=cost+penalty*check.merit_violation;
    replace=proposedCheck.finite&&proposedCost+penalty*proposedCheck.merit_violation<oldMerit;
    for line=1:p.backtracking_steps
        if replace,break;end
        trial=state+2^(-line)*(solution.x-state);
        [proposed,proposedCheck,proposedTrajectory,proposedCost]=ch5InspectCandidate(trial,parameter,x,previous,o,r,opt,preview);
        replace=proposedCheck.finite&&proposedCost+penalty*proposedCheck.merit_violation<oldMerit;
    end
    if replace,state=proposed;check=proposedCheck;trajectory=proposedTrajectory;cost=proposedCost;lambda=solution.lam_g;else,lambda=[];end
    if check.accepted&&cost<backupCost
        backup=state;backupCheck=check;backupTrajectory=trajectory;backupCost=cost;
    end
    attempts{iteration}=struct('status',solution.stats.return_status,'solver_success',solution.stats.success, ...
        'seconds',toc(timer),'linearization_seconds',solution.linearization_seconds,'solver_seconds',solution.solver_seconds, ...
        'proposal_accepted',proposedCheck.accepted,'retained_incumbent',~replace);
end
finalAttempts={};
if p.polish_iterations>0
    finalOpt=ch5PredictiveOptimizer(r,find(o(22,:)>.5),true);timer=tic;
    solution=ch5SolveProjection(finalOpt,state,parameter,lambda);
    [proposed,proposedCheck,proposedTrajectory,proposedCost]=ch5InspectCandidate(solution.x,parameter,x,previous,o,r,opt,preview);
    if proposedCheck.accepted&&proposedCost<backupCost
        backup=proposed;backupCheck=proposedCheck;backupTrajectory=proposedTrajectory;backupCost=proposedCost;
    end
    finalAttempts={struct('status',solution.stats.return_status,'solver_success',solution.stats.success,'seconds',toc(timer),'solver_seconds',solution.solver_seconds)};
end
if ~isempty(backup)
    u=backupTrajectory.U(:,1);diagnostic=struct('feasible',true,'usedFallback',false,'exitflag',1,'chosen',1,'solver_failure',false);
    setappdata(0,'ch5SpeedWarm',struct('time',time,'mode',r.mode,'U',backupTrajectory.U,'artificial_force',backupTrajectory.artificial_force));
    prediction=struct('chosen',1,'candidates',{{backupTrajectory}},'checks',{{backupCheck}}, ...
        'denoise_attempts',{attempts},'final_attempts',{finalAttempts});
else
    [u,diagnostic]=ch5SafetyQp(x(1:6),x(7:12),x(1:6),zeros(6,1),zeros(6,1),previous,previous,o,r,time);
    diagnostic.usedFallback=true;diagnostic.fallbackSafetyCertified=false;
    prediction=struct('chosen',0,'candidates',{{trajectory}},'checks',{{check}},'denoise_attempts',{attempts},'final_attempts',{finalAttempts});
end
prediction.goal=goal;prediction.observed=o;prediction.initial_guess=initial;prediction.prior_index=selected;
prediction.phase_seconds=struct('neural',neuralSeconds,'candidate_initialization',setupSeconds);
end
