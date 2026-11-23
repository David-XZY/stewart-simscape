function [u,diagnostic,prediction]=ch5DiffusionControl(x,previous,goal,o,r,time)
% The learned object is a force-increment distribution. A trajectory is
% obtained only by the shared dynamics; there is no LQI reference tracker.
w=r.weights;H=r.c.H;batch=r.c.candidates;generator=getappdata(0,'ch5Random');
preview=ch5NativePreview(r);
r.optimizer=ch5PredictiveOptimizer(r,find(o(22,:)>.5));
ns=r.optimizer.shooting_n;
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
noisy=randn(generator,6*H,batch);steps=unique(round(linspace(r.c.diffusion_steps-1,0,r.c.ddim_steps)),'stable');
states=cell(batch,1);attempts=cell(numel(steps),batch);lambda=cell(batch,1);candidate=zeros(6,H,batch);
incumbentChecks=cell(batch,1);incumbentTrajectories=cell(batch,1);lastLearnedPrior=candidate;
warm=[];
if isappdata(0,'ch5DiffusionWarm')
    saved=getappdata(0,'ch5DiffusionWarm');
    if saved.mode==r.mode&&time>saved.time&&time-saved.time<=r.c.Ts*1.01,warm=saved;end
end
for step=1:numel(steps)
    t=steps(step);alpha=double(w.alpha_bar(t+1));noise=ch5Denoise(noisy,condition,t,w);
    clean=(noisy-sqrt(1-alpha)*noise)/sqrt(alpha);
    for b=1:batch
        prior=reshape(clean(:,b).*double(w.action_std(:))+double(w.action_mean(:)),6,H);
        U=previous+cumsum(prior,2);
        for j=1:H
            if j==1,last=previous;else,last=U(:,j-1);end
            U(:,j)=min(max(U(:,j),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)),min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));
        end
        prior=diff([previous,U],1,2);parameter=[x;previous;goal;o(:);prior(:)];
        lastLearnedPrior(:,:,b)=prior;
        if isempty(states{b})
            U=repmat(previous,1,H);
            if ~isempty(warm)
                U=[warm.U(:,2:end),warm.artificial_force];
                for j=1:H
                    if j==1,last=previous;else,last=U(:,j-1);end
                    U(:,j)=min(max(U(:,j),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)),min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));
                end
            end
            nodes=full(preview(x,U));[drift,G]=r.ad.dynamics(nodes(1:6,end),zeros(6,1));
            ua=full(G)\(-full(drift));initialDelta=diff([previous,U],1,2);
            states{b}=[initialDelta(:);U(:);reshape(nodes(1:ns,2:end),[],1);nodes(1:6,end);ua];
            [states{b},incumbentChecks{b},incumbentTrajectories{b}]=ch5InspectCandidate(states{b},parameter,x,previous,o,r,r.optimizer,preview);
        end
        % The new denoised sequence changes the prior term in the objective.
        % Keep the previously optimized, dynamically consistent decision as
        % the next solve's starting point instead of overwriting only U/D.
        timer=tic;
        try
            solution=ch5SolveProjection(r.optimizer,states{b},parameter,lambda{b});status=r.optimizer.solver.stats();
            constraints=solution.g_before_increment_canonicalization;cost=solution.f;
            violation=max([r.optimizer.lower-constraints;constraints-r.optimizer.upper;0]);
            attempts{step,b}=struct('denoise_step',t,'status',status.return_status,'solver_success',status.success, ...
                'violation_before_increment_canonicalization',violation,'violation',violation, ...
                'increment_correction_inf',solution.increment_correction_inf,'cost',full(cost),'seconds',toc(timer));
            [proposed,proposedCheck,proposedTrajectory,proposedCost]=ch5InspectCandidate(solution.x,parameter,x,previous,o,r,r.optimizer,preview);
            [~,oldCost]=r.optimizer.check(states{b},parameter);oldCost=full(oldCost);oldCheck=incumbentChecks{b};
            replace=(proposedCheck.accepted&&(~oldCheck.accepted||proposedCost<=oldCost))|| ...
                (~oldCheck.accepted&&proposedCheck.finite&&proposedCheck.constraint_score<oldCheck.constraint_score);
            if replace
                states{b}=proposed;incumbentChecks{b}=proposedCheck;incumbentTrajectories{b}=proposedTrajectory;lambda{b}=solution.lam_g;
            else,lambda{b}=[];end
            attempts{step,b}.proposal_check=proposedCheck;attempts{step,b}.retained_incumbent=~replace;
            prior=reshape(states{b}(1:6*H),6,H);
        catch ex
            attempts{step,b}=struct('denoise_step',t,'status','exception','message',ex.message,'violation',inf,'cost',inf,'seconds',toc(timer));
        end
        candidate(:,:,b)=prior;
        clean(:,b)=(prior(:)-double(w.action_mean(:)))./double(w.action_std(:));
    end
    if step<numel(steps)
        nextAlpha=double(w.alpha_bar(steps(step+1)+1));
        noisy=sqrt(nextAlpha)*clean+sqrt(1-nextAlpha)*noise;
    else,noisy=clean;end
end
accepted=false(1,batch);costs=inf(1,batch);checks=cell(1,batch);trajectories=cell(1,batch);
finalOptimizer=ch5PredictiveOptimizer(r,find(o(22,:)>.5),true);finalAttempts=cell(1,batch);
for b=1:batch
    finalParameter=[x;previous;goal;o(:);reshape(lastLearnedPrior(:,:,b),[],1)];timer=tic;
    retained=states{b};retainedCheck=incumbentChecks{b};retainedTrajectory=incumbentTrajectories{b};
    [~,retainedCost]=r.optimizer.check(retained,finalParameter);retainedCost=full(retainedCost);
    try
        solution=ch5SolveProjection(finalOptimizer,states{b},finalParameter,lambda{b});
        states{b}=full(solution.x);candidate(:,:,b)=reshape(states{b}(1:6*H),6,H);status=finalOptimizer.solver.stats();
        finalAttempts{b}=struct('status',status.return_status,'solver_success',status.success,'seconds',toc(timer));
    catch ex
        finalAttempts{b}=struct('status','exception','solver_success',false,'message',ex.message,'seconds',toc(timer));
    end
    U=previous+cumsum(candidate(:,:,b),2);
    try
    [nodes,values,trace]=preview(x,U);nodes=full(nodes);values=full(values);trace=full(trace);
    internalCheck=ch5CheckRollout(nodes,values,trace,U,o,r);
    % Certify the actual rollout of accumulated increments, not approximate
    % auxiliary shooting states returned by a finite optimization.
    exactDecision=states{b};exactDecision(1:12*H)=[reshape(candidate(:,:,b),[],1);U(:)];
    exactDecision(12*H+(1:ns*H))=reshape(nodes(1:ns,2:end),[],1);
    % Eliminate the artificial pose/force analytically at the true terminal
    % pose. This does not alter any candidate input. The equilibrium and
    % actuator/velocity continuation conditions are still checked below.
    artificialPose=nodes(1:6,end);
    [steadyDrift,steadyMap]=r.ad.dynamics(artificialPose,zeros(6,1));
    artificialForce=full(steadyMap)\(-full(steadyDrift));
    exactDecision(12*H+ns*H+(1:12))=[artificialPose;artificialForce];
    [g,cost]=r.optimizer.check(exactDecision,finalParameter);g=full(g);
    eq=r.optimizer.lower==r.optimizer.upper;ineq=~eq;
    equalityViolation=max([abs(g(eq)-r.optimizer.lower(eq));0]);
    equalityScaled=max([abs(g(eq)-r.optimizer.lower(eq))./r.optimizer.g_scale(eq);0]);
    equalityToleranceRatio=max([abs(g(eq)-r.optimizer.lower(eq))./r.optimizer.equality_tolerance(eq);0]);
    inequalityViolation=max([r.optimizer.lower(ineq)-g(ineq);g(ineq)-r.optimizer.upper(ineq);0]);
    violation=max(equalityViolation,inequalityViolation);
    mismatch=norm(reshape(states{b}(12*H+(1:ns*H)),ns,H)-nodes(1:ns,2:end),'inf');
    % Auxiliary-state mismatch is diagnostic. Physical constraints and the
    % terminal condition are checked afresh on the exact control rollout.
    terminal=nodes(7:12,end);accepted(b)=internalCheck.accepted&&equalityToleranceRatio<=1&&inequalityViolation<=1e-6;
    costs(b)=full(cost);checks{b}=struct('accepted',accepted(b),'violation',violation, ...
        'equality_violation',equalityViolation,'scaled_equality_violation',equalityScaled, ...
        'equality_tolerance_ratio',equalityToleranceRatio, ...
        'internal_check',internalCheck, ...
        'inequality_violation',inequalityViolation,'rollout_mismatch',mismatch,'terminal_velocity',terminal);
    trajectories{b}=struct('X',nodes,'U',U,'delta',candidate(:,:,b), ...
        'interval_value_minimum',min(values,[],2),'interval_value_maximum',max(values,[],2), ...
        'interval_value_names',[compose("length_%d",(1:6)');compose("speed_%d",(1:6)'); ...
            compose("acceleration_%d",(1:6)');"sigma_lower_squared";compose("actual_force_%d",(1:6)')], ...
        'artificial_pose',artificialPose,'artificial_force',artificialForce);
    catch ex
        accepted(b)=false;costs(b)=inf;
        checks{b}=struct('accepted',false,'violation',inf,'postcheck_error',ex.message);
        trajectories{b}=struct('X',[],'U',U,'delta',candidate(:,:,b));
    end
    if retainedCheck.accepted&&(~accepted(b)||retainedCost<costs(b))
        states{b}=retained;accepted(b)=true;costs(b)=retainedCost;checks{b}=retainedCheck;
        trajectories{b}=retainedTrajectory;candidate(:,:,b)=retainedTrajectory.delta;
        finalAttempts{b}.retained_incumbent=true;
    else,finalAttempts{b}.retained_incumbent=false;end
end
if any(accepted)
    scores=costs;scores(~accepted)=inf;[~,chosen]=min(scores);u=trajectories{chosen}.U(:,1);
    diagnostic=struct('feasible',true,'usedFallback',false,'exitflag',1,'chosen',chosen,'solver_failure',false);
    setappdata(0,'ch5DiffusionWarm',struct('time',time,'mode',r.mode,'U',trajectories{chosen}.U, ...
        'artificial_force',trajectories{chosen}.artificial_force));
else
    [u,diagnostic]=ch5SafetyQp(x(1:6),x(7:12),x(1:6),zeros(6,1),zeros(6,1),previous,previous,o,r,time);
    diagnostic.usedFallback=true;diagnostic.diffusion_no_acceptable_candidate=true;diagnostic.fallbackSafetyCertified=false;chosen=0;
end
prediction=struct('candidates',{trajectories},'checks',{checks},'denoise_attempts',{attempts},'final_attempts',{finalAttempts},'chosen',chosen,'observed',o,'goal',goal);
end
