function [u,diagnostic,prediction]=ch5DiffusionControl(x,previous,goal,o,r,time)
% The learned object is a force-increment distribution. A trajectory is
% obtained only by the shared dynamics; there is no LQI reference tracker.
w=r.weights;H=r.c.H;batch=r.c.candidates;generator=getappdata(0,'ch5Random');
persistent lastCallTime lastCallMode
% Reset episode state even when the preceding episode had no acceptable
% plan near its end. A saved plan timestamp alone cannot detect a restart.
if isempty(lastCallTime)||time<=lastCallTime||string(r.mode)~=lastCallMode
    if isappdata(0,'ch5DiffusionWarm'),rmappdata(0,'ch5DiffusionWarm');end
end
lastCallTime=time;lastCallMode=string(r.mode);
preview=ch5NativePreview(r);
nodesPreview=ch5NativeNodes(r);
r.optimizer=ch5PredictiveOptimizer(r,find(o(22,:)>.5));
ns=r.optimizer.shooting_n;
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
noisy=randn(generator,6*H,batch);steps=unique(round(linspace(r.c.diffusion_steps-1,0,r.c.ddim_steps)),'stable');
states=cell(batch,1);attempts=cell(numel(steps),batch);lambda=cell(batch,1);candidate=zeros(6,H,batch);
incumbentChecks=cell(batch,1);incumbentTrajectories=cell(batch,1);lastLearnedPrior=candidate;
feasibleStates=cell(batch,1);feasibleChecks=cell(batch,1);feasibleTrajectories=cell(batch,1);
warm=[];
if isappdata(0,'ch5DiffusionWarm')
    saved=getappdata(0,'ch5DiffusionWarm');
    age=(time-saved.time)/r.c.Ts;shift=round(age);
    if saved.mode==r.mode&&shift>=1&&shift<=H&&abs(age-shift)<1e-6
        warm=saved;warm.shift_steps=shift;
    end
end
[initialU,initialNodes,initialInformation]=ch5ControlInitialGuess(x,previous,warm,r,nodesPreview);
initialValid=all(isfinite(initialNodes),'all')&&all(isfinite(initialU),'all');
initialUa=zeros(6,1);
if initialValid
    [initialDrift,initialG]=r.ad.dynamics(initialNodes(1:6,end),zeros(6,1));
    initialG=full(initialG);initialDrift=full(initialDrift);
    initialValid=all(isfinite(initialG),'all')&&all(isfinite(initialDrift))&&rcond(initialG)>=1e-12;
    if initialValid,initialUa=initialG\(-initialDrift);initialValid=all(isfinite(initialUa));end
end
if ~initialValid
    % A numerical guess outside the model domain cannot be an NLP start.
    % Record rejection and use the existing, explicitly uncertified fallback.
    [u,diagnostic]=ch5SafetyQp(x(1:6),x(7:12),x(1:6),zeros(6,1),zeros(6,1),previous,previous,o,r,time);
    diagnostic.usedFallback=true;diagnostic.diffusion_no_acceptable_candidate=true;diagnostic.fallbackSafetyCertified=false;
    initialInformation.rejection_reason='nonfinite_or_singular_initial_guess';
    rejected=struct('accepted',false,'finite',false,'violation',inf,'rejection_reason',initialInformation.rejection_reason);
    trajectory=struct('X',initialNodes,'U',initialU,'delta',diff([previous,initialU],1,2));
    attempts=repmat({struct('status','rejected_initial_guess','solver_success',false,'seconds',0,'violation',inf,'cost',inf)},numel(steps),batch);
    finalAttempts=repmat({struct('status','rejected_initial_guess','solver_success',false,'seconds',0)},1,batch);
    prediction=struct('candidates',{repmat({trajectory},1,batch)},'checks',{repmat({rejected},1,batch)}, ...
        'denoise_attempts',{attempts},'final_attempts',{finalAttempts},'chosen',0,'observed',o,'goal',goal,'initial_guess',initialInformation);
    return;
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
            U=initialU;nodes=initialNodes;
            ua=initialUa;initialDelta=diff([previous,U],1,2);
            states{b}=[initialDelta(:);U(:);reshape(nodes(r.optimizer.shooting_rows,2:end),[],1);nodes(1:6,end);ua];
            [states{b},incumbentChecks{b},incumbentTrajectories{b}]=ch5InspectCandidate(states{b},parameter,x,previous,o,r,r.optimizer,preview);
            if incumbentChecks{b}.accepted
                feasibleStates{b}=states{b};feasibleChecks{b}=incumbentChecks{b};feasibleTrajectories{b}=incumbentTrajectories{b};
            end
        end
        % The new denoised sequence changes the prior term in the objective.
        % Keep the previously optimized, dynamically consistent decision as
        % the next solve's starting point instead of overwriting only U/D.
        timer=tic;
        try
            solution=ch5SolveProjection(r.optimizer,states{b},parameter,lambda{b});status=solution.stats;
            constraints=solution.g_before_increment_canonicalization;cost=solution.f;
            violation=max([r.optimizer.lower-constraints;constraints-r.optimizer.upper;0]);
            attempts{step,b}=struct('denoise_step',t,'status',status.return_status,'solver_success',status.success, ...
                'violation_before_increment_canonicalization',violation,'violation',violation, ...
                'increment_correction_inf',solution.increment_correction_inf,'cost',full(cost),'seconds',toc(timer));
            attempts{step,b}.constraint_evaluation=solution.constraint_evaluation;
            if isfield(solution,'rejected_solver_output'),attempts{step,b}.rejected_solver_output=solution.rejected_solver_output;end
            [proposed,proposedCheck,proposedTrajectory,proposedCost]=ch5InspectCandidate(solution.x,parameter,x,previous,o,r,r.optimizer,preview);
            [~,oldCost]=r.optimizer.checkExecutable(states{b},parameter);oldCost=full(oldCost);oldCheck=incumbentChecks{b};
            % Standard SQP merit acceptance permits intermediate nonlinear
            % defects while retaining a separate feasible executable backup.
            % Requiring every intermediate SQP point to satisfy a tight
            % terminal equality would prevent any nonzero control update.
            penalty=max(r.c.projection_merit_penalty_floor,1.1*norm(solution.lam_g,inf));
            oldMerit=oldCost+penalty*oldCheck.merit_violation;
            replace=proposedCheck.finite&&proposedCost+penalty*proposedCheck.merit_violation<oldMerit;
            % Backtracking uses the same physical acceptance check. A
            % linearized SQP constraint is never treated as a certificate.
            for line=1:r.c.projection_backtracking_steps
                if replace,break;end
                trial=states{b}+2^(-line)*(solution.x-states{b});
                [trial,trialCheck,trialTrajectory,trialCost]=ch5InspectCandidate(trial,parameter,x,previous,o,r,r.optimizer,preview);
                trialReplace=trialCheck.finite&&trialCost+penalty*trialCheck.merit_violation<oldMerit;
                if trialReplace
                    proposed=trial;proposedCheck=trialCheck;proposedTrajectory=trialTrajectory;proposedCost=trialCost;
                    replace=true;attempts{step,b}.backtracking_fraction=2^(-line);
                end
            end
            if replace
                states{b}=proposed;incumbentChecks{b}=proposedCheck;incumbentTrajectories{b}=proposedTrajectory;lambda{b}=solution.lam_g;
            else,lambda{b}=[];end
            if incumbentChecks{b}.accepted
                backupCost=inf;
                if ~isempty(feasibleStates{b}),[~,backupCost]=r.optimizer.checkExecutable(feasibleStates{b},parameter);backupCost=full(backupCost);end
                [~,workingCost]=r.optimizer.checkExecutable(states{b},parameter);workingCost=full(workingCost);
                if workingCost<=backupCost
                    feasibleStates{b}=states{b};feasibleChecks{b}=incumbentChecks{b};feasibleTrajectories{b}=incumbentTrajectories{b};
                end
            end
            attempts{step,b}.proposal_check=proposedCheck;attempts{step,b}.retained_incumbent=~replace;
            attempts{step,b}.merit_penalty=penalty;
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
    if ~isempty(feasibleStates{b})
        retained=feasibleStates{b};retainedCheck=feasibleChecks{b};retainedTrajectory=feasibleTrajectories{b};
    end
    [~,retainedCost]=r.optimizer.checkExecutable(retained,finalParameter);retainedCost=full(retainedCost);
    try
        solution=ch5SolveProjection(finalOptimizer,states{b},finalParameter,lambda{b});
        states{b}=full(solution.x);candidate(:,:,b)=reshape(states{b}(1:6*H),6,H);status=solution.stats;
        finalAttempts{b}=struct('status',status.return_status,'solver_success',status.success,'seconds',toc(timer));
    catch ex
        finalAttempts{b}=struct('status','exception','solver_success',false,'message',ex.message,'seconds',toc(timer));
    end
    U=previous+cumsum(candidate(:,:,b),2);
    try
    [nodes,values,trace]=preview(x,U);nodes=full(nodes);values=full(values);trace=full(trace);
    if ~all(isfinite(nodes),'all')||~all(isfinite(values),'all')||~all(isfinite(trace),'all')
        error('ch5:NonfiniteCandidate','Nonfinite executable rollout.');
    end
    internalCheck=ch5CheckRollout(nodes,values,trace,U,o,r);
    % Certify the actual rollout of accumulated increments, not approximate
    % auxiliary shooting states returned by a finite optimization.
    exactDecision=states{b};exactDecision(1:12*H)=[reshape(candidate(:,:,b),[],1);U(:)];
    exactDecision(12*H+(1:ns*H))=reshape(nodes(r.optimizer.shooting_rows,2:end),[],1);
    % Eliminate the artificial pose/force analytically at the true terminal
    % pose. This does not alter any candidate input. The equilibrium and
    % actuator/velocity continuation conditions are still checked below.
    artificialPose=nodes(1:6,end);
    [steadyDrift,steadyMap]=r.ad.dynamics(artificialPose,zeros(6,1));
    if ~all(isfinite(full(steadyMap)),'all')||rcond(full(steadyMap))<1e-12
        error('ch5:SingularCandidate','Singular terminal equilibrium map.');
    end
    artificialForce=full(steadyMap)\(-full(steadyDrift));
    exactDecision(12*H+ns*H+(1:12))=[artificialPose;artificialForce];
    [g,cost]=r.optimizer.check(exactDecision,finalParameter);g=full(g);
    eq=r.optimizer.lower==r.optimizer.upper;ineq=~eq;
    equalityViolation=max([abs(g(eq)-r.optimizer.lower(eq));0]);
    equalityScaled=max([abs(g(eq)-r.optimizer.lower(eq))./r.optimizer.g_scale(eq);0]);
    equalityToleranceRatio=max([abs(g(eq)-r.optimizer.lower(eq))./r.optimizer.equality_tolerance(eq);0]);
    inequalityViolation=max([r.optimizer.lower(ineq)-g(ineq);g(ineq)-r.optimizer.upper(ineq);0]);
    violation=max(equalityViolation,inequalityViolation);
    mismatch=norm(reshape(states{b}(12*H+(1:ns*H)),ns,H)-nodes(r.optimizer.shooting_rows,2:end),'inf');
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
        if ismember(ex.identifier,{'ch5:NonfiniteCandidate','ch5:SingularCandidate'})
            checks{b}=struct('accepted',false,'violation',inf,'finite',false,'rejection_reason',ex.identifier);
        else
            checks{b}=struct('accepted',false,'violation',inf,'postcheck_error',ex.message);
        end
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
prediction=struct('candidates',{trajectories},'checks',{checks},'denoise_attempts',{attempts},'final_attempts',{finalAttempts},'chosen',chosen,'observed',o,'goal',goal,'initial_guess',initialInformation);
end
