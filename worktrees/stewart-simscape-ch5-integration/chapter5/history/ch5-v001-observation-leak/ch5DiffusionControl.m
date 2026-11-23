function [u,diagnostic,prediction]=ch5DiffusionControl(x,previous,goal,o,r,time)
% The learned object is a force-increment distribution. A trajectory is
% obtained only by the shared dynamics; there is no LQI reference tracker.
w=r.weights;H=r.c.H;batch=r.c.candidates;generator=getappdata(0,'ch5Random');
r.optimizer=ch5PredictiveOptimizer(r,find(o(22,:)>.5));
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
noisy=randn(generator,6*H,batch);steps=unique(round(linspace(r.c.diffusion_steps-1,0,r.c.ddim_steps)),'stable');
states=cell(batch,1);attempts=cell(numel(steps),batch);lambda=cell(batch,1);candidate=zeros(6,H,batch);
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
        if isempty(states{b})
            nodes=full(r.plant.rollout(x,U));[drift,G]=r.ad.dynamics(nodes(1:6,end),zeros(6,1));
            ua=full(G)\(-full(drift));states{b}=[prior(:);U(:);reshape(nodes(:,2:end),[],1);nodes(1:6,end);ua];
        else,states{b}(1:12*H)=[prior(:);U(:)];end
        timer=tic;
        try
            solution=ch5SolveProjection(r.optimizer,states{b},parameter,lambda{b});states{b}=solution.x;lambda{b}=solution.lam_g;status=r.optimizer.solver.stats();
            [constraints,cost]=r.optimizer.check(states{b},parameter);constraints=full(constraints);
            violation=max([r.optimizer.lower-constraints;constraints-r.optimizer.upper;0]);
            attempts{step,b}=struct('denoise_step',t,'status',status.return_status,'solver_success',status.success, ...
                'violation',violation,'cost',full(cost),'seconds',toc(timer));
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
    finalParameter=[x;previous;goal;o(:);reshape(candidate(:,:,b),[],1)];timer=tic;
    try
        solution=ch5SolveProjection(finalOptimizer,states{b},finalParameter,lambda{b});
        states{b}=full(solution.x);candidate(:,:,b)=reshape(states{b}(1:6*H),6,H);status=finalOptimizer.solver.stats();
        finalAttempts{b}=struct('status',status.return_status,'solver_success',status.success,'seconds',toc(timer));
    catch ex
        finalAttempts{b}=struct('status','exception','solver_success',false,'message',ex.message,'seconds',toc(timer));
    end
    U=previous+cumsum(candidate(:,:,b),2);
    try
    [nodes,values]=r.plant.rollout(x,U);nodes=full(nodes);values=full(values);
    % Certify the actual rollout of accumulated increments, not approximate
    % auxiliary shooting states returned by a finite optimization.
    exactDecision=states{b};exactDecision(1:12*H)=[reshape(candidate(:,:,b),[],1);U(:)];
    exactDecision(12*H+(1:r.plant.n*H))=reshape(nodes(:,2:end),[],1);
    [g,cost]=r.optimizer.check(exactDecision,finalParameter);g=full(g);
    eq=r.optimizer.lower==r.optimizer.upper;ineq=~eq;
    equalityViolation=max([abs(g(eq)-r.optimizer.lower(eq));0]);
    equalityScaled=max([abs(g(eq)-r.optimizer.lower(eq))./r.optimizer.g_scale(eq);0]);
    inequalityViolation=max([r.optimizer.lower(ineq)-g(ineq);g(ineq)-r.optimizer.upper(ineq);0]);
    violation=max(equalityViolation,inequalityViolation);
    mismatch=norm(reshape(states{b}(12*H+1:12*H+r.plant.n*H),r.plant.n,H)-nodes(:,2:end),'inf');
    % Auxiliary-state mismatch is diagnostic. Physical constraints and the
    % terminal condition are checked afresh on the exact control rollout.
    terminal=nodes(7:12,end);accepted(b)=all(isfinite(nodes),'all')&&equalityScaled<=1e-5&&inequalityViolation<=1e-6&&norm(terminal,inf)<=1e-5;
    costs(b)=full(cost);checks{b}=struct('accepted',accepted(b),'violation',violation, ...
        'equality_violation',equalityViolation,'scaled_equality_violation',equalityScaled, ...
        'inequality_violation',inequalityViolation,'rollout_mismatch',mismatch,'terminal_velocity',terminal);
    trajectories{b}=struct('X',nodes,'U',U,'delta',candidate(:,:,b), ...
        'interval_value_minimum',min(values,[],2),'interval_value_maximum',max(values,[],2), ...
        'interval_value_names',[compose("length_%d",(1:6)');compose("speed_%d",(1:6)'); ...
            compose("acceleration_%d",(1:6)');"sigma_lower_squared";compose("actual_force_%d",(1:6)')], ...
        'artificial_pose',states{b}(12*H+r.plant.n*H+(1:6)), ...
        'artificial_force',states{b}(12*H+r.plant.n*H+6+(1:6)));
    catch ex
        accepted(b)=false;costs(b)=inf;
        checks{b}=struct('accepted',false,'violation',inf,'postcheck_error',ex.message);
        trajectories{b}=struct('X',[],'U',U,'delta',candidate(:,:,b));
    end
end
if any(accepted)
    scores=costs;scores(~accepted)=inf;[~,chosen]=min(scores);u=trajectories{chosen}.U(:,1);
    diagnostic=struct('feasible',true,'usedFallback',false,'exitflag',1,'chosen',chosen,'solver_failure',false);
else
    [u,diagnostic]=ch5SafetyQp(x(1:6),x(7:12),x(1:6),zeros(6,1),zeros(6,1),previous,previous,o,r,time);
    diagnostic.usedFallback=true;diagnostic.diffusion_no_acceptable_candidate=true;diagnostic.fallbackSafetyCertified=false;chosen=0;
end
prediction=struct('candidates',{trajectories},'checks',{checks},'denoise_attempts',{attempts},'final_attempts',{finalAttempts},'chosen',chosen,'observed',o,'goal',goal);
end
