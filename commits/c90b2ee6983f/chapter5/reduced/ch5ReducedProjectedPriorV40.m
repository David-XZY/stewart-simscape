function [candidates,info]=ch5ReducedProjectedPriorV40(x,previous,goal,o,anchor,r,k,startTime)
% SDEdit-style initialization and in-denoising feasible-ray projection.
% This bounded heuristic is not an exact DPCC set projection or density sample.
timer=tic;w=r.weights;H=r.c.H;meanAction=double(w.action_mean(:));stdAction=double(w.action_std(:));
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
[anchor,anchorCheck,~,anchorCost]=ch5ReducedAuditV23(x,previous,anchor,goal,o,r,k,zeros(6,H));
assert(anchorCheck.accepted,'ch5:InvalidAnchor','The denoising experiment needs an independently accepted anchor.');
anchorDelta=diff([previous,anchor],1,2);normalized=(anchorDelta(:)-meanAction)./stdAction;
generator=getappdata(0,'ch5Random');epsilon=randn(generator,6*H,4);alpha=double(w.alpha_bar(startTime+1));
noisy=sqrt(alpha)*normalized+sqrt(1-alpha)*epsilon;
steps=unique(round(linspace(startTime,0,min(8,startTime+1))),'stable');
saved=repmat({anchor},1,4);projectionRows={};rays=[1,.5,.25,.125,.0625,0];trust=3;
for step=1:numel(steps)
    t=steps(step);alpha=double(w.alpha_bar(t+1));noise=ch5Denoise(noisy,condition,t,w);
    clean=(noisy-sqrt(1-alpha)*noise)/sqrt(alpha);
    for ci=1:4
        delta=reshape(clean(:,ci).*stdAction+meanAction,6,H);proposal=previous+cumsum(delta,2);
        % Fixed engineering trust bound and command limits; all other limits
        % are checked through the original augmented nonlinear rollout.
        proposal=min(max(proposal,anchor-trust),anchor+trust);last=previous;
        for j=1:H
            proposal(:,j)=min(max(proposal(:,j),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)), ...
                min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));last=proposal(:,j);
        end
        accepted=false;checks={};projectionTimer=tic;
        for ray=rays
            trial=saved{ci}+ray*(proposal-saved{ci});
            [trial,check,~,cost]=ch5ReducedAuditV23(x,previous,trial,goal,o,r,k,zeros(6,H));checks{end+1}=check;
            if check.accepted,saved{ci}=trial;accepted=true;break;end
        end
        assert(accepted,'ch5:ProjectionFailure','The accepted seed itself did not pass rechecking.');
        physical=diff([previous,saved{ci}],1,2);clean(:,ci)=(physical(:)-meanAction)./stdAction;
        projectionRows{end+1}=struct('t',t,'candidate',ci,'ray',ray,'check',check,'cost',cost, ...
            'checks',{checks},'seconds',toc(projectionTimer),'anchor_force_rms_change',sqrt(mean((saved{ci}-anchor).^2,'all')));
    end
    if step<numel(steps)
        next=double(w.alpha_bar(steps(step+1)+1));noisy=sqrt(next)*clean+sqrt(1-next)*noise;
    end
end
candidates={};
for ci=1:4
    [U,check,trajectory,cost]=ch5ReducedAuditV23(x,previous,saved{ci},goal,o,r,k,zeros(6,H));
    candidates{ci}=struct('U',U,'X',trajectory.X,'check',check,'cost',cost, ...
        'force_rms_change',sqrt(mean((U-anchor).^2,'all')),'improved',check.accepted&&cost<anchorCost-1e-10);
end
info=struct('seconds',toc(timer),'start_t',startTime,'steps',steps,'anchor_cost',anchorCost, ...
    'anchor_check',anchorCheck,'trust_force_N',trust,'projection_rows',{projectionRows}, ...
    'scope','Bounded feasible-ray heuristic inside DDIM; no exact projected distribution or stability guarantee');
end
