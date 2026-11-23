function [candidates,info]=ch5ReducedAffinePriorV46(x,previous,goal,o,anchor,r,k,startTime)
% Cheap bounded affine action projection INSIDE DDIM. Final physics still gates.
timer=tic;w=r.weights;H=r.c.H;learnedH=numel(w.action_mean)/6;
meanAction=reshape(double(w.action_mean(:)),6,learnedH);stdAction=reshape(double(w.action_std(:)),6,learnedH);
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
generator=getappdata(0,'ch5Random');epsilon=randn(generator,6*learnedH,4);
if startTime==99,noisy=epsilon;else
    assert(H==learnedH,'ch5:AffineWarmDimension','Pilot warm initialization uses the original complete horizon.');
    normalized=(diff([previous,anchor],1,2)-meanAction)./stdAction;alpha=double(w.alpha_bar(startTime+1));
    noisy=sqrt(alpha)*normalized(:)+sqrt(1-alpha)*epsilon;
end
count=20;if startTime<99,count=8;end
steps=unique(round(linspace(startTime,0,count)),'stable');target=anchor(:,end)-previous;
weights=stdAction(:,1:H).^2;rate=r.strict.forceRateLimit*r.c.Ts;projectionSeconds=0;maxResidual=0;
for step=1:numel(steps)
    t=steps(step);alpha=double(w.alpha_bar(t+1));noise=ch5Denoise(noisy,condition,t,w);clean=(noisy-sqrt(1-alpha)*noise)/sqrt(alpha);
    phase=tic;
    for ci=1:4
        delta=reshape(clean(:,ci),6,learnedH).*stdAction+meanAction;raw=delta(:,1:H);
        lower=min((raw-rate)./weights,[],2);upper=max((raw+rate)./weights,[],2);
        for iteration=1:40
            multiplier=(lower+upper)/2;projected=min(max(raw-multiplier.*weights,-rate),rate);high=sum(projected,2)>target;
            lower(high)=multiplier(high);upper(~high)=multiplier(~high);
        end
        delta(:,1:H)=min(max(raw-((lower+upper)/2).*weights,-rate),rate);
        maxResidual=max(maxResidual,norm(sum(delta(:,1:H),2)-target,inf));
        normalized=(delta-meanAction)./stdAction;clean(:,ci)=normalized(:);
    end
    projectionSeconds=projectionSeconds+toc(phase);
    if step<numel(steps),next=double(w.alpha_bar(steps(step+1)+1));noisy=sqrt(next)*clean+sqrt(1-next)*noise;end
end
candidates={};
for ci=1:4
    delta=reshape(clean(:,ci),6,learnedH).*stdAction+meanAction;proposal=previous+cumsum(delta(:,1:H),2);last=previous;
    for j=1:H
        proposal(:,j)=min(max(proposal(:,j),max(r.strict.forceMin,last-rate)),min(r.strict.forceMax,last+rate));last=proposal(:,j);
    end
    [~,rawCheck,~,rawCost]=ch5ReducedAuditV23(x,previous,proposal,goal,o,r,k,zeros(6,H),[],false);
    [U,check,trajectory,cost]=ch5ReducedAuditV23(x,previous,proposal,goal,o,r,k,zeros(6,H));
    candidates{ci}=struct('U',U,'X',trajectory.X,'raw_check',rawCheck,'check',check,'cost',cost,'raw_cost',rawCost, ...
        'force_rms_change',sqrt(mean((proposal-anchor).^2,'all')),'final_force_anchor_error',norm(proposal(:,end)-anchor(:,end),inf));
end
info=struct('seconds',toc(timer),'projection_seconds',projectionSeconds,'start_t',startTime,'steps',steps, ...
    'maximum_affine_residual',maxResidual,'scope','Input-rate and terminal-force affine projection only; nonlinear constraints require the final full gate');
end
