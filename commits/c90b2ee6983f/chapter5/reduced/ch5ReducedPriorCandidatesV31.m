function [prior,info,candidates]=ch5ReducedPriorCandidatesV31(x,previous,goal,o,r,k)
timer=tic;w=r.weights;
% Convert once per planning call; denoiser arithmetic and 20 DDIM steps stay identical.
for layer=1:4
    name=sprintf('W%d',layer);w.(name)=double(w.(name));name=sprintf('b%d',layer);w.(name)=double(w.(name));
end
H=r.c.H;generator=getappdata(0,'ch5Random');
condition=(ch5Context(x,previous,goal,o,r.mode)-double(w.context_mean(:)))./double(w.context_std(:));
noisy=randn(generator,6*H,r.c.candidates);steps=unique(round(linspace(r.c.diffusion_steps-1,0,r.c.ddim_steps)),'stable');
for step=1:numel(steps)
    t=steps(step);alpha=double(w.alpha_bar(t+1));noise=ch5Denoise(noisy,condition,t,w);
    clean=(noisy-sqrt(1-alpha)*noise)/sqrt(alpha);
    if step<numel(steps),next=double(w.alpha_bar(steps(step+1)+1));noisy=sqrt(next)*clean+sqrt(1-next)*noise;end
end
scores=inf(1,r.c.candidates);priors=cell(1,r.c.candidates);candidates=cell(1,r.c.candidates);
for b=1:r.c.candidates
    delta=reshape(clean(:,b).*double(w.action_std(:))+double(w.action_mean(:)),6,H);
    proposal=previous+cumsum(delta,2);last=previous;
    for j=1:H
        proposal(:,j)=min(max(proposal(:,j),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)),min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));last=proposal(:,j);
    end
    priors{b}=diff([previous,proposal],1,2);nodes=full(k.nodes(x,proposal));candidates{b}=struct('U',proposal,'X',nodes,'delta',priors{b});
    if all(isfinite(nodes),'all')
        scale=[.1;.1;.1;deg2rad(10)*ones(3,1)];
        scores(b)=sum(((nodes(1:6,2:end)-goal)./scale).^2,'all')+.1*sum(nodes(7:12,2:end).^2,'all');
    end
end
[~,selected]=min(scores);prior=priors{selected};info=struct('seconds',toc(timer),'scores',scores,'selected',selected);
end
