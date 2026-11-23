function rank=pgCoarse(bundle,task,c)
% Ranking only. Every returned trajectory still needs the full independent gate.
rank=[1,inf];
try
    tr=bdReconstruct(bundle,linspace(0,task.T,33)); p=pgPhysics(task.model); m=task.model;
    [L,Ld,Ldd,F,J]=p.point(tr.q,tr.v,tr.a); L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F); J=full(J);
    if any(~isfinite([tr.q(:);tr.v(:);tr.a(:);L(:);Ld(:);Ldd(:);F(:);J(:)])), return; end
    gap=pgClearanceBatch(tr.q,task); e=bdRobotEnvelope(task,tr.q,c); sigma=zeros(1,33); cond=sigma;
    for k=1:33, s=svd(J(:,(k-1)*6+(1:6))*diag([1,1,1,2,2,2])); sigma(k)=min(s); cond(k)=max(s)/min(s); end
    groups={[(m.lmin-L)./m.lmax;(L-m.lmax)./m.lmax],abs(Ld)./m.actuator.ldotMax-1,abs(Ldd)./m.actuator.lddotMax-1, ...
        [(m.actuator.forceMin-F)/2000;(F-m.actuator.forceMax)/2000],(.01-[gap;e.gaps])/.01,(.1-sigma)/.1,cond/20-1};
    score=0; violated=false;
    for k=1:numel(groups), g=max(0,groups{k}); if ~isempty(g), score=score+mean(g(:).^2); violated=violated||any(g(:)>0); end; end
    rank=[double(violated),score];
catch
    rank=[1,inf];
end
end
