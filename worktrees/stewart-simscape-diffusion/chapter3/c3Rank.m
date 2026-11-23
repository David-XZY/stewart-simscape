function [scores,order,evaluationSeconds,selectionSeconds]=c3Rank(cp,task,c,standard)
% Coarse screening only. Final acceptance uses dense physical checks and 5ms scores.
clock=tic; scores=inf(size(cp,3),2); coarse=c; coarse.qualityStep=task.T/60;
p=sgPhysics(task.model); m=task.model; times=linspace(0,task.T,61);
for k=1:size(cp,3)
    tr=sgSpline(cp(:,:,k),times,task.T); if any(~isfinite([tr.q(:);tr.v(:);tr.a(:)])), continue; end
    [L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F);
    gap=sgClearanceBatch(tr.q,task);
    residual=[max(0,(m.lmin-L)/.1);max(0,(L-m.lmax)/.1);max(0,abs(Ld)./m.actuator.ldotMax-1); ...
        max(0,abs(Ldd)./m.actuator.lddotMax-1);max(0,(m.actuator.forceMin-F)/2000); ...
        max(0,(F-m.actuator.forceMax)/2000);max(0,(.01-min(gap,[],1))/.01)];
    scores(k,1)=sum(residual(:).^2);
    q=c3Quality(struct('kind','spline','cp',cp(:,:,k),'T',task.T),task,coarse);
    s=c3Score(q,task.family,standard); scores(k,2)=s.costs(standard.primary);
end
evaluationSeconds=toc(clock); clock=tic; [~,order]=sortrows(scores,[1,2]); selectionSeconds=toc(clock);
end
