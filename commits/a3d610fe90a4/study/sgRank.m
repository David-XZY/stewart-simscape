function [scores,order]=sgRank(cp,task)
N=size(cp,3); scores=inf(N,2); p=sgPhysics(task.model); m=task.model;
times=linspace(0,task.T,61); reference=sgSpline(dpBaseline(task),times,task.T);
for k=1:N
    tr=sgSpline(cp(:,:,k),times,task.T);
    if any(~isfinite([tr.q(:);tr.v(:);tr.a(:)])), continue; end
    [L,Ld,Ldd,F,~,~,phi]=p.point(tr.q,tr.v,tr.a); L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F); phi=full(phi);
    gap=sgClearanceBatch(tr.q,task);
    residual=[max(0,(m.lmin-L)/.1);max(0,(L-m.lmax)/.1); ...
        max(0,abs(Ld)./m.actuator.ldotMax-1);max(0,abs(Ldd)./m.actuator.lddotMax-1); ...
        max(0,(m.actuator.forceMin-F)/2000);max(0,(F-m.actuator.forceMax)/2000); ...
        max(0,(.01-min(gap,[],1))/.01)];
    deviation=sum(((tr.q-reference.q)./[.1;.1;.1;deg2rad(10)*ones(3,1)]).^2,1);
    rate=diff(F,1,2)/(task.T/60)/1000;
    scores(k,:)=[sum(residual(:).^2),mean(.1*deviation+.3*sum((Ldd/1.2).^2,1)+.4*phi)+.08*mean(sum(rate.^2,1))];
end
[~,order]=sortrows(scores,[1,2]);
end
