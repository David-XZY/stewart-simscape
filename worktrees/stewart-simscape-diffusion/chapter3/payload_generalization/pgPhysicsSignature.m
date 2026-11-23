function [x,meta]=pgPhysicsSignature(task)
timer=tic; p=pgPhysics(task.model); cp=bdBaseline(task); tr=bdReconstruct(struct('kind','spline','cp',cp,'T',task.T),task.T*[0,.25,.5,.75,1]);
x=zeros(51,5); valid=true(1,5);
for k=1:5
    q=tr.q(:,k); [~,~,~,~,J,W]=p.point(q,zeros(6,1),zeros(6,1)); J=full(J); W=full(W);
    [~,~,~,E]=p.rotation(q,zeros(6,1),zeros(6,1)); E=full(E);
    sv=svd(J*diag([1,1,1,2,2,2])); valid(k)=min(sv)>=1e-4 && rcond(E)>1e-10;
    if valid(k), inverse=@(w)J.'\w; else, inverse=@(w)(J*J.'+1e-8*eye(6))\(J*w); end
    f0=inverse(W); response=zeros(6,6);
    for j=1:6
        a=zeros(6,1); if j<=3, a(j)=1; else, v=zeros(3,1); v(j-3)=1; a(4:6)=pinv(E)*v; end
        [~,~,~,~,~,ww]=p.point(q,zeros(6,1),a); response(:,j)=inverse(full(ww))-f0;
    end
    gap=zeros(6,1); g=pgClearance(q,task,false); gap(1:numel(g))=g;
    values=[f0/2000;response(:)/2000;gap/.01;min(sv)/.1;max(sv)/max(min(sv),1e-12)/20];
    assert(all(isfinite(values)),'PG:Signature','Nonfinite signature.'); x(:,k)=[max(-1000,min(1000,values));double(valid(k))];
end
x=x(:); meta=struct('seconds',toc(timer),'validity',valid,'version','pg_signature255_v1');
end
