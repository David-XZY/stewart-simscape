function out=refineAndValidate(task,candidates,c)
timer=tic; count=size(candidates,3); rankTimer=tic; scores=inf(count,2);
p=dpPhysics(task.model); m=task.model;
for k=1:count
    try
        tr=dpSpline(candidates(:,:,k),linspace(0,task.T,61),task.T);
        [L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F);
        d=zeros(1,61); for i=1:61, ds=dpClearance(tr.q(:,i),task,false); d(i)=min(ds); end
        residual=[max(0,(m.lmin-L)/.1);max(0,(L-m.lmax)/.1);max(0,abs(Ld)/.6-1); ...
            max(0,abs(Ldd)/1.2-1);max(0,abs(F)/2000-1);max(0,(.01-d)/.01)];
        normalizedAcceleration=tr.a./repmat([.1;.1;.1;deg2rad(10)*ones(3,1)],1,size(tr.a,2));
        scores(k,:)=[sum(residual(:).^2),sum(normalizedAcceleration(:).^2)];
    catch
        scores(k,:)=[inf,inf];
    end
end
[~,order]=sortrows(scores,[1,2]); rankingSeconds=toc(rankTimer);
attempts=struct('candidate',{},'passed',{},'seconds',{},'reason',{},'file',{}); best=[];
for ix=1:count
    k=order(ix); attemptTimer=tic; file=fullfile(c.output,sprintf('candidate_%02d.mat',k));
    result=[];
    try
        assert(isfinite(scores(k,1)),'DP:Candidate','Nonfinite candidate.');
        result=dpSolve(task,candidates(:,:,k),c); passed=result.passed; reason=result.solverStatus;
    catch e, passed=false; reason=e.message; end
    if ~isfolder(c.output), mkdir(c.output); end
    save(file,'result','task','reason','-v7');
    attempts(end+1)=struct('candidate',k,'passed',passed,'seconds',toc(attemptTimer),'reason',reason,'file',file); %#ok<AGROW>
    if passed, best=result; break; end
end
out=struct('passed',~isempty(best),'best',best,'attempts',attempts,'order',order,'scores',scores, ...
    'rankingSeconds',rankingSeconds,'totalSeconds',toc(timer));
end
