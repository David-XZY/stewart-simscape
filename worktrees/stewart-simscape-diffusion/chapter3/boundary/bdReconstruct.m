function tr=bdReconstruct(b,t)
t=t(:).';
if strcmp(b.kind,'path_time_map')
    assert(strcmp(b.pathHash,dpHash(b.path)),'BD:GeometryChanged','Fixed path hash mismatch.');
    if isempty(b.timeControls), s=t/b.T; beta=ones(size(t))/b.T; alpha=zeros(size(t));
    else
        [B,D,DD]=bdBasis(t,b.T,numel(b.timeControls));
        s=b.timeControls(:).'*B; beta=b.timeControls(:).'*D; alpha=b.timeControls(:).'*DD;
    end
    f=bdPathFunction(b.path); [q,qs,qss]=f(s); q=full(q); qs=full(qs); qss=full(qss);
    tr=struct('t',t,'q',q,'v',qs.*beta,'a',qss.*beta.^2+qs.*alpha,'T',b.T);
elseif strcmp(b.kind,'spline')
    [B,D,DD]=bdBasis(t,b.T,size(b.cp,2)); tr=struct('t',t,'q',b.cp*B,'v',b.cp*D,'a',b.cp*DD,'T',b.T);
else, tr=dpReconstruct(b,t); end
end
