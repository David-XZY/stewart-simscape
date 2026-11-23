function r=bdVariationBound(L,vplus,vminus,epsilon)
assert(all(isfinite(L(:)))&&all(vplus>0)&&all(vminus>0)&&all(epsilon(:)>=0),'BD:BoundInput','Invalid bound data.');
if isscalar(epsilon), epsilon=repmat(epsilon,size(L)); end
assert(isequal(size(epsilon),size(L)),'BD:BoundError','Error allowance shape mismatch.');
d=diff(L,1,2); er=epsilon(:,1:end-1)+epsilon(:,2:end);
vp=sum(max(0,d-er),2); vm=sum(max(0,-d-er),2);
r=struct('positiveVariationLower',vp,'negativeVariationLower',vm, ...
    'sampledVariation',sum(abs(d),2),'timeLowerBound',max(vp./vplus(:)+vm./vminus(:)), ...
    'errorAllowanceMax',max(epsilon(:)));
end
