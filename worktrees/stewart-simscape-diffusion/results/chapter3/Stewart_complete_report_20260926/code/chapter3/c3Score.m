function s=c3Score(q,family,standard)
% No task-specific teacher or test-set normalization is available here.
v=q.values(:).'; normed=v./standard.scales(:).'; groups=zeros(1,4);
for k=1:4, groups(k)=mean(normed(standard.groups{k})); end
costs=standard.weights*groups.'; scores=100./(1+costs);
limit=standard.limits.(family); primary=standard.primary;
finite=all(isfinite([v,groups,q.positionDetour,q.rotationDetour])) && all(v>=0);
groupPass=groups<=limit.groups+1e-12;
detourPass=[q.positionDetour,q.rotationDetour]<=limit.detours+1e-12;
s=struct('groups',groups,'normalizedMetrics',normed,'scores',scores.', ...
    'costs',costs.','primaryScore',scores(primary),'qualityPassed',finite && all(groupPass) && ...
    all(detourPass) && costs(primary)<=limit.costs(primary)+1e-12, ...
    'groupPass',groupPass,'detourPass',detourPass,'standardHash',standard.hash);
end
