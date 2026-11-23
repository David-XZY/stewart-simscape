function report=evaluateProposalDiversity(candidates)
count=size(candidates,3); features=reshape(candidates,[],count); distances=[];
for k=1:count, for j=k+1:count, distances(end+1)=norm(features(:,k)-features(:,j)); end, end %#ok<AGROW>
if isempty(distances), meanDistance=0; else, meanDistance=mean(distances); end
report=struct('candidateCount',count,'uniqueCount',size(unique(round(features.',8),'rows'),1), ...
    'meanPairwiseControlPointDistance',meanDistance,'units','mixed metres/radians; diagnostic only');
end
