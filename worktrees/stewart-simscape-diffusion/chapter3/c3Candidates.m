function [cp,info]=c3Candidates(kind,task,ids,seed,asset)
if strcmp(kind,'diffusion'), [cp,info]=c3Sample(task,ids,seed,asset); return; end
base=dpBaseline(task); cp=repmat(base,1,1,numel(ids)); indices=[];
if strcmp(kind,'retrieval')
    x=(task.context(:).'-asset.model.inputMean(:).')./asset.model.inputScale(:).';
    [~,order]=sort(sum((asset.trainNormalized-x).^2,2));
    assert(max(ids)<=numel(order),'C3:Library','Insufficient training history.'); indices=order(ids);
    for k=1:numel(ids), cp(:,4:13,k)=base(:,4:13)+reshape(asset.trainY(indices(k),:),6,10); end
else
    for j=1:numel(ids)
        k=ids(j);
        if k<=8, cp(:,:,j)=dpSeed(task,k); continue; end
        center=dpSeed(task,1); d=task.qGoal(1:3)-task.q0(1:3); d=d/max(norm(d),1e-12);
        transverse=cross(d,[0;0;1]); if norm(transverse)<.1, transverse=cross(d,[0;1;0]); end
        transverse=transverse/max(norm(transverse),1e-12); vertical=cross(d,transverse);
        angle=k*pi*(3-sqrt(5)); amplitude=.03*(1+mod(k-9,4)); wave=sin(pi*(1:10)/11);
        center(1:3,4:13)=center(1:3,4:13)+amplitude*(cos(angle)*transverse+sin(angle)*vertical)*wave;
        center(4:6,4:13)=center(4:6,4:13)+deg2rad(4)*[cos(angle);sin(angle);cos(2*angle)]*wave;
        cp(:,:,j)=center;
    end
end
info=struct('kind',kind,'ids',ids,'retrievalIndices',indices,'seed',seed);
end
