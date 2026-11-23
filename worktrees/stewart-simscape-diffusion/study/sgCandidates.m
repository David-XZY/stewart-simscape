function [cp,log]=sgCandidates(kind,task,count,seed,asset)
timer=tic;
if strcmp(kind,'diffusion'), [cp,log]=sgSample(task,count,seed,asset); return; end
base=dpBaseline(task); cp=repmat(base,1,1,count); indices=[];
if strcmp(kind,'retrieval')
    x=(task.context(:).'-asset.model.inputMean(:).')./asset.model.inputScale(:).';
    distance=sum((asset.trainNormalized-x).^2,2); [~,indices]=sort(distance);
    indices=indices(1:count);
    for k=1:count, cp(:,4:13,k)=base(:,4:13)+reshape(asset.trainY(indices(k),:),6,10); end
elseif strcmp(kind,'geometric')
    for k=1:count
        if k<=8, cp(:,:,k)=dpSeed(task,k); continue; end
        center=dpSeed(task,1); d=task.qGoal(1:3)-task.q0(1:3); d=d/max(norm(d),1e-12);
        transverse=cross(d,[0;0;1]); if norm(transverse)<.1, transverse=cross(d,[0;1;0]); end
        transverse=transverse/norm(transverse); vertical=cross(d,transverse);
        angle=k*pi*(3-sqrt(5)); amplitude=.03*(1+mod(k-9,4)); wave=sin(pi*(1:10)/11);
        center(1:3,4:13)=center(1:3,4:13)+amplitude*(cos(angle)*transverse+sin(angle)*vertical)*wave;
        center(4:6,4:13)=center(4:6,4:13)+deg2rad(4)*[cos(angle);sin(angle);cos(2*angle)]*wave;
        cp(:,:,k)=center;
    end
else, error('SG:Candidates','Unknown candidate source.'); end
log=struct('kind',kind,'count',count,'seed',seed,'retrievalIndices',indices,'seconds',toc(timer));
end
