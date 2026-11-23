function [pool,timing]=bdCandidates(method,task,ids,asset,clockHandle,budget,c,seed)
started=tic; preparation=tic; timing=struct('retrieval',0,'sampling',0,'adaptation',0,'regression',0,'control',0);
pool={}; if isempty(ids)||toc(clockHandle)>=budget, return; end
base=bdBaseline(task,c.controlCount); free=4:c.controlCount-3;
if any(strcmp(method,{'M1','M2_H','M2_C','M3_L','M5_P'}))
    x=(task.context(:).'-asset.inputMean)./asset.inputScale;
    if strcmp(method,'M1'), distances=sum((asset.trainNormalized-x).^2,2);
    else, distances=blockDistance(task,asset); end
    [~,order]=sort(distances,'ascend');
    if any(strcmp(method,{'M1','M2_C','M3_L','M5_P'})) && isfield(asset,'fittedAvailable'), order=order(asset.fittedAvailable(order)); end
end
timing.retrieval=toc(preparation);
for id=ids
    if toc(clockHandle)>=budget, break; end
    candidateClock=tic;
    meta=struct('id',id,'method',method,'sourceLabel','','adapter','','raw',true);
    if strcmp(method,'M0')
        if id>8, break; end
        b=struct('kind','spline','T',task.T,'cp',bdSeed(task,id,c));
    elseif any(strcmp(method,{'M2_H','M2_C'}))
        if strcmp(method,'M2_C'), rank=id; else, rank=ceil(id/4); end
        if rank>numel(order), break; end
        h=asset.library{order(rank)}; names={'A0','A5','A7','A5-C'}; adapter=names{1+mod(id-1,4)};
        if strcmp(method,'M2_C'), adapter='A5-C'; end
        path=bdAdapt(h,task,adapter,c); b=struct('kind','path_time_map','path',path,'pathHash',dpHash(path),'T',task.T,'timeControls',[]);
        meta.sourceLabel=h.labelId; meta.adapter=adapter;
    elseif strcmp(method,'M1')
        if id>numel(order), break; end
        cp=base; cp(:,free)=cp(:,free)+reshape(asset.trainY(order(id),:),6,numel(free));
        b=struct('kind','spline','T',task.T,'cp',cp); meta.sourceLabel=asset.library{order(id)}.labelId;
    elseif strcmp(method,'M3_L')
        modes=unique(asset.modeIds(order),'stable'); if id>numel(modes), break; end
        selected=order(asset.modeIds(order)==modes(id)); selected=selected(1:min(asset.localNeighbors,numel(selected)));
        if numel(selected)<3
            y=asset.trainY(selected(1),:); meta.adapter='nearest_fallback';
        else
            X=[ones(numel(selected),1),asset.trainNormalized(selected,:)-x];
            weights=1./max(distances(selected),1e-3); weights=weights/sum(weights);
            penalty=asset.ridge*eye(size(X,2)); penalty(1,1)=0;
            coef=(X.'*(X.*weights)+penalty)\(X.'*(asset.trainY(selected,:).*weights)); y=coef(1,:);
        end
        cp=base; cp(:,free)=cp(:,free)+reshape(y,6,numel(free)); b=struct('kind','spline','T',task.T,'cp',cp);
    elseif any(strcmp(method,{'M3_N','M4'}))
        model=asset.model; if isfield(model,'method'), assert(strcmp(model.method,method),'BD:ModelMethod','Model/method mismatch.'); end
        dim=numel(model.outputMean); x=(task.context(:)-model.inputMean(:))./model.inputScale(:);
        if strcmp(method,'M3_N')
            if id>1, break; end
            y=network(x,model.layers);
        else
            beta=model.beta(:); abar=cumprod(1-beta); steps=numel(beta);
            stream=RandStream('mt19937ar','Seed',mod(double(seed)+7919*(id-1),2^32-1)); noise=randn(stream,dim,steps); y=noise(:,1);
            for j=steps:-1:1
                if toc(clockHandle)>=budget, break; end
                phase=(j-1)/(steps-1)*pi; pred=network([x;y;sin(phase);cos(phase)],model.layers);
                clean=(y-sqrt(1-abar(j))*pred)/sqrt(abar(j));
                if j>1, y=sqrt(abar(j-1))*beta(j)/(1-abar(j))*clean+sqrt(1-beta(j))*(1-abar(j-1))/(1-abar(j))*y+sqrt(beta(j)*(1-abar(j-1))/(1-abar(j)))*noise(:,j);
                else, y=clean; end
            end
            if toc(clockHandle)>=budget, break; end
        end
        cp=base; cp(:,free)=cp(:,free)+reshape(y.*model.outputScale(:)+model.outputMean(:),6,numel(free));
        b=struct('kind','spline','T',task.T,'cp',cp);
    else, error('BD:Method','Method not implemented: %s',method); end
    pool{end+1}=struct('bundle',b,'meta',meta);
    if any(strcmp(method,{'M2_H','M2_C','M1','M0'})), timing.adaptation=timing.adaptation+toc(candidateClock);
    elseif strcmp(method,'M4'), timing.sampling=timing.sampling+toc(candidateClock);
    else, timing.regression=timing.regression+toc(candidateClock); end
end
timing.control=max(0,toc(started)-sum(structfun(@double,timing)));
end
function y=network(y,layers)
if iscell(layers), layers=[layers{:}]; end
for k=1:numel(layers), y=layers(k).W*y+layers(k).b(:); if k<numel(layers), y=tanh(y); end; end
end
function d=blockDistance(task,a)
x=(task.context(:).'-a.inputMean)./a.inputScale; z=a.trainNormalized-x;
% Rotation angles use SO(3); train-only spread scales remain fixed.
pos=mean(z(:,[1:3,7:9]).^2,2); rot=zeros(size(pos));
for k=1:numel(a.library)
    h=a.library{k};
    for pair={[task.q0,h.task.q0],[task.qGoal,h.task.qGoal]}
        q=pair{1}; R=rpy2rotmZYX(q(4:6,1)).'*rpy2rotmZYX(q(4:6,2)); rot(k)=rot(k)+acos(max(-1,min(1,(trace(R)-1)/2)))^2;
    end
end
scale=max(mean(a.inputScale([4:6,10:12]).^2),1e-6); rot=rot/(2*scale);
obs=mean(z(:,16:end).^2,2); times=log(task.T./a.trainX(:,13)).^2/max(var(log(a.trainX(:,13)),1),1e-6);
w=ones(1,4);
switch a.distanceVariant
    case 'legacy', d=sum(z.^2,2); return;
    case 'obstacle_x2', w(3)=2;
    case 'rotation_x2', w(2)=2;
    case 'time_x2', w(4)=2;
end
d=[pos,rot,obs,times]*w.';
end
