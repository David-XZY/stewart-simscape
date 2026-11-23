function [candidates,log]=sgSample(task,count,seed,asset)
% Candidate-ID streams give identical prefixes at every candidate count.
timer=tic; a=asset.model; beta=a.beta(:); abar=cumprod(1-beta); steps=numel(beta);
x=(task.context(:)-a.inputMean(:))./a.inputScale(:); noise=zeros(60,steps,count);
for k=1:count
    stream=RandStream('mt19937ar','Seed',mod(double(seed)+7919*(k-1),2^32-1));
    noise(:,:,k)=randn(stream,60,steps);
end
y=reshape(noise(:,1,:),60,count); initial=y; layers=a.layers; if iscell(layers), layers=[layers{:}]; end
for s=steps:-1:1
    phase=(s-1)/(steps-1)*pi; prediction=[repmat(x,1,count);y;repmat([sin(phase);cos(phase)],1,count)];
    for j=1:numel(layers)
        prediction=layers(j).W*prediction+layers(j).b(:);
        if j<numel(layers), prediction=tanh(prediction); end
    end
    clean=(y-sqrt(1-abar(s))*prediction)/sqrt(abar(s));
    if s>1
        y=sqrt(abar(s-1))*beta(s)/(1-abar(s))*clean+sqrt(1-beta(s))*(1-abar(s-1))/(1-abar(s))*y+ ...
            sqrt(beta(s)*(1-abar(s-1))/(1-abar(s)))*reshape(noise(:,s,:),60,count);
    else, y=clean; end
end
base=dpBaseline(task); candidates=repmat(base,1,1,count);
for k=1:count, candidates(:,4:13,k)=base(:,4:13)+reshape(y(:,k).*a.outputScale(:)+a.outputMean(:),6,10); end
log=struct('seed',seed,'count',count,'initialNoise',initial,'seconds',toc(timer),'modelHash',asset.modelHash);
end
