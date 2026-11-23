function [candidates,log]=sampleCandidates(task,file,count,seed,guided,c)
timer=tic; asset=jsondecode(fileread(file));
assert(asset.schemaVersion==1 && strcmp(asset.kind,'stewart-full-diffusion') && strcmp(asset.status,'trained'), ...
    'DP:Asset','Expected trained full-horizon diffusion asset.');
assert(numel(asset.outputMean)==60 && numel(task.context)==numel(asset.inputMean), 'DP:Shape','Asset shape mismatch.');
assert(all(asset.inputScale(:)>0) && all(asset.outputScale(:)>0),'DP:Scale','Invalid normalization.');
x=(task.context(:)-asset.inputMean(:))./asset.inputScale(:);
beta=asset.beta(:); abar=cumprod(1-beta); steps=numel(beta); base=dpBaseline(task);
stream=RandStream('mt19937ar','Seed',seed); initial=randn(stream,60,count); y=initial;
fun=[]; if guided, fun=dpGuidance(task,asset,c); end
updates=struct('candidate',{},'step',{},'before',{},'after',{},'accepted',{});
for s=steps:-1:1
    phase=(s-1)/(steps-1)*pi;
    epshat=network([repmat(x,1,count);y;repmat([sin(phase);cos(phase)],1,count)],asset.layers);
    clean=(y-sqrt(1-abar(s))*epshat)/sqrt(abar(s));
    if guided && s<=20 && mod(s,2)==0
        for k=1:count
            for repeat=1:2
                [cost,grad]=fun(clean(:,k)); cost=full(cost); grad=full(grad); before=cost; accepted=false;
                if isfinite(cost) && all(isfinite(grad))
                    delta=c.guidanceStep*grad/max(1,sqrt(mean(grad.^2)));
                    for backtrack=0:4
                        trial=clean(:,k)-delta/(2^backtrack); after=full(fun(trial));
                        if isfinite(after) && after<cost
                            clean(:,k)=trial; cost=after; accepted=true; break;
                        end
                    end
                end
                updates(end+1)=struct('candidate',k,'step',s,'before',before,'after',cost,'accepted',accepted); %#ok<AGROW>
                if ~accepted, break; end
            end
        end
    end
    if s>1
        y=sqrt(abar(s-1))*beta(s)/(1-abar(s))*clean+ ...
            sqrt(1-beta(s))*(1-abar(s-1))/(1-abar(s))*y+ ...
            sqrt(beta(s)*(1-abar(s-1))/(1-abar(s)))*randn(stream,60,count);
    else, y=clean; end
end
candidates=zeros(6,16,count);
for k=1:count
    cp=base; cp(:,4:13)=base(:,4:13)+reshape(y(:,k).*asset.outputScale(:)+asset.outputMean(:),6,10);
    candidates(:,:,k)=cp;
end
log=struct('seed',seed,'guided',guided,'initialNoise',initial,'updates',updates,'seconds',toc(timer), ...
    'assetHash',dpHash(fileread(file)),'taskHash',task.hash);
end
function y=network(y,layers)
if iscell(layers), layers=[layers{:}]; end
for k=1:numel(layers), y=layers(k).W*y+layers(k).b(:); if k<numel(layers), y=tanh(y); end; end
end
