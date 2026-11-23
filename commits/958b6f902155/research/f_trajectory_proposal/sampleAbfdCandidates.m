function candidates=sampleAbfdCandidates(task,file,count,seed)
assert(isfile(file),'ABFD:MissingProposal','A trained F asset is required.');
a=jsondecode(fileread(file));
assert(a.schemaVersion==1 && strcmp(a.kind,'trajectory-proposal') && strcmp(a.status,'trained'), ...
    'ABFD:ProposalAsset','A valid trained proposal is required.');
context=encodeTaskContext(task); mean=a.inputMean(:); scale=a.inputScale(:);
assert(numel(context)==numel(mean) && all(scale>0) && all(isfinite([mean;scale])), ...
    'ABFD:ProposalContext','Context schema mismatch.');
z=(context-mean)./scale; stream=RandStream('mt19937ar','Seed',seed);
features=zeros(24,count);
switch a.mode
    case 'nearest'
        d=sum(((a.contexts-context.')./scale.').^2,2); [~,order]=sort(d);
        for k=1:count, features(:,k)=a.features(order(mod(k-1,numel(order))+1),:).'; end
    case 'mlp'
        y=mlp(z,a.layers).*a.outputScale(:)+a.outputMean(:); features=repmat(y,1,count);
    case 'diffusion'
        beta=a.beta(:); abar=cumprod(1-beta); steps=numel(beta);
        assert(all(beta>0 & beta<1),'ABFD:DiffusionSchedule','Invalid noise schedule.');
        for k=1:count
            y=randn(stream,24,1);
            for s=steps:-1:1
                phase=(s-1)/(steps-1)*pi;
                noise=mlp([z;y;sin(phase);cos(phase)],a.layers);
                y=(y-beta(s)/sqrt(1-abar(s))*noise)/sqrt(1-beta(s));
                if s>1, y=y+sqrt(beta(s)*(1-abar(s-1))/(1-abar(s)))*randn(stream,24,1); end
            end
            features(:,k)=y.*a.outputScale(:)+a.outputMean(:);
        end
    otherwise, error('ABFD:ProposalMode','Unsupported proposal mode.');
end
assert(all(isfinite(features),'all'),'ABFD:ProposalNonfinite','Nonfinite candidate.');
candidates=zeros(6,10,count);
for k=1:count
    candidates(:,1:3,k)=repmat(task.scene.q0,1,3);
    candidates(:,8:10,k)=repmat(task.scene.qWaypoint,1,3);
    candidates(:,4:7,k)=reshape(features(:,k),6,4);
end
end
function y=mlp(y,layers)
if iscell(layers), layers=[layers{:}]; end
for k=1:numel(layers)
    y=layers(k).W*y+layers(k).b(:); if k<numel(layers), y=tanh(y); end
end
end
