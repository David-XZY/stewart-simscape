function [pool,timing]=pgCandidates(method,task,ids,a,clock,budget,c,seed,auditNoise)
started=tic; pool={}; timing=struct('generation',0,'guidance',0); guideSeconds=0;
base=bdBaseline(task,c.controlCount);
for id=ids
    if toc(clock)>=budget, break; end
    meta=struct('id',id,'method',method,'sourceLabelId','','adapterId','','pathHashBefore','','pathHashAfter','','guidanceCalls',0,'guidanceSeconds',0,'failureReason',''); b=[];
    try
        if strcmp(method,'ihsid')
            if id>8, break; end
            b=struct('kind','spline','T',task.T,'cp',bdSeed(task,id,c));
        elseif startsWith(method,'retrieval')
            rank=ceil(id/4); if rank>numel(a.callOrder), break; end
            h=a.library{a.callOrder(rank)}; names={'A0','A5','A7','A5-C'}; adapter=names{mod(id-1,4)+1};
            path=bdAdapt(h,task,adapter,c); hash=dpHash(path);
            b=struct('kind','path_time_map','T',task.T,'path',path,'pathHash',hash,'timeControls',[]);
            meta.sourceLabelId=h.labelId; meta.adapterId=adapter; meta.pathHashBefore=hash; meta.pathHashAfter=hash;
        else
            model=a.model; x=(a.callContext(:)-model.inputMean(:))./model.inputScale(:); dim=numel(model.outputMean);
            if startsWith(method,'regression')
                if id>1, break; end
                y=network(x,model.layers);
            else
                s=model.noiseSchedule; stream=RandStream('mt19937ar','Seed',mod(double(seed)+7919*(id-1),2^32-1)); noise=randn(stream,dim,s.steps); y=noise(:,1);
                if nargin>=9, assert(numel(ids)==1,'PG:AuditNoise','One candidate required.'); noise=auditNoise; y=noise(:,1); end
                for j=s.steps:-1:1
                    if toc(clock)>=budget, break; end
                    phase=(j-1)/(s.steps-1)*pi; eps=network([x;y;sin(phase);cos(phase)],model.layers);
                    clean=(y-sqrt(1-s.alphaBar(j))*eps)/sqrt(s.alphaBar(j));
                    if isfield(model,'cleanProjection'), clean=max(model.cleanProjection.lower(:),min(model.cleanProjection.upper(:),clean)); end
                    if strcmp(method,'ddpm_guided')&&any(j==[20,15,10,5,1])
                        [clean,g]=pgGuide(clean,model,task,clock,budget,c); meta.guidanceCalls=meta.guidanceCalls+g.calls; meta.guidanceSeconds=meta.guidanceSeconds+g.seconds; guideSeconds=guideSeconds+g.seconds;
                    end
                    y=s.posteriorMeanClean(j)*clean+s.posteriorMeanNoisy(j)*y;
                    if j>1, y=y+sqrt(s.posteriorVariance(j))*noise(:,j); end
                end
                if strcmp(method,'ddpm_end_guided')
                    for k=1:5
                        [y,g]=pgGuide(y,model,task,clock,budget,c); meta.guidanceCalls=meta.guidanceCalls+g.calls; meta.guidanceSeconds=meta.guidanceSeconds+g.seconds; guideSeconds=guideSeconds+g.seconds;
                    end
                end
            end
            if toc(clock)>=budget, meta.failureReason='incomplete_candidate_deadline';
            else
                cp=base; cp(:,4:end-3)=reshape(y.*model.outputScale(:)+model.outputMean(:),6,c.controlCount-6);
                b=struct('kind','spline','T',task.T,'cp',cp);
            end
        end
    catch e
        meta.failureReason=[e.identifier,':',e.message];
    end
    pool{end+1}=struct('bundle',b,'meta',meta);
end
timing.guidance=guideSeconds; timing.generation=max(0,toc(started)-guideSeconds);
end
function y=network(y,layers)
if iscell(layers), layers=[layers{:}]; end
for k=1:numel(layers), y=layers(k).W*y+layers(k).b(:); if k<numel(layers), y=tanh(y); end; end
end
