function pgSamplingAudit(inputPath,outputPath)
% Exercise the production sampler with explicitly shared noise, not RNG parity.
x=jsondecode(fileread(inputPath)); c=pgConfig('payload_sampling_audit_v2');
t=pgFinalizeTask(x.task,x.task.payload,c); c.controlCount=x.model.controlCount;
a=struct('model',x.model,'callContext',x.context);
[p,~]=pgCandidates(x.model.method,t,1,a,tic,inf,c,20260919,x.noise.');
assert(numel(p)==1&&~isempty(p{1}.bundle),'PG:Sampler','No audit candidate');
dpWriteJson(outputPath,struct('controls',reshape(p{1}.bundle.cp(:,4:end-3),1,[])));
end
