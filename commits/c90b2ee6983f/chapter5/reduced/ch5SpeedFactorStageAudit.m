function report=ch5SpeedFactorStageAudit()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','factor_stage_audit_v1');
if ~isfolder(directory),mkdir(directory);end
r=ch5ReducedPrepare(1,"nonideal-force");k=ch5ReducedKernels(r);timer=tic;stage=ch5ReducedFactorStage(r);buildSeconds=toc(timer);
saved=load(fullfile(root,'chapter5','artifacts','reduced_probe','closed_fast_v3','001_nonideal_force.mat'),'records');
indices=[1,2,8,26,100,250,500,750];rows={};
for index=indices
    record=saved.records{index};[y,A,B]=stage(record.x,record.u);[yr,Ar,Br]=k.stage(record.x,record.u);
    errors=[norm(full(y-yr),inf),norm(full(A-Ar),inf),norm(full(B-Br),inf)];
    physical=full(r.plant.fd(record.x,record.u,zeros(6,1)));primalError=norm(full(y)-physical,inf);
    rows{end+1}=struct('sample',index,'errors',errors,'physical_primal_error',primalError,'accepted',max(errors)<1e-8&&primalError<1e-8);
end
sample=saved.records{26};brake=ch5ReducedBrakeSequence(r);U=full(brake(sample.x,sample.previous_u));nodes=full(k.nodes(sample.x,U));
mapped=stage.map(r.c.H);oldSeconds=zeros(1,5);newSeconds=oldSeconds;
for j=1:5
    timer=tic;[yr,Ar,Br]=k.stages(nodes(:,1:r.c.H),U);oldSeconds(j)=toc(timer);
    timer=tic;[y,A,B]=mapped(nodes(:,1:r.c.H),U);newSeconds(j)=toc(timer);
end
report=struct('rows',{rows},'build_seconds',buildSeconds,'original_seconds',oldSeconds,'factor_seconds',newSeconds, ...
    'identity',ch5ReducedIdentity(),'pid',feature('getpid'),'accepted',all(cellfun(@(s)s.accepted,rows)));
[goal,o]=ch5Observe(r.task,sample.time);prior=ones(6,r.c.H)*16;
[~,~,costZero]=ch5ReducedAuditV4(sample.x,sample.previous_u,U,goal,o,r,k,zeros(size(U)));
[~,trajectory,costPrior]=ch5ReducedAuditV4(sample.x,sample.previous_u,U,goal,o,r,k,prior);
expected=.05*sum(((trajectory.delta-prior)/160).^2-(trajectory.delta/160).^2,'all');
report.prior_cost=struct('difference',costPrior-costZero,'expected',expected,'accepted',abs(costPrior-costZero-expected)<1e-9);
report.accepted=report.accepted&&report.prior_cost.accepted;
save(fullfile(directory,'audit.mat'),'report','y','A','B','yr','Ar','Br','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Factorized derivatives accepted=%d old %.6fs new %.6fs cold %.2fs\n',report.accepted,median(oldSeconds),median(newSeconds),buildSeconds);
assert(report.accepted,'ch5:FactorMismatch','Factored substep sensitivity must agree with the original derivative map.');
end
