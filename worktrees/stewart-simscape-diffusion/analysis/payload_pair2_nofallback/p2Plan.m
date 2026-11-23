function out=p2Plan(task,method,B,seed,asset,c)
% Single online deadline. No ranking, retrieval, warm history or NLP fallback.
clock=tic; clear pgDetourPenalty; events={}; solverEvidence=[]; accepted=false; activeCandidate=0; batch=0;
assert(any(strcmp(method,{'ddpm_basic','ddpm_signature','regression','regression_signature','ihsid'})));
out=struct('taskId',task.taskId,'taskHash',task.hash,'modelHash',task.modelHash,'method',method,'B',B,'T',task.T, ...
 'seed',seed,'returnPolicy',c.returnPolicy,'passed',false,'physicalPassed',false,'qualityPassed',false, ...
 'reason','no_qualified_output','bundle',[],'report',[],'candidateRequested',0,'candidateGenerated',0, ...
 'candidateChecked',0,'candidateEvents',{{}},'bestUnacceptedBundle',[],'selectedCandidateId',NaN, ...
 'returnStage','','firstPhysicalFoundTime',NaN,'firstQualityFoundTime',NaN,'actualReturnTime',NaN, ...
 'signatureSeconds',0,'signatureValidity',[],'guidanceCalls',0,'guidanceSeconds',0,'solver',[], ...
 'attempts',{{}},'repairBlocks',{{}},'rankingCalls',0,'rankingSeconds',0,'ihsidCalls',0,'ihsidSeconds',0);
st=tic; task=pgFinalizeTask(task,task.payload,c); event('input_and_endpoint',st);
if ~task.endpointPassed, out.reason='boundary_rejected'; finish(); return; end
if toc(clock)>=B, finish(); return; end
st=tic;
if ~strcmp(method,'ihsid')
 pgModelCheck(asset.model,method,c);
 if contains(method,'signature')
  [sig,sm]=pgPhysicsSignature(task); asset.callContext=[pgEncodeContext(task,'basic');sig];
  out.signatureSeconds=sm.seconds; out.signatureValidity=sm.validity;
 else, asset.callContext=pgEncodeContext(task,'basic'); end
end
event('encoding_and_retrieval',st); next=1;
while toc(clock)<B
 batch=batch+1;
 if strcmp(method,'ihsid')
  totals=[1,4,8]; if batch>3, break; end; ids=next:totals(batch);
 elseif startsWith(method,'regression')
  if batch>1, break; end; ids=1;
 else, ids=next:next+1;
 end
 next=ids(end)+1; out.candidateRequested=out.candidateRequested+numel(ids);
 st=tic; [generated,gt]=pgCandidates(method,task,ids,asset,clock,B,c,seed); elapsed=toc(st);
 names=fieldnames(gt); subtotal=0;
 for gi=1:numel(names), events{end+1}=struct('stage',names{gi},'seconds',gt.(names{gi})); subtotal=subtotal+gt.(names{gi}); end
 events{end+1}=struct('stage','generation_other','seconds',max(0,elapsed-subtotal));
 complete=numel(generated)==numel(ids)&&all(cellfun(@(x)~isempty(x.bundle),generated));
 for j=1:numel(ids)
  meta=struct('id',ids(j),'event','generated','batch',batch,'complete',false,'batchComplete',complete,'elapsed',toc(clock),'failureReason','not_started_deadline');
  if j<=numel(generated), meta.complete=~isempty(generated{j}.bundle); meta.failureReason=generated{j}.meta.failureReason; end
  out.candidateEvents{end+1}=meta; out.candidateGenerated=out.candidateGenerated+meta.complete;
 end
 % A partial diffusion pair is never admitted, even if its first member exists.
 if ~complete, out.reason='incomplete_batch'; break; end
 for j=1:numel(generated)
  if toc(clock)>=B, break; end
  activeCandidate=ids(j); b=generated{j}.bundle;
  if strcmp(method,'ihsid')
   if solve(b), finish(); return; end
  elseif evaluate(b,'direct',[]), finish(); return;
  end
 end
 if strcmp(method,'ihsid'), continue; end
 for j=1:numel(generated)
  activeCandidate=ids(j); b=generated{j}.bundle; block=0;
  while toc(clock)<B
   block=block+1; st=tic;
   try
    [b,info]=p2Repair(b,task,clock,B,c); event('repair',st);
    info.candidateId=activeCandidate; info.batch=batch; info.block=block; info.elapsed=toc(clock);
    out.repairBlocks{end+1}=info; out.bestUnacceptedBundle=b;
    if evaluate(b,'repair',[]), finish(); return; end
    if ~startsWith(method,'regression')||~info.continueAllowed, break; end
   catch e
    event('repair_exception',st); failure('repair',e); out.reason='repair_exception'; finish(); return;
   end
  end
 end
end
finish();
 function yes=solve(b)
  assert(strcmp(method,'ihsid'),'P2:ForbiddenFallback','Learned method entered solver.');
  out.ihsidCalls=out.ihsidCalls+1; yes=false; st=tic;
  try
   cc=c; cc.clockHandle=clock; cc.wallBudget=B; sol=pgSolve(task,b.cp,cc); elapsed=toc(st);
   parts={'initializationSeconds','buildSeconds','solveSeconds','validationSeconds'};
   names={'solver_initialization','solver_build','solver','physical'}; subtotal=0;
   for ii=1:4, events{end+1}=struct('stage',names{ii},'seconds',sol.(parts{ii})); subtotal=subtotal+sol.(parts{ii}); end
   events{end+1}=struct('stage','solver_other','seconds',max(0,elapsed-subtotal)); out.ihsidSeconds=out.ihsidSeconds+elapsed;
   out.attempts{end+1}=struct('stage','ihsid','candidateId',activeCandidate,'passed',sol.passed,'elapsed',toc(clock), ...
    'reason',sol.solverStatus,'physicalPassed',sol.report.passed,'maxEquality',sol.maxEquality, ...
    'maxInequality',sol.maxInequality,'maxBounds',sol.maxBounds,'buildSeconds',sol.buildSeconds, ...
    'solveSeconds',sol.solveSeconds,'cacheHit',sol.problemCacheHit);
   if sol.passed, yes=evaluate(sol.bundle,'ihsid',sol); end
  catch e, event('ihsid_including_internal_acceptance',st); failure('ihsid',e); end
 end
 function yes=evaluate(b,stage,sol)
  st=tic; [yes,r]=pgAccept(b,task,c.returnPolicy,clock,B,c,sol); elapsed=toc(st);
  events{end+1}=struct('stage','physical','seconds',r.physicalSeconds);
  events{end+1}=struct('stage','quality','seconds',r.qualitySeconds);
  events{end+1}=struct('stage','acceptance_other','seconds',max(0,elapsed-r.physicalSeconds-r.qualitySeconds));
  out.candidateChecked=out.candidateChecked+1;
  out.attempts{end+1}=struct('stage',stage,'candidateId',activeCandidate,'batch',batch,'elapsed',toc(clock), ...
   'passed',yes,'physicalPassed',r.physicalPassed,'qualityPassed',r.qualityPassed,'reason',r.reason);
  out.candidateEvents{end+1}=struct('event','checked','id',activeCandidate,'batch',batch,'stage',stage,'passed',yes,'elapsed',toc(clock));
  if r.physicalPassed&&isnan(out.firstPhysicalFoundTime), out.firstPhysicalFoundTime=toc(clock); end
  if r.qualityPassed&&isnan(out.firstQualityFoundTime), out.firstQualityFoundTime=toc(clock); end
  if yes
   accepted=true; out.bundle=b; out.report=r; out.physicalPassed=true; out.qualityPassed=r.qualityPassed;
   out.returnStage=stage; out.reason='passed'; solverEvidence=sol; out.selectedCandidateId=activeCandidate;
  else, out.bestUnacceptedBundle=b; end
 end
 function failure(stage,e)
  out.attempts{end+1}=struct('stage',stage,'passed',false,'elapsed',toc(clock),'reason',[e.identifier,': ',e.message]);
 end
 function event(name,st), events{end+1}=struct('stage',name,'seconds',toc(st)); end
 function finish()
  assert(strcmp(method,'ihsid')||out.ihsidCalls==0,'P2:ForbiddenFallback','Unexpected solver call.');
  out.solver=solverEvidence; out.events=events; out.payloadShape=task.payload.shape; out.payloadMass=task.payload.mass;
  out.payloadDimensions=task.payload.dimensions; out.derivedInertia=task.derived.inertiaCOM; out.contextSchema=task.contextSchema;
  out.baseSceneId=task.baseSceneId; out.split=task.split; out.codeHash=c.codeHash; out.actualReturnTime=toc(clock);
  elapsed=0; for z=1:numel(events), elapsed=elapsed+events{z}.seconds; end
  out.otherSeconds=out.actualReturnTime-elapsed; assert(out.otherSeconds>=-1e-6,'P2:TimingOverlap','Stage overlap.');
  out.returnedWithinBudget=out.actualReturnTime<=B; out.passed=accepted&&out.returnedWithinBudget;
  out.timeoutOverrun=max(0,out.actualReturnTime-B); if ~out.returnedWithinBudget, out.reason='timeout'; end
 end
end
