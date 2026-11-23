function summary=pgExpandDataset(workerIndex,workerCount)
% One label per condition; physical acceptance independent of solver status.
if nargin<1, workerIndex=1; workerCount=1; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
addpath(fullfile(root,'chapter3','payload_generalization'));
runName='payload_condition_expansion_v2_longtime_20260926'; c=pgConfig(runName);
parentFolder=fullfile(root,'results','chapter3','payload_formal_dataset_v1');
sourceFolder=fullfile(root,'results','chapter3','payload_formal_references_v1');
parentFile=fullfile(parentFolder,'dataset.json'); old=jsondecode(readUtf8(parentFile));
manifest=jsondecode(readUtf8(fullfile(here,'candidates.json')));
identity=struct('parentSha256',bdFileHash(parentFile),'manifestSha256',bdFileHash(fullfile(here,'candidates.json')), ...
    'implementationSha256',bdFileHash([mfilename('fullpath'),'.m']),'physicalCodeHash',c.codeHash);
idFile=fullfile(c.output,'identity.json');
if isfile(idFile), assert(isequal(jsondecode(readUtf8(idFile)),identity),'Expansion:Identity','Changed inputs');
else, dpWriteJson(idFile,identity); end
cache=fullfile(c.output,'accepted'); if ~isfolder(cache), mkdir(cache); end
logs=fullfile(c.output,'checks'); if ~isfolder(logs), mkdir(logs); end
attemptDir=fullfile(c.output,'new_attempts'); if ~isfolder(attemptDir), mkdir(attemptDir); end
known={old.rows.taskId}; assert(numel(unique(known))==numel(known),'Expansion:Parent','One parent row per task expected');
begun=tic;

% Reconsider every saved attempt of previously missing, endpoint-valid train tasks.
files=dir(fullfile(sourceFolder,'attempt_*.json')); reviewed=0; rescued=0;
for i=1:numel(files)
    sourceJson=fullfile(files(i).folder,files(i).name); historical=jsondecode(readUtf8(sourceJson));
    if ~strcmp(historical.split,'train') || ismember(historical.taskId,known) || ~historical.endpointPassed, continue; end
    acceptedFile=fullfile(cache,[historical.taskId,'.mat']);
    if isfile(acceptedFile), continue; end
    [~,stem]=fileparts(sourceJson); checkFile=fullfile(logs,[stem,'.json']);
    if isfile(checkFile), continue; end
    source=fullfile(sourceFolder,[stem,'.mat']); s=load(source); reviewed=reviewed+1;
    result=struct('taskId',historical.taskId,'source',source,'historicalPassed',historical.passed,'historicalReason',historical.reason,'origin','historical_recheck');
    if isempty(s.sol) || ~isfield(s.sol,'bundle')
        result.accepted=false; result.reason='no_saved_trajectory';
    else
        task=pgFinalizeTask(s.task,s.task.payload,c);
        [record,result]=checkCandidate(s.sol,task,source,result,c);
        if result.accepted, save(acceptedFile,'record','result','-v7'); rescued=rescued+1; end
    end
    dpWriteJson(checkFile,result);
    progress('historical_recheck',reviewed,0,result);
end

% Freeze a balanced batch based only on endpoint feasibility.
selectionFile=fullfile(c.output,'selected_tasks.json');
if isfile(selectionFile)
    selected=jsondecode(readUtf8(selectionFile)); specs=selected.tasks;
else
    counts=containers.Map('KeyType','char','ValueType','double'); chosen={}; endpointAudit={};
    for i=1:numel(manifest.tasks)
        spec=manifest.tasks(i); key=[spec.family,'_',spec.taskClass,'_',spec.shape];
        if isKey(counts,key)&&counts(key)>=manifest.targetPerStratum, continue; end
        task=pgMakeTask(spec,c);
        endpointAudit{end+1}=struct('taskId',task.taskId,'stratum',key,'passed',task.endpointPassed,'reason',task.endpointReason);
        if ~task.endpointPassed, continue; end
        chosen{end+1}=spec; if ~isKey(counts,key), counts(key)=0; end; counts(key)=counts(key)+1;
        if numel(chosen)==manifest.targetConditions, break; end
    end
    assert(numel(chosen)==manifest.targetConditions,'Expansion:Selection','Insufficient endpoint-valid candidates');
    selected=struct('tasks',{chosen},'endpointAudit',{endpointAudit},'count',numel(chosen),'selectionRule',manifest.selection);
    dpWriteJson(selectionFile,selected); specs=[chosen{:}];
end

claims=fullfile(c.output,'group_claims'); if ~isfolder(claims), mkdir(claims); end
warm=[]; warmGroup=''; ownedGroup=false;
for i=1:numel(specs)
    if mod(i-1,5)==0
        groupFinished=true;
        for j=i:min(i+4,numel(specs)), groupFinished=groupFinished&&isfile(fullfile(logs,[specs(j).taskId,'.json'])); end
        ownedGroup=false;
        if ~groupFinished
            claim=java.io.File(fullfile(claims,[specs(i).baseConditionId,'.lock']));
            ownedGroup=claim.createNewFile();
        end
    end
    if ~ownedGroup, continue; end
    spec=specs(i); checkFile=fullfile(logs,[spec.taskId,'.json']);
    if ~strcmp(warmGroup,spec.baseConditionId), warm=[]; warmGroup=spec.baseConditionId; end
    if isfile(checkFile)
        acceptedFile=fullfile(cache,[spec.taskId,'.mat']);
        if isfile(acceptedFile), cached=load(acceptedFile,'record'); warm=struct('bundle',cached.record.fitted); end
        continue;
    end
    task=pgMakeTask(spec,c); assert(task.endpointPassed,'Expansion:Endpoint','Invalid selected endpoint');
    source=fullfile(attemptDir,[task.taskId,'.mat']);
    result=struct('taskId',task.taskId,'source',source,'historicalPassed',false,'historicalReason','','origin','new_condition');
    sol=[]; timer=tic;
    try
        if isfile(source), saved=load(source,'sol'); sol=saved.sol;
        else
            cc=c; cc.clockHandle=timer; cc.wallBudget=120; cc.maxCpuSeconds=120;
            sol=pgSolve(task,bdSeed(task,1,c),cc,warm);
            save(source,'task','sol','-v7');
        end
        result.historicalPassed=sol.passed; result.historicalReason=sol.solverStatus;
        [record,result]=checkCandidate(sol,task,source,result,c);
        if result.accepted
            save(fullfile(cache,[task.taskId,'.mat']),'record','result','-v7'); warm=struct('bundle',record.fitted);
        elseif isfield(result,'rawPhysical')&&result.rawPhysical.passed
            warm=struct('bundle',sol.bundle);
        end
    catch e
        result.accepted=false; result.reason=['exception:',e.identifier]; result.error=getReport(e,'extended','hyperlinks','off');
        if ~isfile(source), save(source,'task','sol','-v7'); end
    end
    result.attemptAndAuditSeconds=toc(timer); result.family=task.family; result.taskClass=task.taskClass; result.shape=task.payload.shape;
    dpWriteJson(checkFile,result); progress('new_conditions',reviewed,i,result);
end

if workerIndex~=1, summary=struct('status','shard_completed','workerIndex',workerIndex); return; end
joinClock=tic;
while numel(dir(fullfile(logs,'expand_*.json')))<numel(specs)
    assert(toc(joinClock)<14400,'Expansion:Join','Other workers did not finish'); pause(2);
end

% Materialize one physically audited curve per condition. Parent labels stay exact.
rows=num2cell(old.rows); library={}; independent={};
for i=1:numel(rows)
    row=rows{i}; s=load(fullfile(parentFolder,[row.labelId,'.mat']),'record'); record=s.record;
    record.task=row.task; audit=pgValidate(record.fitted,row.task,c);
    assert(audit.passed,'Expansion:ParentAudit','Parent label failed fresh audit');
    save(fullfile(c.output,[row.labelId,'.mat']),'record','-v7');
    library{end+1}=librow(record,row); independent{end+1}=compact(audit,row.taskId,'parent');
end
accepted=dir(fullfile(cache,'*.mat')); newCount=0; rescueCount=0;
for i=1:numel(accepted)
    s=load(fullfile(accepted(i).folder,accepted(i).name)); record=s.record; task=record.task;
    audit=pgValidate(record.fitted,task,c); assert(audit.passed,'Expansion:FinalAudit','Accepted curve failed recheck');
    id=sprintf('expanded_%05d',i); record.labelId=id;
    row=struct('labelId',id,'taskId',task.taskId,'baseSceneId',task.baseSceneId,'split','train','context',task.context(:).', ...
        'signature',pgPhysicsSignature(task).','controls',reshape(record.fitted.cp(:,4:end-3),1,[]),'task',task, ...
        'objective',record.objective,'rawPassed',true,'fittedPassed',true);
    rows{end+1}=row; save(fullfile(c.output,[id,'.mat']),'record','-v7'); library{end+1}=librow(record,row);
    independent{end+1}=compact(audit,task.taskId,s.result.origin);
    if strcmp(s.result.origin,'new_condition'), newCount=newCount+1; else, rescueCount=rescueCount+1; end
end
allRows=[rows{:}]; assert(numel(unique({allRows.taskId}))==numel(rows),'Expansion:Duplicate','Duplicate condition');
data=rmfield(old,'contentHash'); data.rows=rows; data.parentContentHash=old.contentHash;
data.sourceCount=old.sourceCount+numel(specs);
data.parentAuditScope='rawAudit and fitAudit preserve the original dataset history; expanded labels have fresh independent_audits.json and checks/*.json';
data.acceptancePolicy=struct('version','physical_only_one_per_condition_v1','solverSuccessRequired',false, ...
    'optimizationResidualGateRequired',false,'wallTimeAcceptanceGateRequired',false,'fitModeDistanceGateRequired',false, ...
    'qualityCostCapRequired',false,'oneLabelPerCondition',true,'hardPhysicalGate','unmodified pgValidate', ...
    'denseStepSeconds',c.denseStep,'nearStepSeconds',c.nearStep,'clearanceMetres',.01, ...
    'rawPassedMeaning','stored teacher curve passes physical audit; historical solver qualification is separate', ...
    'scope','model-based dense validation; not continuous-time proof or hardware tracking certification');
data.expansionIdentity=identity; data.contentHash=dpHash(data);
dpWriteJson(fullfile(c.output,'dataset.json'),data);
dpWriteJson(fullfile(c.output,'library.json'),struct('version','pg_library_v1','controlCount',16,'records',{library},'datasetHash',data.contentHash));
dpWriteJson(fullfile(c.output,'independent_audits.json'),independent);
summary=struct('status','completed','parentTrainConditions',175,'newConditionsAttempted',numel(specs), ...
    'newConditionsAccepted',newCount,'historicalConditionsRecovered',rescueCount,'trainConditions',175+newCount+rescueCount, ...
    'validationConditions',53,'validationEvaluationConditions',numel(old.validationTasks),'labels',numel(rows), ...
    'oneLabelPerCondition',true,'trainingAllowed',true,'modelRetrained',false,'physicalGateUnchanged',true,'seconds',toc(begun),'datasetHash',data.contentHash);
dpWriteJson(fullfile(c.output,'dataset_report.json'),summary); dpWriteJson(fullfile(c.output,'progress.json'),summary);
fprintf('EXPANSION_COMPLETED train=%d new=%d recovered=%d\n',summary.trainConditions,newCount,rescueCount);
    function progress(stage,reviewed,attempted,last)
        dpWriteJson(fullfile(c.output,sprintf('progress_worker_%d.json',workerIndex)),struct('stage',stage,'workerIndex',workerIndex,'historicalAttemptsReviewedThisSession',reviewed, ...
            'newConditionsAttempted',attempted,'acceptedAdditionalConditions',numel(dir(fullfile(cache,'*.mat'))),'seconds',toc(begun),'last',last));
        fprintf('EXPANSION %s reviewed=%d new=%d task=%s accepted=%d reason=%s\n',stage,reviewed,attempted,last.taskId,last.accepted,last.reason);
    end
end

function [record,result]=checkCandidate(sol,task,source,result,c)
record=[]; result.accepted=false;
try
    tt=linspace(0,task.T,1001); tr=bdReconstruct(sol.bundle,tt); [B,~,~]=bdBasis(tt,task.T,16);
    cp=bdBaseline(task,16); cp(:,4:end-3)=0; cp(:,4:end-3)=(tr.q-cp*B)/B(4:end-3,:);
    fitted=struct('kind','spline','T',task.T,'cp',cp); r=pgValidate(fitted,task,c);
    result.accepted=r.passed; result.reason=r.reason; result.physical=compact(r,task.taskId,result.origin);
    result.solverSuccess=sol.solverSuccess; result.solverObjective=sol.objective;
    result.sourceSha256=bdFileHash(source);
    if r.passed
        % This physically checked curve is the actual teacher target, not an
        % assertion that the historical collocation/force samples were valid.
        objective=sol.objective; if ~isfinite(objective), objective=0; end
        record=struct('labelId','','task',task,'bundle',fitted,'fitted',fitted,'fittedPassed',true, ...
            'source',source,'objective',objective,'sourceSolverPassed',sol.passed,'sourceSolverStatus',sol.solverStatus, ...
            'sourceKind','IHSID_curve_fitted_and_independently_physically_accepted');
    else
        raw=pgValidate(sol.bundle,task,c); result.rawPhysical=compact(raw,task.taskId,result.origin);
    end
catch e
    result.accepted=false; result.reason=['check_exception:',e.identifier]; result.error=getReport(e,'extended','hyperlinks','off');
end
end
function r=compact(a,taskId,origin)
r=struct('taskId',taskId,'origin',origin,'passed',a.passed,'reason',a.reason,'minClearance',a.minClearance, ...
    'minSigma',a.minSigma,'maxCondition',a.maxCondition,'boundaryError',a.boundaryError,'sampleCount',a.sampleCount, ...
    'maxDenseDynamicResidual',a.maxDenseDynamicResidual);
if a.passed
    r.maxAbsLegSpeed=max(abs(a.Ld(:))); r.maxAbsLegAcceleration=max(abs(a.Ldd(:))); r.minForce=min(a.FRequired(:)); r.maxForce=max(a.FRequired(:));
end
end
function r=librow(record,row)
r=struct('labelId',row.labelId,'taskId',row.taskId,'split',row.split,'file',[row.labelId,'.mat'],'rawPassed',true,'fittedPassed',true,'source',record.source);
end
function s=readUtf8(file)
fid=fopen(file,'r','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); s=fread(fid,inf,'*char').';
end
