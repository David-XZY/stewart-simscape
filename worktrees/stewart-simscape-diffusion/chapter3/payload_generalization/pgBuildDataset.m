function report=pgBuildDataset(runNames,outName)
if nargin<2, outName='payload_dataset_v1'; end
c=pgConfig(outName); root=fileparts(c.output); sources={};
for k=1:numel(runNames), files=dir(fullfile(root,runNames{k},'attempt_*.mat')); for j=1:numel(files), sources{end+1}=fullfile(files(j).folder,files(j).name); end; end
assert(~isfile(fullfile(c.output,'dataset.json')),'PG:Immutable','Dataset already exists.');
audits={}; rawLabels={};
for i=1:numel(sources)
    s=load(sources{i}); if ~isfield(s,'row')||~s.row.passed, continue; end
    task=pgFinalizeTask(s.task,s.task.payload,c); independent=pgValidate(s.sol.bundle,task,c);
    entry=struct('source',sources{i},'taskId',task.taskId,'rawPassed',independent.passed,'reason',independent.reason,'fits',{{}});
    if independent.passed, rawLabels{end+1}=struct('task',task,'bundle',s.sol.bundle,'objective',s.sol.objective,'source',sources{i}); end
    audits{end+1}=entry;
end
assert(~isempty(rawLabels),'PG:Labels','No independently qualified raw labels.');
selected=[]; trials={};
for n=[16,20,24]
    fitted=cell(size(rawLabels)); passes=false(size(rawLabels));
    for i=1:numel(rawLabels)
        h=rawLabels{i}; task=h.task; tt=linspace(0,task.T,1001); tr=bdReconstruct(h.bundle,tt); [B,~,~]=bdBasis(tt,task.T,n);
        cp=bdBaseline(task,n); cp(:,4:end-3)=0; cp(:,4:end-3)=(tr.q-cp*B)/B(4:end-3,:);
        b=struct('kind','spline','T',task.T,'cp',cp); r=pgValidate(b,task,c); fitted{i}=b;
        modeError=sqrt(mean((pgModeFeatures(h.bundle,task)-pgModeFeatures(b,task)).^2)); passes(i)=r.passed&&modeError<=.15;
        trials{end+1}=struct('taskId',task.taskId,'source',h.source,'controlCount',n,'passed',passes(i),'reason',r.reason,'modeError',modeError,'maxPoseError',max(abs(reshape(cp*B-tr.q,[],1))));
    end
    selected=n; if mean(~passes)<=.05, break; end
end
rows={}; library={}; qualityRows={}; modeStore=containers.Map('KeyType','char','ValueType','any');
for i=1:numel(rawLabels)
    h=rawLabels{i}; task=h.task;
    if any(strcmp(task.split,{'train','dev_train'})), split='train';
    elseif any(strcmp(task.split,{'validation','dev_validation'})), split='validation';
    else, error('PG:Leakage','Test labels cannot enter the learning dataset.'); end
    feature=pgModeFeatures(h.bundle,task); old=[]; if isKey(modeStore,task.taskId), old=modeStore(task.taskId); end
    isNew=isempty(old)||min(sqrt(mean((old-feature).^2,1)))>.15;
    if ~isNew || size(old,2)>=4, continue; end
    modeStore(task.taskId)=[old,feature]; id=sprintf('label_%05d',i); cp=fitted{i}.cp;
    metrics=pgQuality(h.bundle,task,c); qualityRows{end+1}=struct('taskId',task.taskId,'shape',task.payload.shape,'split',split,'passed',true,'positionDetour',metrics.positionDetour,'rotationDetour',metrics.rotationDetour);
    record=struct('labelId',id,'task',task,'bundle',h.bundle,'fitted',fitted{i},'fittedPassed',passes(i),'source',h.source,'objective',h.objective);
    save(fullfile(c.output,[id,'.mat']),'record','-v7');
    library{end+1}=struct('labelId',id,'taskId',task.taskId,'split',split,'file',[id,'.mat'],'rawPassed',true,'fittedPassed',passes(i),'source',h.source);
    if ~passes(i), continue; end
    rows{end+1}=struct('labelId',id,'taskId',task.taskId,'baseSceneId',task.baseSceneId,'split',split,'context',task.context(:).', ...
        'signature',pgPhysicsSignature(task).','controls',reshape(cp(:,4:end-3),1,[]),'task',task,'objective',h.objective,'rawPassed',true,'fittedPassed',true);
end
validationTasks={};
for k=1:numel(runNames)
    mf=fullfile(root,runNames{k},'frozen_tasks.json');
    if isfile(mf)
        manifest=jsondecode(fileread(mf)); specs=manifest.tasks;
        for j=1:numel(specs)
            if any(strcmp(specs(j).split,{'dev_validation','validation'}))
                task=pgMakeTask(specs(j),c); validationTasks{end+1}=struct('taskId',task.taskId,'task',task,'context',task.context(:).','signature',pgPhysicsSignature(task).');
            end
        end
        break;
    end
end
data=struct('version','pg_dataset_v1','experimentVersion',c.version,'controlCount',selected,'rows',{rows},'validationTasks',{validationTasks},'fitAudit',{trials},'rawAudit',{audits},'sourceCount',numel(sources));
data.contentHash=dpHash(data); dpWriteJson(fullfile(c.output,'dataset.json'),data);
dpWriteJson(fullfile(c.output,'library.json'),struct('version','pg_library_v1','controlCount',selected,'records',{library},'datasetHash',data.contentHash));
dpWriteJson(fullfile(c.output,'quality_reference_rows.json'),qualityRows);
trainIds={}; valIds={};
for i=1:numel(rows), if strcmp(rows{i}.split,'train'), trainIds{end+1}=rows{i}.taskId; else, valIds{end+1}=rows{i}.taskId; end; end
report=struct('rawQualified',numel(rawLabels),'commonQualified',numel(rows),'controlCount',selected,'fitDamageRate',mean(~passes), ...
    'trainConditions',numel(unique(trainIds)),'validationConditions',numel(unique(valIds)),'trainingAllowed',false,'datasetHash',data.contentHash);
shapes=cellfun(@(r)r.task.payload.shape,rows,'UniformOutput',false);
report.trainingAllowed=report.trainConditions>=50&&report.validationConditions>=20&&numel(unique(shapes))==2&&report.fitDamageRate<=.05;
dpWriteJson(fullfile(c.output,'dataset_report.json'),report);
end
