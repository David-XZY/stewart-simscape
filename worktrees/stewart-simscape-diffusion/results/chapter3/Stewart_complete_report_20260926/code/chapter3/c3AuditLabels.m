function c3AuditLabels(runName)
c=c3Setup(runName); st=tic; source=load(fullfile(c.source,'manifest.mat')); tasks=source.manifest.tasks;
if ~iscell(tasks), tasks=num2cell(tasks); end
folder=fullfile(c.output,'label_audit'); if ~isfolder(folder), mkdir(folder); end
rows={}; kept={};
% Original test labels are never used for calibration, retraining or retrieval.
for i=1:numel(tasks)
    task=tasks{i}; if strcmp(task.split,'test'), continue; end
    for k=1:8
        id=sprintf('%s__%02d',task.taskId,k); target=fullfile(folder,[id,'.mat']);
        if isfile(target), z=load(target); row=z.row;
        else
            file=fullfile(c.source,'attempts',task.taskId,sprintf('start_%02d.mat',k));
            z=load(file); row=struct('labelId',id,'taskId',task.taskId,'split',task.split, ...
                'family',task.family,'source',file,'sourcePassed',z.record.passed,'teacherPassed',false, ...
                'physicalPassed',false,'teacherValues',NaN(1,10),'teacherDetours',[NaN,NaN], ...
                'values',NaN(1,10),'detours',[NaN,NaN],'reason',z.record.reason,'seconds',0);
            clock=tic;
            if z.record.passed
                tr=sgValidate(z.result.bundle,task,c);
                finite=all(isfinite([z.result.bundle.X(:);z.result.bundle.A(:);z.result.bundle.F(:)]));
                bounds=max([0;reshape(task.model.actuator.forceMin-z.result.bundle.F,[],1);reshape(z.result.bundle.F-task.model.actuator.forceMax,[],1)])/1000;
                row.teacherPassed=c3SolverGate(z.result.solverSuccess,z.result.solverStatus,z.result.maxEquality,z.result.maxInequality,bounds,finite,true,tr.passed);
                q=c3Quality(z.result.bundle,task,c); row.teacherValues=q.values; row.teacherDetours=[q.positionDetour,q.rotationDetour];
                b=struct('kind','spline','cp',z.record.cp,'T',task.T); r=sgValidate(b,task,c);
                q=c3Quality(b,task,c); row.physicalPassed=r.passed; row.values=q.values; row.detours=[q.positionDetour,q.rotationDetour];
                row.reason=r.reason;
            end
            row.seconds=toc(clock); save(target,'row','-v7');
        end
        rows{end+1}=row; %#ok<AGROW>
        fprintf('C3_LABEL %s teacher=%d spline=%d\n',id,row.teacherPassed,row.physicalPassed);
    end
end
rows=[rows{:}]; standard=c3FreezeStandard(rows,c); sourceData=jsondecode(fileread(fullfile(c.source,'dataset.json')));
newData=sourceData; X=[]; Y=[]; splits={}; groups={}; families={}; retained={};
for i=1:numel(rows)
    q=struct('values',rows(i).values,'positionDetour',rows(i).detours(1),'rotationDetour',rows(i).detours(2));
    score=c3Score(q,rows(i).family,standard); rows(i).qualityPassed=score.qualityPassed;
    rows(i).retained=rows(i).teacherPassed && rows(i).physicalPassed && score.qualityPassed;
    if ~rows(i).retained, continue; end
    z=load(rows(i).source); task=z.task; base=dpBaseline(task); delta=z.record.cp(:,4:13)-base(:,4:13);
    X(end+1,:)=task.context(:).'; Y(end+1,:)=delta(:).'; %#ok<AGROW>
    splits{end+1}=task.split; groups{end+1}=task.groupId; families{end+1}=task.family; retained{end+1}=rows(i).labelId; %#ok<AGROW>
end
newData.x=X; newData.y=Y; newData.split=splits; newData.groups=groups; newData.family=families;
newData.parentManifestHash=sourceData.manifestHash; newData.manifestHash=dpHash(struct('retained',{retained},'standard',standard.hash));
newData.labelIds=retained; newData.attempts=rows; newData.hash=dpHash(struct('x',X,'y',Y,'splits',{splits}));
dpWriteJson(fullfile(c.output,'dataset.json'),newData);
dpWriteJson(fullfile(c.output,'standard_provisional.json'),standard); save(fullfile(c.output,'standard_provisional.mat'),'standard');
audit=struct('complete',true,'rows',rows,'retained',numel(retained),'sourceAccepted',sum([rows.sourcePassed]), ...
    'requiresRetraining',numel(retained)~=sum([rows.sourcePassed]), ...
    'reason','Audited train/validation dataset; old test labels excluded. Training is re-run to bind the exported model to this audited data version.', ...
    'wallSeconds',toc(st),'summedLabelSeconds',sum([rows.seconds]),'physicalParametersInNetwork',false, ...
    'legacyTeacherObjective','Seed-dependent reference; labels reused only after physical and quality audit, not claimed to optimize the new fixed reference.');
dpWriteJson(fullfile(c.output,'label_audit.json'),audit);
save(fullfile(c.output,'label_audit.mat'),'rows','standard');
fprintf('C3_LABEL_AUDIT_COMPLETE retained=%d/%d\n',numel(retained),sum([rows.sourcePassed]));
end
