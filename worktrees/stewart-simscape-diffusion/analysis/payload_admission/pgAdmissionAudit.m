function summary=pgAdmissionAudit(datasetFile,runName)
% Refit and revalidate all previously qualified splines; frozen sources untouched.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
addpath(fullfile(root,'chapter3','payload_generalization'));
c=pgConfig(runName); data=jsondecode(fileread(datasetFile));
assert(data.controlCount==16,'Admission:Representation','Expected frozen 16-control labels.');
identity=struct('datasetSha256',bdFileHash(datasetFile),'codeSha256',bdFileHash([mfilename('fullpath'),'.m']), ...
    'physicalCodeHash',c.codeHash,'samples',513,'physicalStep',c.denseStep);
identityFile=fullfile(c.output,'audit_identity.json');
if isfile(identityFile)
    assert(isequal(jsondecode(fileread(identityFile)),identity),'Admission:Identity','Changed inputs require a new run.');
else
    dpWriteJson(identityFile,identity);
end
cache=fullfile(c.output,'candidate_cache'); if ~isfolder(cache), mkdir(cache); end
trials=data.fitAudit; trials=trials([trials.passed]);
assert(numel(trials)==1506,'Admission:Census','Unexpected accepted-fit census.');
ids=unique({trials.taskId},'stable'); candidates={}; pairRows={}; checked=0; begun=tic;
for k=1:numel(ids)
    taskId=ids{k}; oldIndex=find(strcmp({data.rows.taskId},taskId),1);
    assert(~isempty(oldIndex),'Admission:Coverage','Accepted task missing from old rows.');
    task=data.rows(oldIndex).task; assert(any(strcmp(task.split,{'train','validation'})),'Admission:Leakage','Test access prohibited.');
    group=trials(strcmp({trials.taskId},taskId)); states=cell(numel(group),1); entries=cell(numel(group),1);
    for j=1:numel(group)
        h=group(j); source=h.source; [~,stem]=fileparts(source);
        recordFile=fullfile(cache,[stem,'.mat']); sourceHash=bdFileHash(source);
        if isfile(recordFile)
            saved=load(recordFile,'record','entry','state'); record=saved.record; entry=saved.entry; state=saved.state;
            assert(strcmp(entry.sourceSha256,sourceHash),'Admission:Identity','Source attempt changed.');
        else
            s=load(source); assert(s.row.passed && strcmp(s.row.taskId,taskId),'Admission:Source','Source mismatch.');
            tt=linspace(0,task.T,1001); tr=bdReconstruct(s.sol.bundle,tt); [B,~,~]=bdBasis(tt,task.T,data.controlCount);
            cp=bdBaseline(task,data.controlCount); cp(:,4:end-3)=0;
            cp(:,4:end-3)=(tr.q-cp*B)/B(4:end-3,:);
            fitted=struct('kind','spline','T',task.T,'cp',cp);
            rawFeature=pgModeFeatures(s.sol.bundle,task); fittedFeature=pgModeFeatures(fitted,task);
            modeError=sqrt(mean((rawFeature-fittedFeature).^2)); physical=pgValidate(fitted,task,c);
            fittedPassed=physical.passed && modeError<=.15;
            assert(fittedPassed,'Admission:Regression','Previously qualified fit failed: %s, %s',source,physical.reason);
            controls=reshape(cp(:,4:end-3),1,[]); oldControlError=NaN;
            if endsWith(data.rows(oldIndex).labelId,sprintf('%05d',find(strcmp({data.rawAudit.source},source),1)))
                oldControlError=max(abs(controls-data.rows(oldIndex).controls(:).'));
                assert(oldControlError<1e-10,'Admission:Refit','Frozen training target differs.');
            end
            record=struct('labelId',stem,'task',task,'bundle',s.sol.bundle,'fitted',fitted, ...
                'fittedPassed',true,'source',source,'objective',s.sol.objective);
            state=sampleState(fitted,task,513);
            entry=struct('labelId',stem,'taskId',taskId,'baseSceneId',task.baseSceneId,'split',task.split, ...
                'family',task.family,'taskClass',task.taskClass,'shape',task.payload.shape,'T',task.T, ...
                'source',source,'sourceSha256',sourceHash,'cacheFile',[stem,'.mat'],'objective',s.sol.objective, ...
                'controls',controls,'rawFeature',rawFeature(:).','fittedFeature',fittedFeature(:).', ...
                'historicalFitPassed',true,'physicalRecheckPassed',physical.passed,'physicalReason',physical.reason, ...
                'physicalSamples',physical.sampleCount,'minClearance',physical.minClearance,'modeFitError',modeError, ...
                'oldControlError',oldControlError,'startId',s.row.startId);
            tmp=[recordFile,'.tmp.mat']; save(tmp,'record','entry','state','-v7'); movefile(tmp,recordFile);
        end
        entries{j}=entry; states{j}=state; candidates{end+1}=entry; checked=checked+1;
        if mod(checked,10)==0
            dpWriteJson(fullfile(c.output,'progress.json'),struct('stage','refit_and_physical_recheck','completed',checked,'total',numel(trials),'seconds',toc(begun),'taskId',taskId));
        end
    end
    for i=1:numel(entries)
        for j=i+1:numel(entries)
            a=entries{i}; b=entries{j}; x=states{i}; y=states{j};
            rot=rotationDistance(x.quaternion,y.quaternion)*180/pi;
            pairRows{end+1}=struct('taskId',taskId,'split',task.split,'labelA',a.labelId,'labelB',b.labelId, ...
                'rawModeRms',sqrt(mean((a.rawFeature-b.rawFeature).^2)), ...
                'fittedModeRms',sqrt(mean((a.fittedFeature-b.fittedFeature).^2)), ...
                'positionRmsMm',vectorRms(x.q(1:3,:)-y.q(1:3,:))*1000, ...
                'positionMaxMm',max(sqrt(sum((x.q(1:3,:)-y.q(1:3,:)).^2,1)))*1000, ...
                'rotationRmsDeg',sqrt(mean(rot.^2)),'rotationMaxDeg',max(rot), ...
                'velocityRmsMmS',vectorRms(x.v(1:3,:)-y.v(1:3,:))*1000, ...
                'angularVelocityRmsDegS',vectorRms(x.omega-y.omega)*180/pi, ...
                'accelerationRmsMmS2',vectorRms(x.a(1:3,:)-y.a(1:3,:))*1000, ...
                'angularAccelerationRmsDegS2',vectorRms(x.alpha-y.alpha)*180/pi, ...
                'legVelocityRmsMmS',scalarRms(x.Ld-y.Ld)*1000, ...
                'legAccelerationRmsMmS2',scalarRms(x.Ldd-y.Ldd)*1000, ...
                'forceRmsN',scalarRms(x.F-y.F),'forceMaxN',max(abs(reshape(x.F-y.F,[],1))));
        end
    end
    fprintf('ADMISSION %d/%d tasks; %d/%d accepted fits rechecked; %.1fs\n',k,numel(ids),checked,numel(trials),toc(begun));
end
dpWriteJson(fullfile(c.output,'candidates.json'),struct('identity',identity,'candidates',{candidates}));
writetable(struct2table([pairRows{:}]),fullfile(c.output,'pairs.csv'));
summary=struct('status','completed','candidateCount',numel(candidates),'taskCount',numel(ids), ...
    'pairCount',numel(pairRows),'physicalRecheckPassed',checked,'seconds',toc(begun),'identity',identity);
dpWriteJson(fullfile(c.output,'audit_summary.json'),summary); dpWriteJson(fullfile(c.output,'progress.json'),summary);
end

function s=sampleState(bundle,task,count)
tr=bdReconstruct(bundle,linspace(0,task.T,count)); p=pgPhysics(task.model);
[~,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); [~,omega,alpha]=p.rotation(tr.q,tr.v,tr.a);
r=tr.q(4,:)/2; b=tr.q(5,:)/2; y=tr.q(6,:)/2;
quat=[cos(r).*cos(b).*cos(y)+sin(r).*sin(b).*sin(y); ...
    sin(r).*cos(b).*cos(y)-cos(r).*sin(b).*sin(y); ...
    cos(r).*sin(b).*cos(y)+sin(r).*cos(b).*sin(y); ...
    cos(r).*cos(b).*sin(y)-sin(r).*sin(b).*cos(y)];
s=struct('q',tr.q,'v',tr.v,'a',tr.a,'Ld',full(Ld),'Ldd',full(Ldd),'F',full(F), ...
    'omega',full(omega),'alpha',full(alpha),'quaternion',quat);
end
function a=rotationDistance(q,r)
sg=sign(sum(q.*r,1)); sg(sg==0)=1; r=r.*sg;
a=4*atan2(sqrt(sum((q-r).^2,1)),sqrt(sum((q+r).^2,1)));
end
function v=vectorRms(x), v=sqrt(mean(sum(x.^2,1))); end
function v=scalarRms(x), v=sqrt(mean(x(:).^2)); end
