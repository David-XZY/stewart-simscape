function report=pgExpansionRetimeRecovery()
% Reuse a feasible path only within exactly the same geometry/load condition.
% Every target duration receives a complete new physical audit.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here)); addpath(fullfile(root,'chapter3','payload_generalization'));
c=pgConfig('payload_condition_expansion_v2_longtime_20260926');
reportFile=fullfile(c.output,'time_scaling_report.json');
if isfile(reportFile), report=jsondecode(readUtf8(reportFile)); return; end
data=jsondecode(readUtf8(fullfile(c.output,'dataset.json'))); rows=num2cell(data.rows);
lib=jsondecode(readUtf8(fullfile(c.output,'library.json'))); library=num2cell(lib.records);
audits=jsondecode(readUtf8(fullfile(c.output,'independent_audits.json'))); independent=num2cell(audits);
selected=jsondecode(readUtf8(fullfile(c.output,'selected_tasks.json'))); specs=selected.tasks;
summary=jsondecode(readUtf8(fullfile(c.output,'dataset_report.json')));
prior=fullfile(c.output,'checks_before_time_scaling'); if ~isfolder(prior), mkdir(prior); end
added={}; trials={};
for i=1:numel(specs)
    spec=specs(i); allRows=[rows{:}]; if ismember(spec.taskId,{allRows.taskId}), continue; end
    same=find(strcmp({specs.baseConditionId},spec.baseConditionId));
    % Prefer the shortest qualified duration: slowing down is often feasible,
    % but force limits and all other constraints are still explicitly tested.
    times=arrayfun(@(x)x.parameters(4),specs(same)); [~,order]=sort(times); same=same(order);
    task=pgMakeTask(spec,c);
    for j=same(:).'
        donorSpec=specs(j); match=find(strcmp({allRows.taskId},donorSpec.taskId),1); if isempty(match), continue; end
        donor=allRows(match); saved=load(fullfile(c.output,[donor.labelId,'.mat']),'record'); donorRecord=saved.record;
        assert(isequal(spec.q0,donorSpec.q0)&&isequal(spec.qGoal,donorSpec.qGoal)&&isequal(spec.dimensions,donorSpec.dimensions)&& ...
            spec.mass==donorSpec.mass&&strcmp(spec.family,donorSpec.family)&&isequal(spec.parameters(1:3),donorSpec.parameters(1:3)), ...
            'Expansion:Transfer','Time scaling must preserve complete physical condition except T');
        fitted=donorRecord.fitted; fitted.T=task.T; a=pgValidate(fitted,task,c);
        trials{end+1}=struct('taskId',task.taskId,'donorTaskId',donor.taskId,'targetT',task.T,'donorT',donor.task.T,'passed',a.passed,'reason',a.reason);
        if ~a.passed, continue; end
        id=sprintf('time_scaled_%05d',numel(added)+1); objective=commonCost(a,task);
        record=struct('labelId',id,'task',task,'bundle',fitted,'fitted',fitted,'fittedPassed',true, ...
            'source',donorRecord.source,'objective',objective,'objectiveSource','dense_common_cost_after_time_scaling', ...
            'sourceKind','time_scaled_IHSID_derived_spline_physically_revalidated','donorLabelId',donor.labelId,'donorT',donor.task.T);
        a=pgValidate(record.fitted,task,c); assert(a.passed,'Expansion:TransferAudit','Transfer failed independent repeat audit');
        file=fullfile(c.output,[id,'.mat']); save(file,'record','-v7');
        row=struct('labelId',id,'taskId',task.taskId,'baseSceneId',task.baseSceneId,'split','train','context',task.context(:).', ...
            'signature',pgPhysicsSignature(task).','controls',reshape(fitted.cp(:,4:end-3),1,[]),'task',task,'objective',objective,'rawPassed',true,'fittedPassed',true);
        rows{end+1}=row; library{end+1}=struct('labelId',id,'taskId',task.taskId,'split','train','file',[id,'.mat'],'rawPassed',true,'fittedPassed',true,'source',record.source);
        compact=struct('taskId',task.taskId,'origin','new_condition','passed',true,'reason','passed','minClearance',a.minClearance, ...
            'minSigma',a.minSigma,'maxCondition',a.maxCondition,'boundaryError',a.boundaryError,'sampleCount',a.sampleCount, ...
            'maxDenseDynamicResidual',a.maxDenseDynamicResidual,'maxAbsLegSpeed',max(abs(a.Ld(:))), ...
            'maxAbsLegAcceleration',max(abs(a.Ldd(:))),'minForce',min(a.FRequired(:)),'maxForce',max(a.FRequired(:)));
        independent{end+1}=compact;
        checkFile=fullfile(c.output,'checks',[task.taskId,'.json']); originalCheck=fullfile(prior,[task.taskId,'.json']);
        if ~isfile(originalCheck), copyfile(checkFile,originalCheck); end
        result=jsondecode(readUtf8(originalCheck));
        result.originalAttemptSource=result.source; result.originalAttemptAccepted=result.accepted; result.originalAttemptReason=result.reason;
        result.accepted=true; result.reason='passed'; result.source=file; result.sourceSha256=bdFileHash(file); result.physical=compact;
        result.acceptanceMethod='time_scaled_IHSID_path'; result.donorTaskId=donor.taskId; result.targetT=task.T; result.donorT=donor.task.T;
        dpWriteJson(checkFile,result); save(fullfile(c.output,'accepted',[task.taskId,'.mat']),'record','result','-v7');
        added{end+1}=struct('taskId',task.taskId,'donorTaskId',donor.taskId,'targetT',task.T,'donorT',donor.task.T,'labelId',id);
        break;
    end
end
data=rmfield(data,'contentHash'); data.rows=rows; data.acceptancePolicy.sameConditionTimeScalingAllowed=true;
data.contentHash=dpHash(data); dpWriteJson(fullfile(c.output,'dataset.json'),data);
lib.records=library; lib.datasetHash=data.contentHash; dpWriteJson(fullfile(c.output,'library.json'),lib);
dpWriteJson(fullfile(c.output,'independent_audits.json'),independent);
summary.newConditionsAccepted=summary.newConditionsAccepted+numel(added); summary.trainConditions=summary.trainConditions+numel(added);
summary.labels=summary.labels+numel(added); summary.timeScaledRecovered=numel(added); summary.datasetHash=data.contentHash;
dpWriteJson(fullfile(c.output,'dataset_report.json'),summary);
dpWriteJson(fullfile(c.output,'progress.json'),summary);
report=struct('status','completed','recovered',numel(added),'added',{added},'trials',{trials},'hardPhysicalGateUnchanged',true);
dpWriteJson(reportFile,report);
end
function value=commonCost(a,task)
p=pgPhysics(task.model); [~,~,~,~,~,~,phi]=p.point(a.q,a.v,a.a); phi=full(phi);
ref=sgSpline(dpBaseline(task),a.time,task.T);
deviation=sum(((a.q-ref.q)./[.1;.1;.1;deg2rad(10)*ones(3,1)]).^2,1);
integrand=.1*deviation+.3*sum((a.Ldd/1.2).^2,1)+.4*phi;
dt=diff(a.time); df=diff(a.FRequired,1,2)./dt/1000;
value=trapz(a.time,integrand)+.08*sum(sum(df.^2,1).*dt);
assert(isscalar(value)&&isfinite(value),'Expansion:Cost','Invalid transfer diagnostic cost');
end
function s=readUtf8(file)
fid=fopen(file,'r','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); s=fread(fid,inf,'*char').';
end
