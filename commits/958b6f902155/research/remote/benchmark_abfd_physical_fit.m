function report=benchmark_abfd_physical_fit(collectionDir,outputDir)
% Benchmark training groups already collected. This is not a formal dataset
% export and does not fit normalization or select models using held-out data.
setupAbfdResearch(); tasks=makeAbfdTaskManifest('residual',60,20260905);
attempts=jsondecode(fileread(fullfile(collectionDir,'collection_status.json')));
parts={};
for k=1:numel(attempts)
    id=attempts(k).taskId; index=find(strcmp({tasks.taskId},id));
    if ~strcmp(attempts(k).status,'collected') || ~strcmp(tasks(index).split,'train'), continue; end
    saved=load(fullfile(collectionDir,id,'result.mat'),'result');
    part=buildResidualWrenchTarget(saved.result.log,tasks(index).model);
    n=size(part.x,1); part.split=repmat("train",n,1); part.groupId=repmat(string(id),n,1);
    parts{end+1}=part; %#ok<AGROW>
end
assert(~isempty(parts),'ABFD:NoTrainingSamples','No accepted training sessions.');
data=struct();
for key={'x','acceleration','wrench','split','groupId'}
    values=cellfun(@(p)p.(key{1}),parts,'UniformOutput',false); data.(key{1})=vertcat(values{:});
end
timer=tic; fit=fitPhysicalParameterResidualFast(data,tasks(1).model); elapsed=toc(timer);
physical=tasks(1).model; physical.dynamics.totalMass=fit.mass;
physical.dynamics.comP=fit.com; physical.dynamics.inertiaAtCOM_P=fit.inertia;
errors=zeros(size(data.wrench)); linearErrors=errors;
origin=fit.inertia+fit.mass*((fit.com.'*fit.com)*eye(3)-fit.com*fit.com.');
beta=[fit.mass;fit.mass*fit.com;diag(origin);origin(1,2);origin(1,3);origin(2,3)];
for j=1:size(errors,1)
    q=data.x(j,1:6).'; v=data.x(j,7:12).'; a=data.acceleration(j,:).';
    prediction=computeCompositeRequiredWrench(q,v,a,physical);
    errors(j,:)=prediction.'-data.wrench(j,:);
    linearErrors(j,:)=(buildInertialWrenchRegressor(q,v,a,physical)*beta-prediction).';
end
directLoss=mean((errors./max(std(data.wrench,0,1),1)).^2,'all');
assert(abs(directLoss-fit.identification.dataLoss)<1e-9*max(1,directLoss), ...
    'ABFD:InertialRegressionMismatch','Regressor and original nonlinear data losses disagree.');
report=struct('fit',fit,'totalFitSeconds',elapsed,'groups',unique(data.groupId), ...
    'nonlinearVsRegressorMaxError',max(abs(linearErrors),[],'all'), ...
    'nonlinearVsRegressorLossError',abs(directLoss-fit.identification.dataLoss), ...
    'trainingWrenchRmse',sqrt(mean(errors.^2,1)), ...
    'scope','training-only physical fit timing and identity verification; not held-out model acceptance');
if ~isfolder(outputDir), mkdir(outputDir); end
save(fullfile(outputDir,'physical_fit.mat'),'data','fit','report');
writeAbfdJson(fullfile(outputDir,'physical_fit.json'),report); disp(report);
end
