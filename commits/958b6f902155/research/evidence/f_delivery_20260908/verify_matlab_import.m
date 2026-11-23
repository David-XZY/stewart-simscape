try
 cd('D:/NPU/Matlab_code/stewart-simscape-abfd');addpath('research');setupAbfdResearch();
 base='results/abfd/compute/F_full_delivery_20260908';out=fullfile(base,'import_checks');
 reference=readAbfdJsonUtf8(fullfile(out,'torch_reference.json'));
 audit=readAbfdJsonUtf8(fullfile(out,'model_audit.json'));
 assert(strcmp(abfdFileHash(fullfile(base,'proposal_dataset.json')),reference.datasetHash),'ABFD:FDataHash','Dataset changed.');
 frozen=load(fullfile(base,'frozen.mat'),'tasks');tasks=frozen.tasks;
 maximumContext=0;maximumMlp=0;maximumNearest=0;maximumBoundary=0;
 for k=1:numel(reference.taskIds)
  task=tasks(strcmp({tasks.taskId},reference.taskIds{k}));assert(isscalar(task),'ABFD:FTask','Frozen task missing.');
  maximumContext=max(maximumContext,norm(encodeTaskContext(task)-reference.contexts(k,:).',inf));
  for mode={'mlp','nearest'}
   file=fullfile(base,'models',mode{1},'proposal_model.json');
   hashField=[mode{1},'AssetHash'];assert(strcmp(abfdFileHash(file),audit.(hashField)),'ABFD:FAssetHash','Asset changed.');
   candidate=sampleAbfdCandidates(task,file,8,20260905);assert(isequal(size(candidate),[6,10,8]),'ABFD:FShape','Candidate packing changed.');
   for j=1:8
    if strcmp(mode{1},'mlp'), expected=reshape(reference.mlpFeatures(k,:),6,4);
    else, expected=reshape(reference.nearestFeatures(k,j,:),6,4);end
    difference=norm(candidate(:,4:7,j)-expected,'fro');
    if strcmp(mode{1},'mlp'),maximumMlp=max(maximumMlp,difference);else,maximumNearest=max(maximumNearest,difference);end
    maximumBoundary=max([maximumBoundary,norm(candidate(:,1:3,j)-repmat(task.scene.q0,1,3),'fro'), ...
     norm(candidate(:,8:10,j)-repmat(task.scene.qWaypoint,1,3),'fro')]);
   end
  end
 end
 assert(maximumContext<1e-12 && maximumMlp<1e-10 && maximumNearest<1e-12 && maximumBoundary==0, ...
  'ABFD:FImportContract','Trained F sampling disagrees with the checkpoint or frozen task boundaries.');
 report=struct('passed',true,'tasks',numel(reference.taskIds),'candidatesPerTaskPerMode',8, ...
  'contextMaximumError',maximumContext,'mlpCheckpointMaximumError',maximumMlp, ...
  'nearestMaximumError',maximumNearest,'boundaryMaximumError',maximumBoundary, ...
  'datasetHash',reference.datasetHash,'mlpAssetHash',audit.mlpAssetHash,'nearestAssetHash',audit.nearestAssetHash, ...
  'scope','Actual MATLAB sampler versus PyTorch float64 checkpoint and train-only nearest rows; 6x10 approach control-point packing; optimizer feasibility still required');
 writeAbfdJson(fullfile(out,'matlab_import.json'),report);
catch e
 writeAbfdJson('D:/NPU/Matlab_code/stewart-simscape-abfd/results/abfd/compute/F_full_delivery_20260908/import_checks/failure.json',struct('identifier',e.identifier,'message',e.message,'report',getReport(e,'extended','hyperlinks','off')));rethrow(e);
end
