function report=run_abfd_f_holdout_case(inputDir,outputDir,taskIndex)
% Evaluate a frozen test task; never update normalization, weights or settings.
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
assert(~isfolder(outputDir),'ABFD:HoldoutExists','Use a new output directory; completed tests are not overwritten.');
frozen=load(fullfile(inputDir,'frozen.mat')); task=frozen.tasks(taskIndex);
assert(strcmp(task.split,'test'),'ABFD:HoldoutSplit','Only predeclared test tasks are accepted.');
assert(strcmp(abfdHash(task),frozen.manifest.tasks(taskIndex).taskHash), ...
    'ABFD:HoldoutTaskHash','Frozen task changed.');
audit=readAbfdJsonUtf8(fullfile(inputDir,'import_checks','model_audit.json'));
datasetHash=abfdFileHash(fullfile(inputDir,'proposal_dataset.json'));
assert(audit.passed && strcmp(datasetHash,audit.datasetHash),'ABFD:FDeliveryAudit','Delivery audit mismatch.');
assets=struct('name',{'geometric-single','nearest','mlp'},'file',{'','',''});
assetHashes=cell(1,3); assetHashes{1}='';
for j=2:3
    mode=assets(j).name; assets(j).file=fullfile(inputDir,'models',mode,'proposal_model.json');
    assetHashes{j}=abfdFileHash(assets(j).file); expected=[mode,'AssetHash'];
    assert(strcmp(assetHashes{j},audit.(expected)),'ABFD:FAssetHash','Audited weights changed.');
    asset=readAbfdJsonUtf8(assets(j).file);
    assert(strcmp(asset.datasetHash,datasetHash),'ABFD:FAssetDataset','Weight dataset mismatch.');
end
c=frozen.c; c.outputDir=outputDir;
assert(c.candidateCount==8 && c.refineCount==2 && strcmp(c.designMode,'sequential'), ...
    'ABFD:FHoldoutPolicy','The frozen 8-to-2 sequential policy is required.');
files={'research/f_trajectory_proposal/run_F2_warmstart_benchmark.m', ...
    'research/f_trajectory_proposal/rankAbfdProposalCandidates.m', ...
    'research/f_trajectory_proposal/sampleAbfdCandidates.m', ...
    'research/remote/run_abfd_f_holdout_case.m'};
hashes=cellfun(@(file)abfdFileHash(fullfile(root,file)),files,'UniformOutput',false);
manifest=struct('taskId',task.taskId,'taskIndex',taskIndex,'split',task.split, ...
    'taskHash',abfdHash(task),'datasetHash',datasetHash,'assets',assets,'assetHashes',{assetHashes}, ...
    'configHash',abfdHash(rmfield(c,'outputDir')),'plannerHash',abfdPlannerImplementationHash(), ...
    'entryFiles',{files},'entryHashes',{hashes},'matlab',version, ...
    'scope','Frozen test task; includes tasks with zero original successes; same solver limits and final feasibility acceptance for all methods; no model fitting.');
writeAbfdJson(fullfile(outputDir,'manifest.json'),manifest);
report=run_F2_warmstart_benchmark(task,assets,c,'ranked-top-k');
assert(numel(report.summaries)==3,'ABFD:FHoldoutIncomplete','A method is missing.');
writeAbfdJson(fullfile(outputDir,'task_status.json'),struct('state','complete', ...
    'taskId',task.taskId,'methods',report.summaries,'manifestHash',abfdHash(manifest)));
end
