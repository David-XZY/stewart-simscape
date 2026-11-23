function report=testFrozenFTransferGuards(inputDir,snapshotDir,outputDir)
assert(~isfolder(outputDir),'ABFD:FGuardFixtureExists','Use a fresh fixture directory.'); mkdir(outputDir);
taskId='task_20260954';
target=fullfile(outputDir,'missing-hash'); copyfile(snapshotDir,target);
file=fullfile(target,'transfer_manifest.json'); manifest=readAbfdJsonUtf8(file);
manifest.files(strcmp({manifest.files.path},[taskId,'/manifest.json']))=[];
writeAbfdJson(file,manifest);
missing=reject(@()loadAbfdFrozenFResult(inputDir,target,taskId),'ABFD:FTransferCoverage');

target=fullfile(outputDir,'changed-candidate'); copyfile(snapshotDir,target);
relative=[taskId,'/',taskId,'/method_02/ranking.mat']; file=fullfile(target,relative);
data=load(file,'ranking','cp'); ranking=data.ranking; cp=data.cp; cp(1,4,1)=cp(1,4,1)+1e-4;
save(file,'ranking','cp'); updateHash(target,relative);
candidates=reject(@()loadAbfdFrozenFResult(inputDir,target,taskId),'ABFD:FCandidateReplay');

target=fullfile(outputDir,'changed-selection'); copyfile(snapshotDir,target); file=fullfile(target,relative);
data=load(file,'ranking','cp'); ranking=data.ranking; cp=data.cp; ranking.selected=1:2;
save(file,'ranking','cp'); updateHash(target,relative);
selection=reject(@()loadAbfdFrozenFResult(inputDir,target,taskId),'ABFD:FSelectedCandidates');
assert(missing && candidates && selection);
report=struct('passed',true,'missingHashRejected',missing,'changedCandidateRejected',candidates, ...
    'changedSelectionRejected',selection,'scope','Intentionally modified copies of real transferred F data; original snapshots and trained assets untouched');
end
function updateHash(target,relative)
file=fullfile(target,'transfer_manifest.json'); manifest=readAbfdJsonUtf8(file);
index=find(strcmp({manifest.files.path},relative)); assert(isscalar(index));
manifest.files(index).sha256=abfdFileHash(fullfile(target,relative));
writeAbfdJson(file,manifest);
end
function rejected=reject(call,id)
rejected=false;
try
    call();
catch e
    assert(strcmp(e.identifier,id),'ABFD:UnexpectedRejection','Expected %s, got %s.',id,e.identifier);
    rejected=true;
end
end
