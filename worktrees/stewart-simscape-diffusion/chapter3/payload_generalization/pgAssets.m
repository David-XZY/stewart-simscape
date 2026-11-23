function a=pgAssets(manifestPath,modelPath,signatureIndex)
if nargin<2, modelPath=''; end
if nargin<3, signatureIndex=false; end
manifest=jsondecode(fileread(manifestPath)); assert(strcmp(manifest.version,'pg_library_v1'),'PG:Library','Wrong source manifest.');
root=fileparts(manifestPath); a=struct('library',{{}},'trainX',[],'distanceVariant','equal_blocks');
assert(isfield(manifest,'controlCount'),'PG:Representation','Library representation must be explicit.'); a.controlCount=manifest.controlCount;
for k=1:numel(manifest.records)
    if iscell(manifest.records), r=manifest.records{k}; else, r=manifest.records(k); end
    if ~strcmp(r.split,'train'), continue; end
    data=load(fullfile(root,r.file)); h=data.record;
    assert(any(strcmp(h.task.split,{'train','dev_train'})),'PG:Leakage','Source task split mismatch.');
    a.library{end+1}=h; a.trainX(end+1,:)=h.task.context(:).';
end
assert(~isempty(a.library),'PG:Library','Empty training history.');
ids=cellfun(@(h)h.task.taskId,a.library,'UniformOutput',false); [~,first]=unique(ids,'stable');
a.inputMean=mean(a.trainX(first,:),1); a.inputScale=max(std(a.trainX(first,:),1,1),1e-6);
a.libraryHash=dpHash(manifest);
if signatureIndex
    a.trainSignature=zeros(numel(a.library),255);
    for k=1:numel(a.library), a.trainSignature(k,:)=pgPhysicsSignature(a.library{k}.task).'; end
    a.signatureScale=max(std(a.trainSignature(first,:),1,1),1e-6);
end
if ~isempty(modelPath), a.model=jsondecode(fileread(modelPath)); a.modelHash=dpHash(a.model); end
end
