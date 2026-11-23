function asset=bdLoadLibrary(manifestFile,split,c)
assert(strcmp(split,'train'),'BD:Leakage','Online library may only use training labels.');
manifest=jsondecode(fileread(manifestFile)); rows=manifest.labels; if iscell(rows), rows=[rows{:}]; end
asset=struct('library',{{}},'trainX',[],'trainY',[],'modeIds',[],'fittedAvailable',[],'libraryHash',dpHash(manifest));
for k=1:numel(rows)
    row=rows(k); assert(strcmp(row.split,'train'),'BD:Leakage','Nontraining row in library manifest.');
    file=row.file;
    if ~isfile(file)
        normalized=strrep(file,'\','/'); positions=strfind(normalized,'/results/');
        if ~isempty(positions), file=fullfile(c.root,normalized(positions(end)+1:end));
        else, file=fullfile(fileparts(manifestFile),file); end
    end
    if isfield(row,'fileHash'), assert(strcmp(bdFileHash(file),row.fileHash),'BD:SourceHash','Historical trajectory file changed.'); end
    data=load(file); assert(data.result.passed,'BD:Label','Source reference failed.');
    assert(strcmp(data.task.hash,row.taskHash),'BD:LabelHash','Task provenance mismatch.');
    assert(strcmp(data.task.split,'train')||strcmp(data.task.split,'development'),'BD:Leakage','Source split forbidden.');
    h=struct('bundle',data.result.bundle,'fitted',data.fitted,'task',data.task,'labelId',row.labelId);
    assert(size(h.fitted.cp,2)==c.controlCount,'BD:Representation','Control count mismatch.');
    asset.library{end+1}=h; asset.trainX(end+1,:)=data.task.context(:).';
    cp=data.fitted.cp-bdBaseline(data.task,c.controlCount); asset.trainY(end+1,:)=reshape(cp(:,4:end-3),1,[]);
    asset.modeIds(end+1)=row.modeId; asset.fittedAvailable(end+1)=data.record.fitPassed;
    if isfield(row,'usableForLearning'), asset.fittedAvailable(end)=asset.fittedAvailable(end)&&row.usableForLearning; end
end
assert(~isempty(asset.library),'BD:Library','Empty training library.');
ids=find(asset.fittedAvailable); assert(~isempty(ids),'BD:Library','No usable common-representation labels.');
groups=cellfun(@(h)h.task.taskId,asset.library(ids),'UniformOutput',false); weights=zeros(numel(ids),1);
for k=1:numel(ids), weights(k)=1/sum(strcmp(groups,groups{k})); end
weights=weights/sum(weights); X=asset.trainX(ids,:); asset.inputMean=sum(X.*weights,1); asset.inputScale=max(sqrt(sum((X-asset.inputMean).^2.*weights,1)),1e-6);
asset.trainNormalized=(asset.trainX-asset.inputMean)./asset.inputScale;
asset.distanceVariant='equal_blocks'; asset.localNeighbors=16; asset.ridge=.01;
end
