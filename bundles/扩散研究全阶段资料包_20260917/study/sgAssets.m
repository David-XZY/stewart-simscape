function asset=sgAssets(c)
persistent saved key;
if isempty(key) || ~strcmp(key,c.modelFile)
    model=jsondecode(fileread(c.modelFile)); dataset=jsondecode(fileread(c.datasetFile));
    assert(strcmp(model.status,'trained') && strcmp(model.manifestHash,dataset.manifestHash));
    mask=strcmp(dataset.split,'train'); groups=dataset.groups(mask);
    assert(~any(ismember(groups,dataset.groups(strcmp(dataset.split,'test')))));
    saved=struct('model',model,'modelHash',dpHash(fileread(c.modelFile)), ...
        'trainX',dataset.x(mask,:),'trainY',dataset.y(mask,:),'trainGroups',{groups});
    saved.trainNormalized=(saved.trainX-model.inputMean(:).')./model.inputScale(:).'; key=c.modelFile;
end
asset=saved;
end
