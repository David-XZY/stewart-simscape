function asset=c3Assets(c)
data=jsondecode(fileread(c.datasetFile)); model=jsondecode(fileread(c.modelFile));
train=data.groups(strcmp(data.split,'train')); validation=data.groups(strcmp(data.split,'validation')); test=data.groups(strcmp(data.split,'test'));
assert(isempty(intersect(train,validation)) && isempty(intersect(train,test)) && isempty(intersect(validation,test)), ...
    'C3:DataLeak','Dataset groups overlap.');
assert(strcmp(dpHash(fileread(c.datasetFile)),model.datasetHash),'C3:ModelData','Model is not bound to this dataset.');
asset=sgAssets(c);
end
