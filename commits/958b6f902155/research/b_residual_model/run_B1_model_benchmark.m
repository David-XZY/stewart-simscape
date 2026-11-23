function report=run_B1_model_benchmark(datasetFile,assetFile,outputDir)
d=jsondecode(fileread(datasetFile)); n=importResidualNetworkWeights(assetFile);
assert(strcmp(n.datasetHash,hashBytes(datasetFile)),'ABFD:DatasetVersion','Model belongs to different data.');
test=strcmp(string(d.split),'test'); y=d.y(test,:); x=d.x(test,:); mu=zeros(size(y)); variance=mu;
for k=1:size(x,1), [u,S]=evaluateResidualMlp(x(k,:).',n); mu(k,:)=u.'; variance(k,:)=diag(S).'; end
report=struct('physicalOnlyRmse',sqrt(mean(y.^2,1)), ...
    'structuredResidual',evaluateResidualCalibration(y,mu,variance),'testGroupCount',numel(unique(string(d.groups(test)))));
writeAbfdJson(fullfile(outputDir,'B1_model_benchmark.json'),report);
end
function h=hashBytes(file)
fid=fopen(file,'rb'); cleaner=onCleanup(@()fclose(fid)); b=fread(fid,inf,'*uint8'); clear cleaner;
d=java.security.MessageDigest.getInstance('SHA-256'); d.update(b); h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
