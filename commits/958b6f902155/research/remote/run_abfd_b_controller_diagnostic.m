function report=run_abfd_b_controller_diagnostic(bundleFile,assetFile,outputDir,resume)
% Pilot mechanism comparison on one frozen training-pool reference/true plant.
% B's own TVLQR design is explicit here; the fixed-gain B ablation is separate.
setupAbfdResearch(); saved=load(bundleFile,'bundle'); original=saved.bundle;
if nargin<4, resume=false; end
provider=buildDynamicsProvider(original.model,'residual-B',assetFile);
profile=makeHighFidelityPlantProfile('high-fidelity');
modes={'fullstate-tvlqr','geometric-D','geometric-D','geometric-D'};
variants={'tvlqr','d0','fixed','tvlqr'};
report=struct('id',{},'state',{},'metrics',{},'reason',{});
for k=1:numel(modes)
    id=[modes{k},'_',variants{k}]; folder=fullfile(outputDir,id);
    if ~isfolder(folder), mkdir(folder); end
    file=fullfile(folder,'result.mat');
    assert(resume || ~isfile(file),'ABFD:DiagnosticExists','Existing diagnostic needs explicit resume.');
    c=makeAbfdConfig(struct('modelMode','residual-B','residualFile',assetFile, ...
        'controllerMode',modes{k},'dVariant',variants{k},'plantProfile','high-fidelity','outputDir',folder));
    bundle=buildAbfdReferenceBundle(original.refs.hs,original.model,original.scene,original.disc,provider,c);
    assert(isequal(bundle.refs.q,original.refs.q) && isequal(bundle.refs.qd,original.refs.qd) && ...
        isequal(bundle.refs.qdd,original.refs.qdd),'ABFD:ComparisonReference','Reference changed.');
    row=struct('id',id,'state','running','metrics',[],'reason','');
    try
        if isfile(file)
            previous=load(file);
            assert(strcmp(abfdHash(previous.c),abfdHash(c)) && ...
                strcmp(previous.bundle.providerHash,provider.hash) && ...
                strcmp(abfdHash(previous.bundle.refs.hs),abfdHash(bundle.refs.hs)) && ...
                strcmp(abfdHash(previous.result.profile),abfdHash(profile)), ...
                'ABFD:CheckpointMismatch','Saved diagnostic differs from the fixed protocol.');
            result=previous.result;
        else
            result=runAbfdSimscapeCase(bundle,provider,c,profile);
            save(file,'result','bundle','c');
        end
        row.metrics=result.metrics; row.state='experiment-completed';
    catch exception
        row.state='failed'; row.reason=getReport(exception,'extended','hyperlinks','off');
        writeAbfdJson(fullfile(folder,'failure.json'),row);
    end
    report(k)=row;
    summary=struct('scope','pilot, training-pool task, same reference and true plant; B model used in schedule design', ...
        'assetHash',abfdFileHash(assetFile),'referenceHash',abfdHash(original.refs.hs), ...
        'plantHash',abfdHash(profile),'controllers',report);
    writeAbfdJson(fullfile(outputDir,'diagnostic.json'),summary);
end
end
