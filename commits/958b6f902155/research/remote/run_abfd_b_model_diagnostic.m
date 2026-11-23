function report=run_abfd_b_model_diagnostic(bundleFile,baselineResultFile,assetFile,outputDir)
% Fixed reference, logged baseline gains, physical plant and initial state.
% Only the controller-side wrench model changes to the actual trained B asset.
setupAbfdResearch(); data=load(bundleFile,'bundle'); b=data.bundle;
saved=load(baselineResultFile,'result'); baseline=saved.result;
c=makeAbfdConfig(struct('controllerMode','fullstate-tvlqr','plantProfile','high-fidelity', ...
    'modelMode','residual-B','residualFile',assetFile,'outputDir',outputDir));
p0=buildDynamicsProvider(b.model); nominal=c; nominal.modelMode='analytic'; nominal.residualFile='';
design=buildAbfdSchedule(b.refs,p0,nominal); gainError=0;
for k=1:numel(baseline.log.t)
    j=find(b.refs.t<=baseline.log.t(k)+1e-12,1,'last'); j=min(j,size(design.schedule.K,3));
    logged=reshape(baseline.log.diagnostics(k,73:144),6,12);
    expected=design.schedule.K(:,:,j);
    gainError=max(gainError,norm(logged-expected,'fro')/max(1,norm(expected,'fro')));
end
assert(gainError<1e-10,'ABFD:ComparisonGain','Baseline recorded gains do not match the fixed schedule.');
profile=makeHighFidelityPlantProfile('high-fidelity');
assert(strcmp(abfdHash(profile),abfdHash(baseline.profile)),'ABFD:ComparisonPlant','Physical plant differs from baseline.');
p=buildDynamicsProvider(b.model,'residual-B',assetFile);
bundle=buildAbfdReferenceBundle(b.refs.hs,b.model,b.scene,b.disc,p,c);
assert(isequal(bundle.refs.q,b.refs.q) && isequal(bundle.refs.qd,b.refs.qd) && isequal(bundle.refs.qdd,b.refs.qdd), ...
    'ABFD:ComparisonReference','Reference coordinates or derivatives changed.');
setup=prepareAbfdSimscape(bundle,p,c,profile);
cleanup=onCleanup(@()close_system(setup.modelName,0));
initialForceError=norm(setup.runtime.initialAppliedForce-b.refs.Fleg(:,1));
assert(initialForceError<1e-8,'ABFD:ComparisonInitialization','Physical actuator preload differs from baseline.');
setup.runtime.design=design; assignin('base','abfdRuntime',setup.runtime);
simulation=sim(setup.modelName,'ReturnWorkspaceOutputs','on'); simout=simulation.get('simout');
log=readAbfdSimscapeLog(simout,setup); metrics=evaluateUnifiedMetrics(log,bundle,c,[]);
report=struct('scope','pilot B model; training-pool task, fixed measured baseline gains and same true plant', ...
    'assetHash',abfdFileHash(assetFile),'assetScope',p.network.trainingScope, ...
    'baselineGainRelativeError',gainError,'baselineMetrics',baseline.metrics,'modelMetrics',metrics, ...
    'baselineInitialForceError',initialForceError,'initialForceHash',abfdHash(setup.runtime.initialAppliedForce), ...
    'referenceHash',abfdHash(struct('q',b.refs.q,'qd',b.refs.qd,'qdd',b.refs.qdd)), ...
    'gainHash',abfdHash(design.schedule.K),'plantHash',abfdHash(profile));
save(fullfile(outputDir,'diagnostic.mat'),'report','log','bundle','profile');
writeAbfdJson(fullfile(outputDir,'diagnostic.json'),report);
clear cleanup;
end
