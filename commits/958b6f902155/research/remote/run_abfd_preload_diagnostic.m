function report=run_abfd_preload_diagnostic(bundleFile,assetFile,preloadFile,outputDir)
% Same physical equilibrium initialization for every compared control model.
setupAbfdResearch(); saved=load(bundleFile,'bundle'); original=saved.bundle;
ids={'analytic_high_fidelity','B_fixed_gains','B_geometric_tvlqr'};
report=struct('id',{},'state',{},'reason',{},'metrics',{},'differencedMetrics',{},'initialLegAcceleration',{});
nominalGains=[];
for k=1:numel(ids)
    folder=fullfile(outputDir,ids{k}); if ~isfolder(folder), mkdir(folder); end
    assert(~isfile(fullfile(folder,'raw_simulation.mat')),'ABFD:DiagnosticExists','Use a fresh output directory.');
    mode='analytic'; controller='fullstate-tvlqr';
    if k>=2, mode='residual-B'; end
    if k==3, controller='geometric-D'; end
    profile=applyAbfdStaticPreload(makeHighFidelityPlantProfile('high-fidelity'),original,preloadFile);
    c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','modelMode',mode,'residualFile',assetFile, ...
        'controllerMode',controller,'plantProfile','high-fidelity','plantOverrides',struct('initialAppliedForce',profile.initialAppliedForce),'outputDir',folder));
    p=buildDynamicsProvider(original.model,mode,assetFile,false,original.refs.q(:,1)+profile.initialPoseOffset);
    bundle=buildAbfdReferenceBundle(original.refs.hs,original.model,original.scene,original.disc,p,c);
    row=struct('id',ids{k},'state','running','reason','','metrics',[], 'differencedMetrics',[],'initialLegAcceleration',[]);
    try
        setup=prepareAbfdSimscape(bundle,p,c,profile); cleanup=onCleanup(@()close_system(setup.modelName,0));
        if k==1, nominalGains=setup.runtime.design; end
        if k==2, setup.runtime.design=nominalGains; assignin('base','abfdRuntime',setup.runtime); end
        design=setup.runtime.design; interface=installAbfdAccelerationProbe(setup.modelName);
        simulation=sim(setup.modelName,'ReturnWorkspaceOutputs','on');
        differencedLog=readAbfdSimscapeLog(simulation.get('simout'),setup);
        log=readAbfdDirectAcceleration(simulation,differencedLog);
        assert(norm(log.actualForce(:,1)-profile.initialAppliedForce)<1e-8,'ABFD:ComparisonPreload','Physical preload changed.');
        save(fullfile(folder,'raw_simulation.mat'),'log','differencedLog','bundle','c','profile','design','interface');
        clear cleanup;
        row.metrics=evaluateUnifiedMetrics(log,bundle,c,[]);
        row.differencedMetrics=evaluateUnifiedMetrics(differencedLog,bundle,c,[]);
        initial=computeLegKinematics(log.q(:,1),log.qd(:,1),log.qdd(:,1),bundle.model);
        row.initialLegAcceleration=initial.Ldd; row.state='experiment-completed';
        result=struct('log',log,'metrics',row.metrics,'profile',profile,'design',design, ...
            'initialAppliedForce',profile.initialAppliedForce,'gainHash',abfdHash(design.schedule.K));
        save(fullfile(folder,'result.mat'),'result','bundle','c','differencedLog');
    catch e
        row.state='failed'; row.reason=getReport(e,'extended','hyperlinks','off');
        if exist('cleanup','var'), clear cleanup; end
    end
    report(end+1)=row; %#ok<AGROW>
    writeAbfdJson(fullfile(outputDir,'diagnostic.json'),struct('scope','Same measured physical preload across analytic/B/D controls; direct and differenced acceleration both reported; training-pool diagnosis', ...
        'assetHash',abfdFileHash(assetFile),'preloadHash',abfdFileHash(preloadFile),'referenceHash',abfdHash(original.refs.hs),'cases',report));
end
end
