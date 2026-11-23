function report=run_abfd_sampled_controller_diagnostic(bundleFile,assetFile,outputDir)
% Matched/high-fidelity and fixed-gain B comparisons on the same pilot task.
setupAbfdResearch(); data=load(bundleFile,'bundle'); original=data.bundle;
ids={'analytic_matched','analytic_high_fidelity','B_fixed_gains','B_own_gains','B_geometric_tvlqr'};
report=struct('id',{},'state',{},'metrics',{},'reason',{},'predictionSeconds',{});
nominalGains=[];
for k=1:numel(ids)
    folder=fullfile(outputDir,ids{k}); if ~isfolder(folder), mkdir(folder); end
    assert(~isfile(fullfile(folder,'result.mat')),'ABFD:DiagnosticExists','Use a fresh output directory.');
    mode='analytic'; if k>=3, mode='residual-B'; end
    plant='high-fidelity'; if k==1, plant='matched'; end
    controller='fullstate-tvlqr'; if k==5, controller='geometric-D'; end
    c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','modelMode',mode, ...
        'residualFile',assetFile,'controllerMode',controller,'plantProfile',plant,'outputDir',folder));
    provider=buildDynamicsProvider(original.model,mode,assetFile);
    profile=makeHighFidelityPlantProfile(plant);
    bundle=buildAbfdReferenceBundle(original.refs.hs,original.model,original.scene,original.disc,provider,c);
    assert(isequal(bundle.refs.q,original.refs.q) && isequal(bundle.refs.qd,original.refs.qd) && ...
        isequal(bundle.refs.qdd,original.refs.qdd),'ABFD:ComparisonReference','The fixed reference changed.');
    row=struct('id',ids{k},'state','running','metrics',[],'reason','','predictionSeconds',NaN);
    try
        setup=prepareAbfdSimscape(bundle,provider,c,profile);
        cleanup=onCleanup(@()close_system(setup.modelName,0));
        if k==2, nominalGains=setup.runtime.design; end
        if k==3
            assert(~isempty(nominalGains),'ABFD:ComparisonGain','Nominal comparison gains unavailable.');
            setup.runtime.design=nominalGains; assignin('base','abfdRuntime',setup.runtime);
        end
        design=setup.runtime.design;
        simulation=sim(setup.modelName,'ReturnWorkspaceOutputs','on');
        log=readAbfdSimscapeLog(simulation.get('simout'),setup);
        save(fullfile(folder,'raw_simulation.mat'),'log','bundle','c','profile','design');
        clear cleanup;
        timer=tic; predicted=run_A0_trackability_metric(bundle.refs,provider,c,design);
        row.predictionSeconds=toc(timer);
        tube=struct('t',bundle.refs.t,'P',cat(3,predicted.P{:}));
        metrics=evaluateUnifiedMetrics(log,bundle,c,tube);
        result=struct('log',log,'metrics',metrics,'profile',profile,'prediction',predicted, ...
            'initialAppliedForce',setup.runtime.initialAppliedForce,'gainHash',abfdHash(design.schedule.K));
        save(fullfile(folder,'result.mat'),'result','bundle','c','tube');
        row.state='experiment-completed'; row.metrics=metrics;
    catch exception
        row.state='failed'; row.reason=getReport(exception,'extended','hyperlinks','off');
        if exist('cleanup','var'), clear cleanup; end
    end
    report(end+1)=row; %#ok<AGROW>
    writeAbfdJson(fullfile(outputDir,'diagnostic.json'),struct('scope','training-pool pilot; sampled-hold-v2; same reference and physical initialization', ...
        'assetHash',abfdFileHash(assetFile),'referenceHash',abfdHash(original.refs.hs),'cases',report));
end
end
