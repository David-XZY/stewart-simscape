function report=run_abfd_conditional_prediction_benchmark(sourceRoot,assetFile,temporalFile,calibrationFile,outputDir)
% Replay the unchanged control mean against frozen, directly sensed Simscape logs.
% This evaluates a new covariance model without claiming a new closed-loop run.
setupAbfdResearch();
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:ConditionalBenchmarkExists','Use a fresh output directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
ids={'calibrated_B_fixed_gains','calibrated_B_geometric_tvlqr'};
implementation=abfdHash(struct('planner',abfdPlannerImplementationHash(), ...
    'predictor',abfdFileHash(which('predictAbfdClosedLoopMean')), ...
    'benchmark',abfdFileHash([mfilename('fullpath'),'.m'])));
report=struct('state','running','completed',0,'expected',numel(ids),'implementationHash',implementation, ...
    'staticAssetHash',abfdFileHash(assetFile),'temporalAssetHash',abfdFileHash(temporalFile), ...
    'calibrationHash',abfdFileHash(calibrationFile), ...
    'scope','New pilot conditional covariance against frozen Simscape logs; existing controller/plant/reference unchanged; no full ABFD acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report); rows=struct([]);
try
    for k=1:numel(ids)
        sourceFile=fullfile(sourceRoot,ids{k},'raw_simulation.mat'); source=load(sourceFile);
        c=source.c; c.predictionProfile='actuator-var-v3'; c.temporalFile=temporalFile; c.allowPilotTemporal=true;
        c=makeAbfdConfig(c); profile=source.profile; plain=rmfield(profile,'initialAppliedForce');
        base=buildDynamicsProvider(source.bundle.model,'residual-B',assetFile,false,source.biasCalibration.pose);
        provider=conditionResidualMeanOnStaticBalance(base,source.bundle,plain,calibrationFile);
        assert(strcmp(provider.hash,source.bundle.providerHash) && ...
            strcmp(abfdHash(provider.biasCalibration),abfdHash(source.biasCalibration)), ...
            'ABFD:ConditionalReplayProvider','Frozen controller mean differs from the reconstructed provider.');
        runtime=struct('bundle',source.bundle,'provider',provider,'config',c,'design',source.design, ...
            'initialState',[source.log.q(:,1);source.log.qd(:,1)]);
        options=struct('queryTime',source.log.t,'temporalFile',temporalFile,'allowPilotTemporal',true,'maxStep',.001);
        prediction=predictAbfdClosedLoopMean(runtime,provider,profile,options);
        tube=struct('t',prediction.t,'P',prediction.P,'mean',prediction.x);
        metrics=evaluateUnifiedMetrics(source.log,source.bundle,c,tube);
        error=[source.log.q;source.log.qd]-prediction.x;
        folder=fullfile(outputDir,ids{k}); if ~isfolder(folder), mkdir(folder); end
        save(fullfile(folder,'prediction.mat'),'prediction','metrics','c');
        row=struct('id',ids{k},'sourceHash',abfdFileHash(sourceFile),'providerHash',provider.hash, ...
            'gainHash',abfdHash(source.design.schedule.K),'metrics',metrics, ...
            'meanTranslationPeakError',max(abs(error(1:3,:)),[],'all'), ...
            'meanStateRmse',sqrt(mean(error.^2,2)),'predictionSeconds',prediction.elapsedSeconds, ...
            'samples',numel(prediction.t),'maxStep',options.maxStep);
        if isempty(rows), rows=row; else, rows(k)=row; end
        report.completed=k; report.cases=rows;
        writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    report.state='complete'; writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
