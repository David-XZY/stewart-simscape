function report=run_abfd_mean_prediction_diagnostic(sourceRoot,assetFile,outputDir)
setupAbfdResearch();
ids={'analytic_matched','analytic_high_fidelity','B_fixed_gains','B_geometric_tvlqr'};
report=struct('id',{},'state',{},'reason',{},'translationPredictionRms',{}, ...
    'translationPredictionPeak',{},'predictedTrackingPeak',{},'actualTrackingPeak',{}, ...
    'forcePredictionRms',{},'integrationConvergencePeak',{},'seconds',{});
for k=1:numel(ids)
    row=struct('id',ids{k},'state','running','reason','','translationPredictionRms',NaN, ...
        'translationPredictionPeak',NaN,'predictedTrackingPeak',NaN,'actualTrackingPeak',NaN, ...
        'forcePredictionRms',[],'integrationConvergencePeak',NaN,'seconds',NaN);
    try
        data=load(fullfile(sourceRoot,ids{k},'result.mat'));
        mode='analytic'; if k>=3, mode='residual-B'; end
        provider=buildDynamicsProvider(data.bundle.model,mode,assetFile);
        log=data.result.log; runtime=struct('bundle',data.bundle,'provider',provider, ...
            'config',data.c,'design',data.result.prediction.design,'initialState',[log.q(:,1);log.qd(:,1)]);
        options=struct('queryTime',log.t,'initialForce',data.result.initialAppliedForce);
        predicted=predictAbfdClosedLoopMean(runtime,provider,data.result.profile,options);
        if k==1
            options.maxStep=.0005;
            fine=predictAbfdClosedLoopMean(runtime,provider,data.result.profile,options);
            row.integrationConvergencePeak=max(vecnorm(fine.x(1:3,:)-predicted.x(1:3,:)));
            assert(row.integrationConvergencePeak<1e-7,'ABFD:MeanConvergence', ...
                'Matched mean prediction did not converge under step halving.');
        end
        reference=interp1(data.bundle.refs.t,data.bundle.refs.q.',log.t).';
        difference=predicted.x(1:3,:)-log.q(1:3,:);
        row.translationPredictionRms=sqrt(mean(sum(difference.^2,1)));
        row.translationPredictionPeak=max(vecnorm(difference));
        row.predictedTrackingPeak=max(vecnorm(predicted.x(1:3,:)-reference(1:3,:)));
        row.actualTrackingPeak=max(vecnorm(log.q(1:3,:)-reference(1:3,:)));
        row.forcePredictionRms=sqrt(mean((predicted.actualForce-log.actualForce).^2,2));
        row.seconds=predicted.elapsedSeconds; row.state='completed';
        folder=fullfile(outputDir,ids{k}); if ~isfolder(folder), mkdir(folder); end
        save(fullfile(folder,'prediction.mat'),'predicted','row','options');
    catch exception
        row.state='failed'; row.reason=getReport(exception,'extended','hyperlinks','off');
    end
    report(end+1)=row; %#ok<AGROW>
    writeAbfdJson(fullfile(outputDir,'diagnostic.json'),struct('scope', ...
        'same pilot logs; known actuator and control-side rigid/B model mean prediction only', ...
        'assetHash',abfdFileHash(assetFile),'cases',report));
end
end
