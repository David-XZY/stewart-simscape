function report=run_B3_calibrate_validation_tube(validationRoot,assetFile,temporalFile,outputDir)
% Calibrate only residual/VAR covariance using frozen validation executions.
% Initial state/process covariance, mean, controller and reference stay fixed.
% This is validation calibration, never independent coverage acceptance.
setupAbfdResearch();
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:TubeCalibrationExists','Use a fresh calibration directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
sourceFile=fullfile(validationRoot,'status.json'); source=jsondecode(fileread(sourceFile));
assert(strcmp(source.state,'complete') && source.completed==source.expected && source.expected==4 && ...
    strcmp(source.assetHash,abfdFileHash(assetFile)) && strcmp(source.temporalHash,abfdFileHash(temporalFile)), ...
    'ABFD:TubeCalibrationSource','Complete frozen validation executions and their exact assets are required.');
parent=jsondecode(fileread(temporalFile));
assert(~isfield(parent,'closedLoopValidation') && strcmp(parent.status,'calibrated-pilot') && ...
    strcmp(parent.conditioning.kind,'independent-initial-inertia-static-v2'), ...
    'ABFD:TubeCalibrationProtocol','Use the original startup-conditioned pilot once, not a recursively calibrated model.');
report=struct('state','running','passed',false,'completed',0,'expected',numel(source.cases), ...
    'validationSourceHash',abfdFileHash(sourceFile),'staticAssetHash',abfdFileHash(assetFile), ...
    'parentTemporalHash',abfdFileHash(temporalFile),'implementationHash',abfdPlannerImplementationHash(), ...
    'calibrationImplementationHash',abfdFileHash([mfilename('fullpath'),'.m']),'cases',[], ...
    'scope','Validation-only joint 12D ellipsoid calibration of VAR uncertainty; no changed controls, no test data, no new Simscape execution');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
cases=cell(1,numel(source.cases)); evidence=struct([]);
try
    for k=1:numel(source.cases)
        row=source.cases(k);
        assert(strcmp(row.split,'validation') && strcmp(row.state,'complete') && row.controllerPassed && row.startupInertia, ...
            'ABFD:TubeCalibrationSplit','Only completed validation executions with accepted physical control may calibrate uncertainty.');
        folder=fullfile(validationRoot,row.taskId,'experiment'); resultFile=fullfile(folder,'result.mat');
        saved=load(resultFile,'result'); result=saved.result; bundle=result.planning.bundle;
        manifest=jsondecode(fileread(fullfile(folder,'manifest.json'))); c=manifest.config;
        assert(strcmp(abfdFileHash(c.residualFile),abfdFileHash(assetFile)) && ...
            strcmp(abfdFileHash(c.temporalFile),abfdFileHash(temporalFile)), ...
            'ABFD:TubeCalibrationAsset','Execution manifest refers to different static or temporal weights.');
        plain=rmfield(result.profile,'initialAppliedForce');
        if isfield(c.plantOverrides,'initialAppliedForce'), c.plantOverrides=rmfield(c.plantOverrides,'initialAppliedForce'); end
        [provider,c,profile]=buildAbfdProviderForTask(bundle.model,c,plain,bundle.refs.q(:,1));
        runtime=makeAbfdRuntime(bundle,provider,c);
        runtime.initialState=[bundle.refs.q(:,1)+profile.initialPoseOffset;zeros(6,1)];
        options=struct('queryTime',bundle.refs.t,'temporalFile',temporalFile,'allowPilotTemporal',true);
        if isfield(c,'predictionMaxStep'), options.maxStep=c.predictionMaxStep; end
        total=predictAbfdClosedLoopMean(runtime,provider,profile,options);
        noiseOnly=runtime; noiseOnly.config.initialCovariance=zeros(12); noiseOnly.config.processSpectralDensity=zeros(12);
        noise=predictAbfdClosedLoopMean(noiseOnly,provider,profile,options);
        meanError=norm(total.x-noise.x,'fro'); commandError=norm(total.commands-noise.commands,'fro');
        assert(meanError<1e-12 && commandError<1e-12,'ABFD:TubeCalibrationMean','Covariance decomposition changed mean or command.');
        log=result.log; centers=interp1(total.t,total.x.',log.t(:)).';
        delta=[log.q;log.qd]-centers;
        indices=arrayfun(@(time)find(total.t<=time,1,'last'),log.t(:));
        noiseP=noise.P(:,:,indices); baseP=total.P(:,:,indices)-noiseP;
        symmetry=0; minimum=0; covarianceNorm=0;
        for j=1:numel(indices)
            for matrix={noiseP(:,:,j),baseP(:,:,j)}
                P=matrix{1}; symmetry=max(symmetry,norm(P-P.','fro')); minimum=min(minimum,min(eig((P+P.')/2)));
                covarianceNorm=max(covarianceNorm,norm(P,'fro'));
            end
        end
        roundoff=128*eps*max(1,covarianceNorm);
        assert(symmetry<roundoff && minimum>=-roundoff,'ABFD:TubeCovarianceDecomposition', ...
            'Covariance decomposition is not PSD to machine precision.');
        item=struct('delta',delta,'baseP',baseP,'noiseP',noiseP,'runtime',runtime,'provider',provider, ...
            'profile',profile,'options',options,'total',total,'noise',noise,'log',log,'bundle',bundle,'config',c);
        [coverage,scores]=coverageAtScale(item,1);
        assert(abs(coverage-row.metrics.tubeCoverage95)<1e-12,'ABFD:TubeCalibrationReplay', ...
            'Recomputed joint ellipsoid coverage differs from the original execution.');
        record=struct('taskId',row.taskId,'split',row.split,'samples',numel(indices), ...
            'sourceHash',abfdFileHash(resultFile),'originalCoverage',coverage,'meanReplayDifference',meanError, ...
            'commandReplayDifference',commandError,'minimumCovarianceEigenvalue',minimum,'roundoffBound',roundoff, ...
            'initialScoreMaximum',max(scores));
        cases{k}=item;
        if isempty(evidence), evidence=record; else, evidence(k)=record; end
        report.completed=k; report.cases=evidence; writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    target=.95; lower=1; upper=1;
    while ~all(cellfun(@(item)coverageAtScale(item,upper)>=target,cases))
        upper=upper*2;
        assert(upper<=1024,'ABFD:TubeCalibrationUnbounded','Validation coverage needs an excessive or ineffective uncertainty scale.');
    end
    if upper>1
        for j=1:40
            middle=(lower+upper)/2;
            if all(cellfun(@(item)coverageAtScale(item,middle)>=target,cases)), upper=middle; else, lower=middle; end
        end
    end
    multiplier=upper*(1+1e-8); calibrated=parent;
    calibrated.processCovariance=multiplier*parent.processCovariance;
    calibrated.initialCovariance=multiplier*parent.initialCovariance;
    calibrated.innovationValidationScale=parent.validationScale;
    calibrated.validationScale=parent.validationScale*multiplier;
    calibrated.preClosedLoopEvaluation=parent.evaluation; calibrated=rmfield(calibrated,'evaluation');
    calibrated.closedLoopValidation=struct('kind','validation-closed-loop-ellipsoid-v1','multiplier',multiplier, ...
        'target',target,'chiSquareThreshold',21.0260698175,'metricCovarianceFloor',1e-12, ...
        'sourceStatusHash',report.validationSourceHash,'parentTemporalHash',report.parentTemporalHash, ...
        'cases',evidence,'implementationHash',report.calibrationImplementationHash, ...
        'selection','Smallest common VAR covariance multiplier giving at least 95 percent time-sample joint ellipsoid coverage in every frozen validation task', ...
        'scope','Additional closed-loop validation calibration only; state/process prior and mean unchanged; no test data');
    calibrated.scope='Pilot temporal uncertainty after explicit validation-trajectory joint-state calibration; independent held-out acceptance remains pending';
    calibrated.limitations=[parent.limitations,' Validation time samples are correlated; these fitted coverage values are not an independent probability guarantee.'];
    outputFile=fullfile(outputDir,'temporal_model.json'); writeAbfdJson(outputFile,calibrated);
    for k=1:numel(cases)
        item=cases{k}; item.options.temporalFile=outputFile;
        actual=predictAbfdClosedLoopMean(item.runtime,item.provider,item.profile,item.options);
        expected=item.total.P+(multiplier-1)*item.noise.P;
        relativeError=norm(actual.P(:)-expected(:))/max(norm(expected(:)),eps);
        assert(relativeError<1e-8 && norm(actual.x-item.total.x,'fro')<1e-10 && ...
            norm(actual.commands-item.total.commands,'fro')<1e-9,'ABFD:TubeScaleLinearity');
        tube=struct('t',actual.t,'P',actual.P,'mean',actual.x);
        metrics=evaluateUnifiedMetrics(item.log,item.bundle,item.config,tube);
        assert(metrics.tubeCoverage95>=target,'ABFD:TubeCalibrationVerification','Scaled predictor did not meet the validation-only target.');
        evidence(k).calibratedCoverage=metrics.tubeCoverage95; evidence(k).covarianceScaleRelativeError=relativeError;
        evidence(k).controlMetrics=metrics;
        save(fullfile(outputDir,[evidence(k).taskId,'_prediction.mat']),'actual','metrics');
    end
    report.cases=evidence; report.multiplier=multiplier; report.temporalHash=abfdFileHash(outputFile);
    report.state='complete'; report.passed=true; writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    report.detail=getReport(e,'extended','hyperlinks','off'); writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
function [coverage,scores]=coverageAtScale(item,multiplier)
scores=zeros(1,size(item.delta,2));
for k=1:numel(scores)
    P=item.baseP(:,:,k)+multiplier*item.noiseP(:,:,k)+1e-12*eye(12);
    L=chol((P+P.')/2,'lower'); whitened=L\item.delta(:,k); scores(k)=sum(whitened.^2);
end
coverage=mean(scores<=21.0260698175);
end
