function report=run_abfd_actuator_nlp_probe(seedFile,assetFile,temporalFile,outputDir)
% Full physical horizon and all constraints; measures computational viability.
% Does not solve or accept a trajectory, and never freezes covariance values.
setupAbfdResearch(); import casadi.*
assert(~isfile(fullfile(outputDir,'probe.json')),'ABFD:ProbeExists','Use a fresh probe output.');
if ~isfolder(outputDir), mkdir(outputDir); end
report=struct('state','starting','passed',false,'stage','loading', ...
    'seedHash',abfdFileHash(seedFile),'assetHash',abfdFileHash(assetFile), ...
    'temporalHash',abfdFileHash(temporalFile),'implementationHash',abfdPlannerImplementationHash(), ...
    'scope','Full-horizon candidate-dependent actuator/VAR NLP build/evaluation/AD probe; optimization and Simscape acceptance are separate');
publish();
try
    saved=load(seedFile,'result'); seed=saved.result;
    assert(seed.passed,'ABFD:ProbeSeed','Probe needs an independently accepted physical seed.');
    m=seed.bundle.model; s=seed.bundle.scene; d=seed.disc;
    c=makeAbfdConfig(struct('predictionProfile','actuator-var-v3','controllerMode','geometric-D', ...
        'modelMode','residual-B','residualFile',assetFile,'temporalFile',temporalFile, ...
        'allowPilotTemporal',true,'plantProfile','high-fidelity','designMode','codesign-A'));
    p=buildDynamicsProvider(m,'residual-B',assetFile);
    timer=tic; refs=reconstructIhsidContinuousReference(seed.bundle.refs.hs,p,c.sampleTime);
    design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
    report.referenceSeconds=toc(timer); report.horizon=refs.t(end); report.hsPoints=d.numNodes+d.numMidpoints;
    research=struct('provider',p,'config',c,'design',design,'center',[],'buildOnly',true);
    report.state='running'; report.stage='building-nlp'; publish();
    timer=tic; nlp=buildCasadiTrackableIHSNLP(m,s,d,seed.initial,struct(),research);
    report.buildSeconds=toc(timer); report.sizes=nlp.sizes; report.stage='evaluating-nlp'; publish();
    timer=tic; [cost,eq,g]=nlp.eval(seed.z); v=full([cost;eq;g]);
    assert(all(isfinite(v)),'ABFD:ProbeNonfinite','Full-horizon expressions are not finite.');
    report.evaluationSeconds=toc(timer); report.initialCost=full(cost);
    report.initialMaxIneq=max(0,max(full(g))); report.initialMaxEq=max(abs(full(eq)));
    report.stage='building-derivative'; publish();
    direction=sin((1:numel(seed.z)).'); direction=direction/norm(direction);
    timer=tic; directional=Function('full_actuator_nlp_direction',{nlp.z}, ...
        {jtimes([nlp.J;nlp.gEq;nlp.cIneq],nlp.z,direction)});
    report.derivativeBuildSeconds=toc(timer); report.stage='evaluating-derivative'; publish();
    timer=tic; derivative=full(directional(seed.z)); report.derivativeEvaluationSeconds=toc(timer);
    assert(all(isfinite(derivative)),'ABFD:ProbeDerivative','Nonfinite full-horizon AD derivative.');
    report.derivativeNorm=norm(derivative); report.passed=true; report.state='complete'; report.stage='complete'; publish();
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message; publish(); rethrow(e);
end
    function publish()
        writeAbfdJson(fullfile(outputDir,'probe.json'),report);
    end
end
