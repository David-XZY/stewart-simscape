function report=run_abfd_codesign_pilot(seedFile,outputDir,overrides,primalStartFile)
if nargin<3, overrides=struct(); end
if nargin<4, primalStartFile=''; end
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
assert(~isfile(fullfile(outputDir,'diagnostic.json')),'ABFD:PilotExists','Use a fresh codesign run directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
source=load(seedFile,'result'); seed=source.result;
assert(seed.passed && seed.bundle.publishPassed,'ABFD:CodesignSeed','A physically accepted complete HS seed is required.');
settings=struct('predictionProfile','sampled-hold-v2','plannerProfile','guarded-adaptive-v2', ...
    'controllerMode','geometric-D','designMode','codesign-A','solverMaxIterations',1500, ...
    'solverMaxCpuTime',600,'solverTolerance',1e-9,'outputDir',outputDir);
for field=fieldnames(overrides).', settings.(field{1})=overrides.(field{1}); end
settings.outputDir=outputDir; c=makeAbfdConfig(settings);
assert(strcmp(c.designMode,'codesign-A'),'ABFD:CodesignMode','This entry requires A optimization.');
profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
[p,c]=buildAbfdProviderForTask(seed.bundle.model,c,profile,seed.bundle.scene.q0);
report=struct('state','running','passed',false,'reason','','seedFile',seedFile, ...
    'seedHash',abfdFileHash(seedFile),'configHash',abfdHash(c),'implementationHash',abfdPlannerImplementationHash(), ...
    'modelMode',c.modelMode,'predictionProfile',c.predictionProfile, ...
    'scope','Trajectory/gain optimization on a frozen training-pool task; see explicit model and prediction profile; Simscape performance pending');
writeAbfdJson(fullfile(outputDir,'diagnostic.json'),report);
timer=tic;
try
    primalStart=[];
    if ~isempty(primalStartFile)
        restart=load(primalStartFile,'result');
        assert(isfield(restart,'result') && isequal(restart.result.disc,seed.disc), ...
            'ABFD:PrimalStartDiscretization','Explicit iterate must use the same full discretization as the accepted seed.');
        primalStart=restart.result.z;
        report.primalStartFile=primalStartFile; report.primalStartFileHash=abfdFileHash(primalStartFile);
        report.primalStartHash=abfdHash(primalStart);
        report.primalStartScope='Numerical initial guess only; no old multipliers, covariance factors or acceptance transferred';
        writeAbfdJson(fullfile(outputDir,'diagnostic.json'),report);
    end
    reference=reconstructIhsidContinuousReference(seed.bundle.refs.hs,p,c.sampleTime);
    certificate=validateAbfdReference(reference,seed.bundle.model,seed.bundle.scene,c);
    assert(certificate.passed,'ABFD:CodesignSeedRevalidation','Seed failed independent current-source dense acceptance.');
    result=run_A1_alternating_codesign(seed.bundle.model,seed.bundle.scene,p,c,seed,primalStart);
    report.state='completed'; report.passed=result.passed; report.objective=result.objective;
    report.codesign=result.codesign; report.history=result.history;
    if ~result.passed, report.reason=result.failureReason; end
    save(fullfile(outputDir,'result.mat'),'result','c','certificate');
catch exception
    report.state='failed'; report.reason=getReport(exception,'extended','hyperlinks','off');
end
report.seconds=toc(timer); writeAbfdJson(fullfile(outputDir,'diagnostic.json'),report);
end
