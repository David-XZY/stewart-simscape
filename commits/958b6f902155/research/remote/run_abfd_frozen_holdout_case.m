function report=run_abfd_frozen_holdout_case(tasksFile,expectedTasksHash,referenceDir,taskId,asset,temporal,templateConfig,out)
% Independent held-out execution under the already frozen startup protocol.
% Reference selection was analytic-only; never choose a replacement based on
% B/D performance. All planning, observation and simulation failures count.
setupAbfdResearch();
assert(~isfile(fullfile(out,'status.json')),'ABFD:HoldoutExists','Use a fresh execution directory.');
assert(strcmp(abfdFileHash(tasksFile),expectedTasksHash),'ABFD:HoldoutFreeze','Pre-training task freeze changed.');
tasks=readAbfdJsonUtf8(tasksFile);
assert(numel(tasks)==12 && numel(unique({tasks.taskId}))==12 && all(strcmp({tasks.split},'test')), ...
    'ABFD:HoldoutMembership','All twelve frozen test tasks must remain in the denominator.');
task=tasks(strcmp({tasks.taskId},taskId));
assert(isscalar(task),'ABFD:HoldoutTask','Task is absent or duplicated in the frozen manifest.');
referenceStatus=fullfile(referenceDir,'status.json');
selection=jsondecode(fileread(referenceStatus));
assert(strcmp(selection.state,'complete') && strcmp(selection.taskId,taskId) && ...
    strcmp(selection.tasksHash,expectedTasksHash), ...
    'ABFD:HoldoutReference','Reference generation is unfinished or belongs to a different task.');
if ~isfolder(out), mkdir(out); end
report=struct('state','running','stage','reference','passed',false,'taskId',taskId,'split','test', ...
    'tasksHash',expectedTasksHash,'taskHash',abfdHash(task),'referenceStatusHash',abfdFileHash(referenceStatus), ...
    'staticAssetHash',abfdFileHash(asset),'temporalAssetHash',abfdFileHash(temporal), ...
    'templateConfigHash',abfdHash(templateConfig),'implementationHash',abfdPlannerImplementationHash(), ...
    'runnerHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
    'scope','Independent frozen B+D test with pre-task force/acceleration inertia and static observations; no fitting or test-based reference/model selection; not formal fixed-gain B ablation or full ABFD acceptance');
writeAbfdJson(fullfile(out,'status.json'),report); timer=tic;
try
    if ~selection.passed
        report.state='complete'; report.stage='planning-rejected'; report.planningPassed=false;
        report.reason='All ordered analytic starts failed; retained as an unsuccessful held-out task.';
        report.totalSeconds=toc(timer); writeAbfdJson(fullfile(out,'status.json'),report); return;
    end
    referenceFile=fullfile(referenceDir,'reference.mat');
    assert(strcmp(abfdFileHash(referenceFile),selection.referenceHash), ...
        'ABFD:HoldoutReferenceHash','Selected reference changed after dense acceptance.');
    saved=load(referenceFile,'bundle','task'); bundle=saved.bundle;
    assert(bundle.publishPassed && strcmp(abfdHash(saved.task),selection.taskHash), ...
        'ABFD:HoldoutReferenceContext','The accepted reference must retain its byte-bound generation task.');
    [bundle,report.labelRestoration]=restoreAbfdFrozenTaskLabels(bundle,saved.task,task);
    report.planningPassed=true; report.referenceHash=selection.referenceHash;
    c=templateConfig; c.modelMode='residual-B'; c.residualFile=asset;
    c.controllerMode='geometric-D'; c.dVariant='tvlqr'; c.designMode='sequential';
    c.plannerMode='ihsid'; c.trajectoryFile=referenceFile; c.plantProfile='high-fidelity';
    c.plantOverrides=task.plantOverrides; c.outputDir=fullfile(out,'experiment');
    c.predictionProfile='actuator-var-v3'; c.temporalFile=temporal; c.allowPilotTemporal=true;
    calibrationDir=fullfile(out,'startup_observation');
    c.staticBalanceFile=fullfile(calibrationDir,'status.json'); c.inertiaCalibrationFile=c.staticBalanceFile;
    c=makeAbfdConfig(c); writeAbfdJson(fullfile(out,'frozen_execution_config.json'),c);
    profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
    observationConfig=makeAbfdConfig(struct('controllerMode','geometric-D','dVariant','tvlqr', ...
        'plantProfile','high-fidelity','plantOverrides',task.plantOverrides,'outputDir',calibrationDir));
    bundle=buildAbfdReferenceBundle(bundle.refs.hs,task.model,task.scene,bundle.disc, ...
        buildDynamicsProvider(task.model),observationConfig);
    canonicalFile=fullfile(out,'canonical_reference.mat'); save(canonicalFile,'bundle');
    c.trajectoryFile=canonicalFile; report.canonicalReferenceHash=abfdFileHash(canonicalFile);
    writeAbfdJson(fullfile(out,'frozen_execution_config.json'),c);
    report.stage='startup-observation'; writeAbfdJson(fullfile(out,'status.json'),report);
    observationTimer=tic; calibrateAbfdStaticPreload(bundle,observationConfig,profile,calibrationDir);
    measurement=readMeasuredInertiaRegression(c.inertiaCalibrationFile,bundle,profile);
    report.calibrationSeconds=toc(observationTimer); report.calibrationHash=abfdFileHash(c.inertiaCalibrationFile);
    report.rigidIdentificationResidual=measurement.relativeRigidResidual;
    report.stage='simscape'; writeAbfdJson(fullfile(out,'status.json'),report);
    experimentTimer=tic; result=runAbfdExperiment(task,c); report.experimentSeconds=toc(experimentTimer);
    assert(strcmp(report.staticAssetHash,abfdFileHash(asset)) && strcmp(report.temporalAssetHash,abfdFileHash(temporal)), ...
        'ABFD:HoldoutAssets','Frozen network or uncertainty changed during testing.');
    report.state='complete'; report.stage='evaluated'; report.metrics=result.metrics;
    report.controllerPassed=result.metrics.engineeringPassed && result.metrics.budgetPassed && result.metrics.onlinePassed;
    report.tubePassed=result.metrics.tubeCoverage95>=.95;
    report.passed=report.controllerPassed && report.tubePassed; report.totalSeconds=toc(timer);
    writeAbfdJson(fullfile(out,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message; report.totalSeconds=toc(timer);
    writeAbfdJson(fullfile(out,'status.json'),report); rethrow(e);
end
end
