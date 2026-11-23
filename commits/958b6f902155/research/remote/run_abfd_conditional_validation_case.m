function report=run_abfd_conditional_validation_case(originalRoot,taskId,asset,temporal,calibration,out,expectedSplit,startupInertia)
% Frozen validation or explicitly requested test evaluation; no fitting here.
if nargin<7, expectedSplit='validation'; end
if nargin<8, startupInertia=false; end
validateattributes(startupInertia,{'logical'},{'scalar'});
expectedSplit=validatestring(expectedSplit,{'validation','test'});
setupAbfdResearch();
assert(~isfile(fullfile(out,'status.json')),'ABFD:ValidationExists','Use a fresh output directory.');
if ~isfolder(out), mkdir(out); end
tasksFile=fullfile(originalRoot,'B_tasks_frozen.json'); tasks=jsondecode(fileread(tasksFile));
task=tasks(strcmp({tasks.taskId},taskId));
assert(isscalar(task) && strcmp(task.split,expectedSplit),'ABFD:ValidationSplit','Task differs from the explicitly selected frozen evaluation split.');
sourceFile=fullfile(originalRoot,'B0',taskId,'result.mat'); saved=load(sourceFile,'result');
bundle=saved.result.planning.bundle; seedFile=fullfile(out,'frozen_reference.mat'); save(seedFile,'bundle');
manifest=jsondecode(fileread(fullfile(originalRoot,'B0',taskId,'manifest.json')));
c=manifest.config; c.modelMode='residual-B'; c.residualFile=asset; c.staticBalanceFile=calibration;
if startupInertia, c.inertiaCalibrationFile=calibration; end
c.controllerMode='geometric-D'; c.dVariant='tvlqr'; c.designMode='sequential';
c.plannerMode='ihsid'; c.trajectoryFile=seedFile; c.outputDir=fullfile(out,'experiment');
c.predictionProfile='actuator-var-v3'; c.temporalFile=temporal; c.allowPilotTemporal=true;
c=makeAbfdConfig(c);
report=struct('state','running','passed',false,'taskId',taskId,'split',task.split, ...
    'sourceHash',abfdFileHash(sourceFile),'tasksHash',abfdFileHash(tasksFile), ...
    'staticAssetHash',abfdFileHash(asset),'temporalAssetHash',abfdFileHash(temporal), ...
    'calibrationHash',abfdFileHash(calibration), ...
    'testEvaluationOnly',strcmp(expectedSplit,'test'), ...
    'scope','Frozen evaluation-task B+D closed loop with explicit startup observation and unchanged physical plant; pilot uncertainty; no fitting/model selection or full ABFD acceptance');
report.startupInertia=startupInertia;
if startupInertia
    report.scope='B+D with additional independent startup force/acceleration inertia and static observations; frozen physical plant and reference; pilot, not formal fixed-gain B ablation or full ABFD acceptance';
end
writeAbfdJson(fullfile(out,'status.json'),report);
try
    timer=tic; result=runAbfdExperiment(task,c); report.seconds=toc(timer);
    report.state='complete'; report.metrics=result.metrics;
    report.controllerPassed=result.metrics.engineeringPassed && result.metrics.budgetPassed && result.metrics.onlinePassed;
    report.tubePassed=result.metrics.tubeCoverage95>=.95;
    report.passed=report.controllerPassed && report.tubePassed;
    writeAbfdJson(fullfile(out,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    writeAbfdJson(fullfile(out,'status.json'),report); rethrow(e);
end
end
