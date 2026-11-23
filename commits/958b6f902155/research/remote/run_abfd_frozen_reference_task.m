function report=run_abfd_frozen_reference_task(tasksFile,index,outputDir)
% Independent analytic reference for a frozen held-out task. Try ordered starts
% until the first dense-feasible plan; never choose a reference using B/D error.
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
tasks=readAbfdJsonUtf8(tasksFile); task=tasks(index);
assert(all(strcmp({tasks.split},'test')) && strcmp(task.split,'test'), ...
    'ABFD:HeldoutManifest','Reference generation requires a frozen test-only manifest.');
if ~isfolder(outputDir), mkdir(outputDir); end
c=makeAbfdConfig(struct('plannerProfile','guarded-adaptive-v2','solverMaxIterations',1500, ...
    'solverMaxCpuTime',600,'candidateCount',8,'outputDir',outputDir));
provider=buildDynamicsProvider(task.model);
disc=buildTwoPhaseIHSDiscretization(task.scene,task.scene.phase.numIntervalsApproach,task.scene.phase.numIntervalsInsertion);
report=struct('state','running','passed',false,'taskId',task.taskId,'split','test', ...
    'tasksHash',abfdFileHash(tasksFile),'taskHash',abfdHash(task),'maximumStarts',8,'attempts',[], ...
    'implementationHash',abfdPlannerImplementationHash(),'generatorHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
    'scope','Frozen held-out analytic reference: first feasible of at most eight ordered starts; no B weights or execution outcomes used; unattempted starts are not successful attempts');
rows=struct([]); timer=tic;
for start=1:c.candidateCount
    checkpoint=fullfile(outputDir,sprintf('candidate_%02d.mat',start));
    config=c; config.outputDir=fullfile(outputDir,sprintf('candidate_%02d',start));
    inputHash=abfdHash(struct('task',task,'config',config,'start',start, ...
        'implementation',report.implementationHash,'generator',report.generatorHash));
    attemptTimer=tic;
    if isfile(checkpoint)
        saved=load(checkpoint,'result','inputHash','row');
        assert(strcmp(saved.inputHash,inputHash),'ABFD:HeldoutCheckpoint','Candidate checkpoint changed.');
        result=saved.result; row=saved.row;
    else
        try
            [cp,seed]=buildObstacleAwareSplineSeed(task,start);
            assert(seed.passed,'ABFD:HeldoutSeed','Ordered geometric start was infeasible.');
            [~,initial]=buildLearnedInitialGuessIHSID(cp,task.model,task.scene,disc,provider);
            result=solveAbfdIhsid(task.model,task.scene,provider,config,initial);
            row=struct('start',start,'passed',result.passed,'seconds',toc(attemptTimer), ...
                'solverStatus',result.solverStatus,'iterations',result.iterations,'objective',result.objective, ...
                'reason',result.failureReason);
        catch e
            result=struct('passed',false,'identifier',e.identifier,'message',e.message);
            row=struct('start',start,'passed',false,'seconds',toc(attemptTimer),'solverStatus',e.identifier, ...
                'iterations',0,'objective',NaN,'reason',e.message);
        end
        save(checkpoint,'result','inputHash','row');
    end
    if isempty(rows), rows=row; else, rows(start)=row; end
    report.attempts=rows; report.attemptedStarts=start; report.elapsedSeconds=toc(timer);
    if result.passed
        assert(result.bundle.publishPassed,'ABFD:HeldoutReference','Selected reference lacks dense acceptance.');
        bundle=result.bundle; save(fullfile(outputDir,'reference.mat'),'bundle','result','task','config');
        report.passed=true; report.selectedStart=start;
        report.referenceHash=abfdFileHash(fullfile(outputDir,'reference.mat'));
    end
    writeAbfdJson(fullfile(outputDir,'status.json'),report);
    if report.passed, break; end
end
report.state='complete'; report.elapsedSeconds=toc(timer);
writeAbfdJson(fullfile(outputDir,'status.json'),report);
end
