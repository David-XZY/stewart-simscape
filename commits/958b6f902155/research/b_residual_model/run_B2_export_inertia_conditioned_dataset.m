function report=run_B2_export_inertia_conditioned_dataset(originalRoot,replayRoot,structuredRoot,contextRoot,outputDir)
% Explicit new inference protocol: every load has an independent startup
% force/acceleration identification. Exposed historical test cases stay
% diagnostic-only; they never fit shared coefficients or choose models.
setupAbfdResearch();
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:ConditionedDatasetExists','Use a fresh dataset directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
parentFile=fullfile(structuredRoot,'residual_dataset.json'); parent=jsondecode(fileread(parentFile));
contextFile=fullfile(contextRoot,'context_dataset.json'); contexts=jsondecode(fileread(contextFile));
contextStatus=jsondecode(fileread(fullfile(contextRoot,'status.json')));
assert(contextStatus.passed && strcmp(contextStatus.state,'complete') && ...
    strcmp(contextStatus.contextDatasetHash,abfdFileHash(contextFile)), ...
    'ABFD:ConditionedContextSource','Independent startup observations were not verified.');
tasksFile=fullfile(originalRoot,'B_tasks_frozen.json'); tasks=jsondecode(fileread(tasksFile));
assert(strcmp(contextStatus.tasksHash,abfdFileHash(tasksFile)) && ...
    strcmp(contexts.conditioning.baseDatasetHash,abfdFileHash(parentFile)), ...
    'ABFD:ConditionedContextSource','Frozen observations belong to another dataset or task manifest.');
output=parent; output.y=zeros(size(parent.y)); output.anchorX=zeros(size(parent.x));
output.physical.inferenceCalibration=struct('kind','independent-initial-inertia-static-v2', ...
    'observation','Independent spatial-acceleration response per actual leg force and static equilibrium before execution', ...
    'required',true,'scope','Per-task startup identification is additional; shared passive coefficients remain train-only');
output.meanAnchoring='initial-static-state-v1';
output.testEvaluationRole='previously exposed historical test; diagnostic only, not new independent acceptance';
output.scope='Explicit independent startup inertia/static protocol for every task; train-only normalization and fitting, validation-only model choice; previously exposed test is diagnostic only';
report=struct('state','running','passed',false,'completed',0,'expected',numel(parent.pairedSourceCases), ...
    'parentDatasetHash',abfdFileHash(parentFile),'contextDatasetHash',abfdFileHash(contextFile), ...
    'tasksHash',abfdFileHash(tasksFile),'cases',[],'protocol','independent-initial-inertia-static-v2','scope',output.scope);
writeAbfdJson(fullfile(outputDir,'status.json'),report); rows=struct([]); cursor=0;
try
    for k=1:numel(parent.pairedSourceCases)
        source=parent.pairedSourceCases(k); id=source.taskId; task=tasks(strcmp({tasks.taskId},id));
        context=contexts.initialBalanceContexts(strcmp({contexts.initialBalanceContexts.taskId},id));
        assert(isscalar(task) && isscalar(context) && strcmp(task.split,source.split) && ...
            strcmp(context.split,source.split),'ABFD:ConditionedDatasetSplit','Frozen group membership differs.');
        originalFile=fullfile(originalRoot,'B0',id,'result.mat');
        assert(strcmp(abfdFileHash(originalFile),source.sourceHash),'ABFD:ConditionedDatasetSource','Original task changed.');
        original=load(originalFile,'result'); bundle=original.result.planning.bundle; profile=original.result.profile;
        calibrationFile=context.calibrationFile;
        assert(strcmp(abfdFileHash(calibrationFile),context.calibrationHash), ...
            'ABFD:ConditionedDatasetSource','Independent response observation changed.');
        % Fixed per-task inference rule, not a fit to the upcoming motion.
        physical=readMeasuredInertiaRegression(calibrationFile,bundle,profile);
        calibration=jsondecode(fileread(calibrationFile)); q0=physical.pose(:);
        m=task.model; m.dynamics.totalMass=physical.mass; m.dynamics.comP=physical.com;
        m.dynamics.inertiaAtCOM_P=physical.inertia;
        initial=sgpJacobian(q0,m); rest=sgpIK(q0,m);
        offset=initial.Jv.'*calibration.equilibriumForce(:)-computeCompositeRequiredWrench(q0,zeros(6,1),zeros(6,1),m);
        replayFile=fullfile(replayRoot,id,'result.mat');
        assert(strcmp(abfdFileHash(replayFile),source.resultHash),'ABFD:ConditionedDatasetSource','Frozen replay changed.');
        saved=load(replayFile,'result'); log=saved.result.log; data=buildResidualWrenchTarget(log,task.model);
        n=numel(log.t); ids=cursor+(1:n);
        assert(isequal(data.x,parent.x(ids,:)) && all(strcmp(string(parent.groups(ids)),id)) && ...
            norm(q0-log.q(:,1),inf)<1e-10,'ABFD:ConditionedDatasetRows','Rows, group or observed initial pose changed.');
        e=parent.physical.elasticity;
        for j=1:n
            q=log.q(:,j); v=log.qd(:,j); J=sgpJacobian(q,m); kin=sgpIK(q,m);
            passive=J.Jv.'*(e.stiffness*(kin.L-rest.L)+e.damping*(J.Jq*v));
            output.y(ids(j),:)=data.wrench(j,:)- ...
                (computeCompositeRequiredWrench(q,v,log.qdd(:,j),m)+passive+offset).';
        end
        output.anchorX(ids,:)=repmat([q0;zeros(6,1)].',n,1);
        row=struct('taskId',id,'split',task.split,'firstRow',cursor+1,'rows',n, ...
            'physical',physical,'staticOffset',offset,'staticForce',calibration.equilibriumForce(:), ...
            'initialPose',q0,'initialLength',rest.L,'calibrationFile',calibrationFile, ...
            'calibrationHash',context.calibrationHash,'plantHash',context.plantHash, ...
            'sourceHash',source.resultHash,'originalSourceHash',source.sourceHash, ...
            'collectionSeconds',context.collectionSeconds,'reusedObservation',true, ...
            'exposedTest',strcmp(task.split,'test'));
        if isempty(rows), rows=row; else, rows(k)=row; end
        cursor=cursor+n; report.completed=k; report.cases=rows;
        writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    assert(cursor==size(parent.x,1) && all(isfinite(output.y(:))),'ABFD:ConditionedDatasetRows','Incomplete or nonfinite residual labels.');
    output.inertiaContexts=rows; output.parentDatasetHash=report.parentDatasetHash;
    output.contextSourceHash=report.contextDatasetHash;
    output.targetDefinition='Actual world wrench minus independently startup-identified rigid wrench, train-identified passive wrench and independent initial static offset; MLP mean is phi(x)-phi(anchorX)';
    writeAbfdJson(fullfile(outputDir,'residual_dataset.json'),output);
    report.state='complete'; report.passed=true; report.rows=cursor;
    report.datasetHash=abfdFileHash(fullfile(outputDir,'residual_dataset.json'));
    writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    report.detail=getReport(e,'extended','hyperlinks','off'); writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
