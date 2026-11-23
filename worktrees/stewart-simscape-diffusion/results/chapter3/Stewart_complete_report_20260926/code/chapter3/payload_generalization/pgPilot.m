function summary=pgPilot(manifestPath,runName)
% Bounded first 32 attempts. No learned model is loaded in mechanism development.
if nargin<2, runName='payload_pilot_v1'; end
c=pgConfig(runName); manifest=jsondecode(fileread(manifestPath));
assert(~isfile(fullfile(c.output,'pilot_summary.json')),'PG:ImmutableRun','Use a new run directory.');
auditPath=fullfile(fileparts(c.output),'payload_audit_v1');
ma=jsondecode(fileread(fullfile(auditPath,'matlab_audit.json'))); pa=jsondecode(fileread(fullfile(auditPath,'torch_audit.json')));
assert(ma.passed&&pa.passed,'PG:StageGate','Stage A required.');
copyfile(manifestPath,fullfile(c.output,'frozen_tasks.json')); rows={}; count=0; success=0; started=tic;
specs=manifest.tasks; ids=manifest.pilotTaskIds;
for startId=1:2
    for k=1:numel(ids)
        spec=specs(strcmp({specs.taskId},ids{k})); taskClock=tic;
        task=pgMakeTask(spec,c); buildTaskSeconds=toc(taskClock); count=count+1;
        row=struct('attempt',count,'taskId',task.taskId,'baseSceneId',task.baseSceneId,'split',task.split, ...
            'shape',task.payload.shape,'mass',task.payload.mass,'T',task.T,'family',task.family,'startId',startId, ...
            'taskHash',task.hash,'codeHash',c.codeHash,'endpointPassed',task.endpointPassed,'passed',false, ...
            'reason',task.endpointReason,'taskConstructionSeconds',buildTaskSeconds);
        attemptClock=tic; sol=[];
        if task.endpointPassed
            cc=c; cc.clockHandle=attemptClock; cc.wallBudget=120; cc.maxCpuSeconds=120;
            try
                sol=pgSolve(task,bdSeed(task,startId,c),cc); row.passed=sol.passed; row.reason=sol.solverStatus;
                row.solverSuccess=sol.solverSuccess; row.maxEquality=sol.maxEquality; row.maxInequality=sol.maxInequality; row.maxBounds=sol.maxBounds;
                row.physicalPassed=sol.report.passed; row.physicalReason=sol.report.reason;
                row.buildSeconds=sol.buildSeconds; row.solveSeconds=sol.solveSeconds; row.validationSeconds=sol.validationSeconds;
                row.cacheHit=sol.problemCacheHit;
            catch e
                row.reason=[e.identifier,':',e.message]; row.exceptionStack=getReport(e,'extended','hyperlinks','off');
            end
        end
        row.actualAttemptSeconds=toc(attemptClock); row.returnedWithinBudget=row.actualAttemptSeconds<=120;
        row.passed=row.passed&&row.returnedWithinBudget; success=success+row.passed;
        stem=sprintf('attempt_%03d',count); row.trajectoryRef=[stem,'.mat'];
        save(fullfile(c.output,[stem,'.mat']),'task','sol','row','-v7'); dpWriteJson(fullfile(c.output,[stem,'.json']),row);
        rows{end+1}=row; dpWriteJson(fullfile(c.output,'progress.json'),struct('attempts',count,'strictSuccesses',success,'elapsedSeconds',toc(started),'last',row));
        fprintf('PG_PILOT %d/32 strict=%d reason=%s\n',count,success,row.reason);
    end
end
summary=struct('experimentVersion',c.version,'attempts',count,'strictSuccesses',success,'taskCount',numel(ids), ...
    'records',{rows},'elapsedSeconds',toc(started),'trainingCalls',0,'diffusionCalls',0,'codeHash',c.codeHash, ...
    'nextGate','audit_labels_before_expansion','referenceResourceHours',72);
if success==0, summary.nextGate='STOP_zero_strict_success_audit_solver'; end
dpWriteJson(fullfile(c.output,'pilot_summary.json'),summary);
end
