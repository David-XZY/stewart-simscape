function summary=pgReferenceBatch(manifestPath,runName,maxAttempts)
if nargin<2, runName='payload_references_v1'; end
if nargin<3, maxAttempts=576; end
c=pgConfig(runName); manifest=jsondecode(fileread(manifestPath)); maxAttempts=min(maxAttempts,576);
assert(~isfile(fullfile(c.output,'reference_summary.json')),'PG:ImmutableRun','Use a new output directory.');
pilot=jsondecode(fileread(fullfile(fileparts(c.output),'payload_pilot_v1','pilot_summary.json')));
assert(pilot.strictSuccesses>0,'PG:StageGate','Pilot zero-success gate prohibits expansion.');
assert(strcmp(pilot.experimentVersion,c.version),'PG:Version','Pilot version mismatch.');
copyfile(manifestPath,fullfile(c.output,'frozen_tasks.json')); specs=manifest.tasks; count=0;
modes=cell(numel(specs),1); completed=false(numel(specs),8); rows={}; accepted=0; started=tic;
% Preserve pilot attempts as sources; do not rerun them and hide their costs.
priorFolders={'payload_pilot_v1'};
smokeFolder=fullfile(fileparts(c.output),'payload_reference_executor_smoke_v1');
if ~strcmp(runName,'payload_reference_executor_smoke_v1')&&isfile(fullfile(smokeFolder,'reference_summary.json'))
    priorFolders{end+1}='payload_reference_executor_smoke_v1';
end
for folderIndex=1:numel(priorFolders)
files=dir(fullfile(fileparts(c.output),priorFolders{folderIndex},'attempt_*.mat'));
for k=1:numel(files)
    src=load(fullfile(files(k).folder,files(k).name)); r=src.row; count=count+1;
    idx=find(strcmp({specs.taskId},r.taskId)); completed(idx,r.startId)=true;
    if r.passed
        f=pgModeFeatures(src.sol.bundle,src.task);
        if isempty(modes{idx})||min(sqrt(mean((modes{idx}-f).^2,1)))>.15, modes{idx}=[modes{idx},f]; end
    end
end
end
for seedId=1:8
    for i=1:numel(specs)
        if count>=maxAttempts, break; end
        if completed(i,seedId)||(seedId>2&&size(modes{i},2)>=2), continue; end
        task=pgMakeTask(specs(i),c); count=count+1; timer=tic; sol=[];
        row=struct('attempt',count,'taskId',task.taskId,'baseSceneId',task.baseSceneId,'split',task.split,'shape',task.payload.shape, ...
            'mass',task.payload.mass,'T',task.T,'startId',seedId,'endpointPassed',task.endpointPassed,'passed',false,'reason',task.endpointReason,'codeHash',c.codeHash);
        if task.endpointPassed
            cc=c; cc.clockHandle=timer; cc.wallBudget=120; cc.maxCpuSeconds=120;
            try
                sol=pgSolve(task,bdSeed(task,seedId,c),cc); row.passed=sol.passed; row.reason=sol.solverStatus;
                row.solverSuccess=sol.solverSuccess; row.maxEquality=sol.maxEquality; row.maxInequality=sol.maxInequality; row.maxBounds=sol.maxBounds;
                row.physicalPassed=sol.report.passed; row.physicalReason=sol.report.reason; row.buildSeconds=sol.buildSeconds; row.solveSeconds=sol.solveSeconds;
            catch e, row.reason=[e.identifier,':',e.message]; row.exceptionStack=getReport(e,'extended','hyperlinks','off'); end
        end
        row.actualAttemptSeconds=toc(timer); row.passed=row.passed&&row.actualAttemptSeconds<=120; completed(i,seedId)=true;
        row.newMode=false;
        if row.passed
            f=pgModeFeatures(sol.bundle,task);
            row.newMode=isempty(modes{i})||min(sqrt(mean((modes{i}-f).^2,1)))>.15;
            if row.newMode&&size(modes{i},2)<4, modes{i}=[modes{i},f]; end
            accepted=accepted+1;
        end
        stem=sprintf('attempt_%03d',count); row.trajectoryRef=[stem,'.mat'];
        save(fullfile(c.output,[stem,'.mat']),'task','sol','row','-v7'); dpWriteJson(fullfile(c.output,[stem,'.json']),row); rows{end+1}=row;
        dpWriteJson(fullfile(c.output,'progress.json'),struct('attemptsIncludingPilot',count,'strictSuccessesNew',accepted,'elapsedSeconds',toc(started),'last',row));
        fprintf('PG_REFERENCES %d/%d accepted=%d\n',count,maxAttempts,accepted);
    end
    if count>=maxAttempts, break; end
end
summary=struct('experimentVersion',c.version,'attemptsIncludingPilot',count,'strictSuccessesNew',accepted,'records',{rows},'modeCounts',cellfun(@(x)size(x,2),modes), ...
    'elapsedSeconds',toc(started),'trainingCalls',0,'codeHash',c.codeHash,'priorSources',{priorFolders},'unfinishedConditions',sum(sum(completed,2)<2));
% All source attempts remain present, even invalid endpoints and duplicate modes.
dpWriteJson(fullfile(c.output,'reference_summary.json'),summary);
end
