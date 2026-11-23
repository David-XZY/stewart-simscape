function report=runBenchmark(manifest,modelFile,c,worker,workers)
if nargin<4, worker=1; workers=1; end
stem='benchmark'; if workers>1, stem=sprintf('benchmark_worker_%d',worker); end
checkNetworkExport(modelFile);
tasks=manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
tasks=tasks(cellfun(@(task)strcmp(task.split,'test'),tasks));
methods={'traditional','diffusion','guided_diffusion'};
rows=struct('taskId',{},'family',{},'method',{},'passed',{},'seconds',{},'samplingSeconds',{}, ...
    'rankingSeconds',{},'solveAttempts',{},'minClearance',{},'maxPositionRmsM',{},'maxAttitudeRmsRad',{},'reason',{});
assetHash=dpHash(fileread(modelFile)); implementation=dpImplementationHash(c);
config=dpRunConfig(c);
for i=worker:workers:numel(tasks)
    task=tasks{i};
    for j=1:3
        method=methods{j}; folder=fullfile(c.output,'benchmark',task.taskId,method);
        if ~isfolder(folder), mkdir(folder); end
        resultFile=fullfile(folder,'benchmark.mat');
        fingerprint=dpHash(struct('task',task.hash,'model',assetHash,'implementation',implementation,'method',method,'config',config));
        if isfile(resultFile)
            loaded=load(resultFile); assert(strcmp(loaded.fingerprint,fingerprint),'DP:Resume','Benchmark fingerprint changed.'); row=loaded.row;
        else
            % Every method pays its own first NLP build; only retries within
            % that method may reuse the task-specific symbolic problem.
            clear dpSolve;
            timer=tic; sampleTimer=tic; candidates=[]; generation=[]; outcome=[]; diversity=[];
            try
                if j==1
                    candidates=zeros(6,16,c.candidateCount);
                    for k=1:c.candidateCount, candidates(:,:,k)=dpSeed(task,k); end
                    generation=struct('kind','geometric-multistart');
                else
                    [candidates,generation]=sampleCandidates(task,modelFile,c.candidateCount,c.seed+task.seed,j==3,c);
                end
                samplingSeconds=toc(sampleTimer); diversity=dpDiversity(candidates,task);
                local=c; local.output=folder; outcome=refineAndValidate(task,candidates,local);
                passed=outcome.passed; clearance=NaN; reason='All candidates failed.';
                if passed, clearance=outcome.best.report.minClearance; reason='passed'; end
                row=struct('taskId',task.taskId,'family',task.family,'method',method,'passed',passed, ...
                    'seconds',toc(timer),'samplingSeconds',samplingSeconds,'rankingSeconds',outcome.rankingSeconds, ...
                    'solveAttempts',numel(outcome.attempts),'minClearance',clearance, ...
                    'maxPositionRmsM',diversity.maxPositionRmsM,'maxAttitudeRmsRad',diversity.maxAttitudeRmsRad,'reason',reason);
            catch e
                row=struct('taskId',task.taskId,'family',task.family,'method',method,'passed',false, ...
                    'seconds',toc(timer),'samplingSeconds',toc(sampleTimer),'rankingSeconds',0,'solveAttempts',0, ...
                    'minClearance',NaN,'maxPositionRmsM',NaN,'maxAttitudeRmsRad',NaN,'reason',e.message);
            end
            save(resultFile,'row','candidates','generation','outcome','diversity','task','fingerprint','config','-v7');
        end
        rows(end+1)=row; %#ok<AGROW>
        dpWriteJson(fullfile(c.output,[stem,'.json']),rows); writetable(struct2table(rows),fullfile(c.output,[stem,'.csv']));
        fprintf('BENCHMARK %s %s pass=%d %.1fs\n',task.taskId,method,row.passed,row.seconds);
    end
end
report=struct('complete',true,'records',rows,'assetHash',assetHash,'implementationHash',implementation);
dpWriteJson(fullfile(c.output,[stem,'_complete.json']),report);
end
