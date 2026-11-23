function report=run_F2_warmstart_benchmark(tasks,assets,c,selectionPolicy)
% Empty asset file uses the original single start. No simulation or fitting.
% Explicit all-candidates preserves the earlier exhaustive diagnostic mode.
if nargin<4, selectionPolicy='ranked-top-k'; end
selectionPolicy=validatestring(selectionPolicy,{'ranked-top-k','all-candidates'});
c=makeAbfdConfig(c);
assert(~isempty(tasks) && ~isempty(assets),'ABFD:EmptyBenchmark','Tasks and methods are required.');
records=struct('taskId',{},'method',{},'candidate',{},'selected',{},'passed',{},'seconds',{}, ...
    'solveSeconds',{},'preparationSeconds',{},'samplingBatchSeconds',{},'rankingBatchSeconds',{}, ...
    'iterations',{},'retryCount',{},'objective',{},'reason',{});
summaries=struct('taskId',{},'method',{},'passed',{},'candidateCount',{},'refinedCount',{}, ...
    'seconds',{},'bestObjective',{},'diversity',{});
for i=1:numel(tasks)
    task=tasks(i); p=buildDynamicsProvider(task.model);
    disc=buildTwoPhaseIHSDiscretization(task.scene,task.scene.phase.numIntervalsApproach,task.scene.phase.numIntervalsInsertion);
    for j=1:numel(assets)
        methodTimer=tic; samplingTimer=tic;
        cp=[]; if ~isempty(assets(j).file), cp=sampleAbfdCandidates(task,assets(j).file,c.candidateCount,c.seed); end
        samplingSeconds=toc(samplingTimer); count=max(1,size(cp,3));
        methodDir=fullfile(c.outputDir,task.taskId,sprintf('method_%02d',j));
        if ~isfolder(methodDir), mkdir(methodDir); end
        initial=cell(1,count); selected=1:count; rankingSeconds=0;
        reasons=repmat({''},1,count); diversity=struct();
        if ~isempty(cp)
            diversity=evaluateProposalDiversity(cp);
            if strcmp(selectionPolicy,'ranked-top-k')
                [initial,ranking]=rankAbfdProposalCandidates(task,cp,p,c);
                selected=ranking.selected; reasons=ranking.reasons; rankingSeconds=ranking.seconds;
                save(fullfile(methodDir,'ranking.mat'),'ranking','cp');
            end
        end
        methodPassed=false; bestObjective=inf; best=[];
        for k=1:count
            chosen=ismember(k,selected);
            row=struct('taskId',task.taskId,'method',assets(j).name,'candidate',k,'selected',chosen,'passed',false, ...
                'seconds',0,'solveSeconds',0,'preparationSeconds',0,'samplingBatchSeconds',samplingSeconds, ...
                'rankingBatchSeconds',rankingSeconds,'iterations',0,'retryCount',0,'objective',NaN,'reason',reasons{k});
            if chosen
                candidateTimer=tic;
                try
                    if ~isempty(cp) && isempty(initial{k})
                        [~,initial{k}]=buildLearnedInitialGuessIHSID(cp(:,:,k),task.model,task.scene,disc,p);
                    end
                    local=c; local.designMode='sequential';
                    local.outputDir=fullfile(methodDir,sprintf('candidate_%02d',k));
                    result=solveAbfdIhsid(task.model,task.scene,p,local,initial{k});
                    if ~isfolder(local.outputDir), mkdir(local.outputDir); end
                    save(fullfile(local.outputDir,'result.mat'),'result');
                    row.passed=result.passed; row.solveSeconds=result.solveTime; row.iterations=result.iterations;
                    row.preparationSeconds=result.preparationTime; row.retryCount=result.retryCount;
                    row.objective=result.objective; row.reason=result.failureReason;
                    if result.passed
                        methodPassed=true;
                        if result.objective<bestObjective, bestObjective=result.objective; best=result; end
                    end
                catch exception, row.reason=exception.message;
                end
                row.seconds=toc(candidateTimer);
            elseif isempty(row.reason)
                row.reason='Not selected by the frozen ranking policy.';
            end
            records(end+1)=row; %#ok<AGROW>
            publish();
        end
        if isempty(best), bestObjective=NaN;
        else, save(fullfile(methodDir,'best_result.mat'),'best'); end
        summaries(end+1)=struct('taskId',task.taskId,'method',assets(j).name, ...
            'passed',methodPassed,'candidateCount',count,'refinedCount',numel(selected), ...
            'seconds',toc(methodTimer),'bestObjective',bestObjective,'diversity',diversity); %#ok<AGROW>
        publish();
    end
end
report.status='experiment-completed';
writeAbfdJson(fullfile(c.outputDir,'warmstart_benchmark.json'),report);
    function publish()
        report=struct('records',records,'summaries',summaries,'status','running', ...
            'selectionPolicy',selectionPolicy,'candidateCount',c.candidateCount,'refineCount',c.refineCount, ...
            'timingContract','Candidate seconds includes conversion and every refinement/retry; add sampling/ranking batch seconds once per task/method. Method seconds is measured wall time.', ...
            'scope','Numerical F holdout benchmark; no Simscape or learned-model fitting; unselected and failed candidates are retained.');
        writeAbfdJson(fullfile(c.outputDir,'warmstart_benchmark.json'),report);
    end
end
