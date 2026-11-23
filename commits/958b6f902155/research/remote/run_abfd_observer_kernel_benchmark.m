function report=run_abfd_observer_kernel_benchmark(frozenFile,outputDir)
% Paired microbenchmark on three predeclared training models, after F ends.
assert(~isfolder(outputDir),'ABFD:ObserverBenchmarkExists','Use a fresh benchmark directory.'); mkdir(outputDir);
timer=tic; data=load(frozenFile,'tasks'); ids=find(strcmp({data.tasks.split},'train'));
assert(numel(ids)>=3,'ABFD:ObserverBenchmarkTrain','Three original training contexts are required.');
files={'stepAbfdIntervalObserver','stepAbfdIntervalObserverFused','buildAbfdIntervalObserverKernel', ...
    'stepAbfdExternalObserver','buildCasadiCompositeDynamics','testIntervalObserverKernel'};
paths=cellfun(@which,files,'UniformOutput',false);
hashes=cellfun(@abfdFileHash,paths,'UniformOutput',false);
writeAbfdJson(fullfile(outputDir,'freeze.json'),struct('inputHash',abfdFileHash(frozenFile), ...
    'trainingTaskIds',{{data.tasks(ids(1:3)).taskId}},'sourceFiles',{paths},'sourceHashes',{hashes}));
records=struct('taskId',{},'checks',{},'currentMedian',{},'fusedMedian',{},'currentP95',{},'fusedP95',{});
for j=1:3
    task=data.tasks(ids(j)); checks=testIntervalObserverKernel(task.model);
    p=buildDynamicsProvider(task.model); x=[task.model.qHome;zeros(6,1)]; dt=.01;
    F=p.inverse(x(1:6),x(7:12),zeros(6,1)); P=1e-4*eye(12);
    for k=1:20
        stepAbfdIntervalObserver(x,P,x(1:6),dt*F,p,dt);
        stepAbfdIntervalObserverFused(x,P,x(1:6),dt*F,p,dt);
    end
    timings=zeros(400,2); functions={@stepAbfdIntervalObserver,@stepAbfdIntervalObserverFused};
    for k=1:size(timings,1)
        order=[1,2]; if mod(k,2)==0, order=[2,1]; end
        for index=order
            started=tic; functions{index}(x,P,x(1:6),dt*F,p,dt); timings(k,index)=toc(started);
        end
    end
    ordered=sort(timings,1); p95=ordered(ceil(.95*size(timings,1)),:);
    records(j)=struct('taskId',task.taskId,'checks',checks,'currentMedian',median(timings(:,1)), ...
        'fusedMedian',median(timings(:,2)),'currentP95',p95(1),'fusedP95',p95(2));
    save(fullfile(outputDir,sprintf('timings_%02d.mat',j)),'timings','task','checks');
    writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','records',records,'seconds',toc(timer)));
end
report=struct('state','complete','passed',all(arrayfun(@(item)item.checks.passed,records)), ...
    'records',records,'seconds',toc(timer),'percentileDefinition','nearest rank, 400 alternating-order paired samples after warm-up', ...
    'scope','Three frozen training models; identical observer equations and refusal gates; standalone compute timing, not full controller P95 or Simscape performance acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
end
