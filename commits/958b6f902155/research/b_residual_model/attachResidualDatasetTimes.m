function report=attachResidualDatasetTimes(datasetFile,sessionRoot,outputFile)
% Restore actual logged clocks without refitting or changing any training rows.
assert(~isfile(outputFile),'ABFD:TimedDatasetExists','Use a fresh timed dataset path.');
data=jsondecode(fileread(datasetFile)); groups=string(data.groups); split=string(data.split);
assert(strcmp(data.kind,'residual-training-dataset'),'ABFD:TemporalDataset','Expected residual dataset.');
time=zeros(size(data.x,1),1); ids=unique(groups,'stable'); sampleTime=[];
sources=struct('taskId',{},'resultHash',{},'rows',{},'split',{});
for k=1:numel(ids)
    taskId=char(ids(k));
    assert(~isempty(regexp(taskId,'^task_[0-9]+$','once')),'ABFD:TemporalTaskId','Invalid task path.');
    rows=find(groups==ids(k));
    assert(all(diff(rows)==1) && isscalar(unique(split(rows))),'ABFD:TemporalGroups','Groups must be contiguous and split-isolated.');
    file=fullfile(sessionRoot,taskId,'result.mat'); saved=load(file,'result'); log=saved.result.log;
    assert(strcmp(log.taskId,taskId) && strcmp(log.forceSource,'measured-actuator-output'), ...
        'ABFD:TemporalSession','Session identity or force source differs.');
    x=[log.q;log.qd].';
    assert(isequal(size(x),size(data.x(rows,:))) && max(abs(x-data.x(rows,:)),[],'all')<1e-12, ...
        'ABFD:TemporalRows','Session rows differ from the original exported dataset.');
    t=log.t(:); difference=diff(t);
    if isempty(sampleTime), sampleTime=median(difference); end
    assert(numel(t)==numel(rows) && sampleTime>0 && all(isfinite(t)) && ...
        all(abs(difference-sampleTime)<1e-8),'ABFD:TemporalClock','Original clocks must be uniform and aligned.');
    time(rows)=t;
    sources(end+1)=struct('taskId',taskId,'resultHash',abfdFileHash(file), ...
        'rows',numel(rows),'split',char(split(rows(1)))); %#ok<AGROW>
end
data.time=time; data.timeSource='original logged state clock; row order unchanged';
data.originalDatasetHash=abfdFileHash(datasetFile); data.timeSources=sources;
writeAbfdJson(outputFile,data);
report=struct('passed',true,'originalDatasetHash',data.originalDatasetHash, ...
    'timedDatasetHash',abfdFileHash(outputFile),'groups',numel(ids),'rows',numel(time),'sampleTime',sampleTime,'sources',sources);
end
