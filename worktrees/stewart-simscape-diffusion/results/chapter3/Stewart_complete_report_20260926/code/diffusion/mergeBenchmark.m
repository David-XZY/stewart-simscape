function report=mergeBenchmark(manifest,c)
% Merge disjoint task shards only after every scheduled result exists.
tasks=manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
methods={'traditional','diffusion','guided_diffusion'}; rows=[];
assetHash=dpHash(fileread(fullfile(c.output,'training','model.json')));
implementation=dpImplementationHash(c);
config=dpRunConfig(c);
for i=1:numel(tasks)
    task=tasks{i}; if ~strcmp(task.split,'test'), continue; end
    for j=1:3
        file=fullfile(c.output,'benchmark',task.taskId,methods{j},'benchmark.mat');
        assert(isfile(file),'DP:BenchmarkIncomplete','Missing result: %s',file);
        s=load(file,'row','fingerprint');
        expected=dpHash(struct('task',task.hash,'model',assetHash,'implementation',implementation,'method',methods{j},'config',config));
        assert(strcmp(s.fingerprint,expected),'DP:BenchmarkProvenance','Result fingerprint mismatch.');
        if isempty(rows), rows=s.row; else, rows(end+1)=s.row; end %#ok<AGROW>
    end
end
assert(~isempty(rows),'DP:BenchmarkEmpty','No held-out comparisons.');
dpWriteJson(fullfile(c.output,'benchmark.json'),rows); writetable(struct2table(rows),fullfile(c.output,'benchmark.csv'));
report=struct('complete',true,'records',rows,'assetHash',assetHash,'implementationHash',implementation, ...
    'execution','Two independent MATLAB task workers; methods sequential within each task.');
dpWriteJson(fullfile(c.output,'benchmark_complete.json'),report);
end
