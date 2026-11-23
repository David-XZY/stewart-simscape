addpath('diffusion'); c=setupDiffusion(); c.output=fullfile(c.output,'merge_unit_fixture');
mkdir(fullfile(c.output,'training'));
dpWriteJson(fullfile(c.output,'training','model.json'),struct('kind','unit-test-metadata-only'));
assetHash=dpHash(fileread(fullfile(c.output,'training','model.json'))); implementation=dpImplementationHash(c);
manifest.tasks={struct('taskId','fixture_A','hash','A','split','test'),struct('taskId','fixture_B','hash','B','split','test')};
methods={'traditional','diffusion','guided_diffusion'};
for i=1:2
    task=manifest.tasks{i};
    for j=1:3
        folder=fullfile(c.output,'benchmark',task.taskId,methods{j}); mkdir(folder);
        row=struct('taskId',task.taskId,'method',methods{j},'passed',false,'seconds',0,'reason','Unit test fixture, no planning performed.');
        fingerprint=dpHash(struct('task',task.hash,'model',assetHash,'implementation',implementation,'method',methods{j},'config',dpRunConfig(c)));
        file=fullfile(folder,'benchmark.mat'); save(file,'row','fingerprint');
    end
end
report=mergeBenchmark(manifest,c); assert(numel(report.records)==6 && all(~[report.records.passed]));
changed=c; changed.guidanceStep=2*c.guidanceStep; caught=false;
try, mergeBenchmark(manifest,changed); catch e, caught=strcmp(e.identifier,'DP:BenchmarkProvenance'); end
assert(caught);
fingerprint='intentionally_wrong'; save(file,'row','fingerprint'); caught=false;
try, mergeBenchmark(manifest,c); catch e, caught=strcmp(e.identifier,'DP:BenchmarkProvenance'); end
assert(caught);
delete(file); caught=false;
try, mergeBenchmark(manifest,c); catch e, caught=strcmp(e.identifier,'DP:BenchmarkIncomplete'); end
assert(caught); fprintf('BENCHMARK_MERGE_UNIT_PASS\n');
