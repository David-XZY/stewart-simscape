function run_abfd_f_batch_stage(stage,outputDir,index)
% Separate MATLAB processes own separate tasks, checkpoints and cache folders.
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
if strcmp(stage,'prepare')
    assert(~isfile(fullfile(outputDir,'frozen.mat')),'ABFD:BatchExists','Frozen batch already exists.');
    tasks=makeAbfdTaskManifest('proposal',60,20260905);
    c=makeAbfdConfig(struct('outputDir',outputDir,'solverMaxIterations',1500, ...
        'solverMaxCpuTime',600,'plannerProfile','guarded-adaptive-v2'));
    configHash=abfdHash(rmfield(c,'outputDir')); implementationHash=abfdPlannerImplementationHash();
    generatorHash=abfdHash(struct('planner',implementationHash, ...
        'generator',abfdFileHash(fullfile(root,'research','f_trajectory_proposal','run_F1_generate_multistart_dataset.m'))));
    items=struct('index',{},'taskId',{},'split',{},'taskHash',{});
    for k=1:numel(tasks)
        items(k)=struct('index',k,'taskId',tasks(k).taskId,'split',tasks(k).split,'taskHash',abfdHash(tasks(k)));
    end
    manifest=struct('schemaVersion',1,'kind','frozen-F-batch','tasks',items, ...
        'configHash',configHash,'implementationHash',implementationHash,'generatorHash',generatorHash, ...
        'candidateCount',c.candidateCount,'plannerProfile',c.plannerProfile,'matlab',version);
    save(fullfile(outputDir,'frozen.mat'),'tasks','c','manifest');
    writeAbfdJson(fullfile(outputDir,'task_manifest.json'),manifest);
    writeAbfdJson(fullfile(outputDir,'config.json'),c);
    return;
end
if strcmp(stage,'audit')
    run_F0_multimodality_audit(fullfile(outputDir,'proposal_dataset.json'),fullfile(outputDir,'multimodality_audit.json'));
    return;
end
assert(strcmp(stage,'task'),'ABFD:UnknownStage','Unknown F batch stage.');
frozen=load(fullfile(outputDir,'frozen.mat')); task=frozen.tasks(index); c=frozen.c;
assert(strcmp(frozen.manifest.implementationHash,abfdPlannerImplementationHash()), ...
    'ABFD:SourceChanged','Planner source changed during a frozen run.');
c.outputDir=fullfile(outputDir,'tasks',task.taskId,'F1');
dataset=[];
try
    dataset=run_F1_generate_multistart_dataset(task,c);
catch exception
    if ~strcmp(exception.identifier,'ABFD:EmptyDataset'), rethrow(exception); end
end
attempts=jsondecode(fileread(fullfile(c.outputDir,'proposal_attempts.json')));
assert(numel(attempts)==c.candidateCount && isequal([attempts.start],1:c.candidateCount), ...
    'ABFD:IncompleteShard','Not every frozen start has a terminal record.');
status=struct('state','complete','taskId',task.taskId,'taskHash',abfdHash(task), ...
    'configHash',frozen.manifest.configHash,'implementationHash',frozen.manifest.implementationHash, ...
    'passed',sum([attempts.passed]),'attempted',numel(attempts),'datasetHash','');
if ~isempty(dataset), status.datasetHash=abfdFileHash(fullfile(c.outputDir,'proposal_dataset.json')); end
writeAbfdJson(fullfile(outputDir,'tasks',task.taskId,'task_status.json'),status);
end
