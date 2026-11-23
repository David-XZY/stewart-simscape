function report=ch5CheckPlanningTiming()
[c,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','validation','planning_timing');
addpath(directory);[physical,scene,ad,strict]=ch5Physics(c);collision=ch5CollisionModel(physical,scene,c);
r=struct('c',c,'physical',physical,'scene',scene,'ad',ad,'strict',strict,'collision',collision);
sample=load(fullfile(root,'chapter5','artifacts','development','development_004_ideal_force_baseline_0.mat'),'records','plans');
first=sample.records{1};timer=tic;
initial=ch5IhsidTiming(first.q,first.qd,first.goal,first.observed,r,6);
firstFailure=[];
for k=2:numel(sample.plans)
    p=sample.plans{k};
    if ~p.plan.accepted&&~isfield(p.plan,'precheck_infeasible')
        firstFailure=p;break;
    end
end
assert(~isempty(firstFailure));
times=cellfun(@(x)x.time,sample.records);[~,index]=min(abs(times-firstFailure.time));entry=sample.records{index};
failure=ch5IhsidTiming(entry.q,entry.qd,entry.goal,firstFailure.observed,r,firstFailure.plan.obstacle_prediction_times(end));
manifest=jsondecode(fileread(fullfile(directory,'source_manifest.json')));
report=struct('original_source_sha256',manifest.original_sha256, ...
    'pid',feature('getpid'),'initial',initial,'failure',failure,'wall_seconds',toc(timer));
save(fullfile(directory,'timings.mat'),'report','-v7');
fid=fopen(fullfile(directory,'timings.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
