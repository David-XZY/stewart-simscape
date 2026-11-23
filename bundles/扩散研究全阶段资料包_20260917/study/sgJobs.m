function jobs=sgJobs(manifest,c)
jobs={}; primary={'geo_ihsid','diff_ihsid','geo_fast','retrieve_fast','diff_direct','diff_fast'};
for i=1:numel(manifest.cases)
    item=manifest.cases{i};
    if item.core, methods=primary; else, methods={'geo_ihsid','diff_ihsid','diff_fast'}; end
    for method=methods
        repeats=1; if item.core && strncmp(method{1},'diff',4), repeats=3; end
        count=64; budget=c.fastBudget;
        if ~isempty(strfind(method{1},'ihsid')), count=8; budget=c.solverBudget; end %#ok<STREMP>
        for repeat=1:repeats, add(i,false,method{1},count,repeat,budget,'main'); end
    end
    if item.core
        for count=[1,4,8,16,32]
            for repeat=1:3, add(i,false,'diff_direct',count,repeat,c.fastBudget,'candidate_count'); end
        end
        if strcmp(item.category,'id')
            add(i,false,'diff_legacy_check',8,1,c.fastBudget,'checker_ablation');
        end
    end
end
for i=1:numel(manifest.probes)
    for repeat=1:3, add(i,true,'diff_direct',64,repeat,c.fastBudget,'permutation'); end
end
% Eight preselected quality references: first task of each scene/category.
for family={'simple','hood'}
    for category={'id','goal_position','goal_attitude','layout'}
        selected=find(cellfun(@(x)strcmp(x.family,family{1})&&strcmp(x.category,category{1})&&x.core,manifest.cases),1);
        add(selected,false,'geo_reference',8,1,c.referenceBudget,'reference');
    end
end
% Fixed interleaving: core+candidate measurements, then mixed scans, reference.
stream=RandStream('mt19937ar','Seed',c.seed+9000); tie=rand(stream,1,numel(jobs)); priorities=zeros(1,numel(jobs));
for k=1:numel(jobs)
    if strcmp(jobs{k}.purpose,'reference'), priorities(k)=3;
    elseif strcmp(jobs{k}.purpose,'permutation'), priorities(k)=1;
    elseif manifest.cases{jobs{k}.caseIndex}.core, priorities(k)=0;
    else, priorities(k)=2; end
end
[~,order]=sortrows([priorities(:),tie(:)],[1,2]); jobs=jobs(order);
save(fullfile(c.output,'jobs.mat'),'jobs','-v7'); dpWriteJson(fullfile(c.output,'jobs.json'),jobs);
fprintf('SG_JOBS %d scheduled\n',numel(jobs));
    function add(index,probe,method,count,repeat,budget,purpose)
        if probe, item=manifest.probes{index}; else, item=manifest.cases{index}; end
        taskSeed=c.seed; if ~isempty(item.planningTask), taskSeed=item.planningTask.seed; end
        seed=mod(double(taskSeed)+100000*repeat,2^32-1);
        id=sprintf('%s__%s__k%02d__r%d',item.caseId,method,count,repeat);
        jobs{end+1}=struct('jobId',id,'caseId',item.caseId,'caseIndex',index,'probe',probe, ...
            'method',method,'count',count,'repeat',repeat,'seed',seed,'budget',budget,'purpose',purpose);
    end
end
