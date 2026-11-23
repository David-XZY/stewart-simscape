function jobs=c3Jobs(manifest,c)
methods={'geo_ihsid','retrieve_ihsid','diff_ihsid','geo_fast','retrieve_fast','diff_fast'};
jobs={}; stream=RandStream('mt19937ar','Seed',c.seed+701);
% Interleave methods; sequence step order is preserved for every repeat/method.
for i=1:numel(manifest.cases)
    item=manifest.cases{i}; available=methods;
    if strcmp(item.category,'sequence'), available{end+1}='previous_ihsid'; end
    for repeat=1:3
        order=randperm(stream,numel(available));
        for k=order, add(available{k},64,repeat,'main',false,true,true); end
    end
    % One predeclared case per independent scene/category; full pipeline is above.
    if i<=40 && endsWith(item.caseId,'_01')
        for count=[1,4,8,16,32,64]
            for repeat=1:3, add('diff_direct',count,repeat,'candidate_count',false,false,false); end
        end
        for repeat=1:3
            add('diff_fast',64,repeat,'full_sweep',true,true,true);
            add('diff_fast',64,repeat,'no_repair',false,false,true);
            add('diff_fast',64,repeat,'no_fallback',false,true,false);
        end
    end
end
save(fullfile(c.output,'jobs.mat'),'jobs','-v7'); dpWriteJson(fullfile(c.output,'jobs.json'),jobs);
    function add(method,count,repeat,purpose,sweep,repair,fallback)
        if ~isempty(strfind(method,'ihsid')), count=c.solverStarts; end %#ok<STREMP>
        jobs{end+1}=struct('jobId',sprintf('%s__%s__%s__k%02d__r%d',item.caseId,method,purpose,count,repeat), ...
            'caseIndex',i,'caseId',item.caseId,'method',method,'count',count,'repeat',repeat, ...
            'seed',mod(c.seed+i*101+repeat*100000,2^32-1),'budget',c.wallBudget,'purpose',purpose, ...
            'fullSweep',sweep,'repairEnabled',repair,'fallbackEnabled',fallback);
    end
end
