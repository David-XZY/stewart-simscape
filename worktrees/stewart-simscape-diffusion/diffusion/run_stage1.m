function run_stage1()
c=setupDiffusion(); out=fullfile(c.output,'stage1'); if ~isfolder(out), mkdir(out); end
summary=struct('family',{},'passed',{},'file',{},'reason',{});
for family={'simple','hood'}
    task=makeTask(family{1},20260915,'development',true);
    fprintf('Endpoint %s = %d\n',family{1},task.endpointPassed);
    for k=1:8
        file=fullfile(out,sprintf('%s_%02d.mat',family{1},k));
        try
            result=dpSolve(task,dpSeed(task,k),c); save(file,'task','result','c','-v7');
            summary(end+1)=struct('family',family{1},'passed',result.passed,'file',file,'reason',result.solverStatus); %#ok<AGROW>
        catch e
            summary(end+1)=struct('family',family{1},'passed',false,'file',file,'reason',e.message); %#ok<AGROW>
            fprintf('%s\n',getReport(e,'extended'));
        end
        dpWriteJson(fullfile(out,'summary.json'),summary);
        if summary(end).passed, break; end
    end
end
assert(any([summary(strcmp({summary.family},'simple')).passed]) && any([summary(strcmp({summary.family},'hood')).passed]), ...
    'DP:Stage1','Both nominal scenes must pass before data generation.');
fprintf('DP_STAGE1_PASS\n');
end
