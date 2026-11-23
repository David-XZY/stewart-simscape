function c3Pilot(runName)
% Development tasks only; never counted in formal results.
c=c3Setup(runName); z=load(fullfile(c.output,'standard.mat')); standard=z.standard; asset=c3Assets(c);
source=load(fullfile(c.source,'manifest.mat')); tasks=source.manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
records={}; methods={'geo_ihsid','retrieve_ihsid','diff_ihsid','geo_fast','retrieve_fast','diff_fast'};
for family={'simple','hood'}
    selected=find(cellfun(@(t)strcmp(t.family,family{1})&&strcmp(t.split,'validation'),tasks),1);
    task=tasks{selected}; history=[];
    for k=1:numel(methods)
        clear c3Solve sgPhysics sgBasis sgRepairPoint
        folder=fullfile(c.output,'pilot'); if ~isfolder(folder), mkdir(folder); end
        file=fullfile(folder,[family{1},'_',methods{k},'.mat']);
        job=struct('method',methods{k},'count',64,'seed',c.seed+101,'budget',600);
        out=c3Plan(task,job,c,asset,standard);
        record=struct('family',family{1},'method',methods{k},'passed',out.passed,'reason',out.reason,'onlineSeconds',out.onlineSeconds,'generationSeconds',out.generationSeconds);
        if out.passed
            audit=sgAudit(out.bundle,task,c); assert(audit.passed,'C3:PilotAudit','Independent audit disagrees.');
            if strcmp(methods{k},'retrieve_ihsid'), history=out.warm; end
        end
        assert(abs(out.timingClosureError)<1e-8 && out.timing.other>=-1e-6);
        save(file,'out','task','job','record','-v7'); records{end+1}=record; %#ok<AGROW>
        fprintf('C3_PILOT %s %s %s %.3f\n',family{1},methods{k},out.reason,out.onlineSeconds);
    end
    if ~isempty(history)
        changed=task; changed.T=task.T+.1; changed=sgFinalizeTask(changed);
        job.method='previous_ihsid'; out=c3Plan(changed,job,c,asset,standard,history);
        assert(out.historyUsed && ~isempty(out.attempts));
        first=out.attempts{1}; assert(isfield(first,'solver') && first.solver.warmInfo.state && first.solver.warmInfo.force && first.solver.warmInfo.dual);
        file=fullfile(c.output,'pilot',[family{1},'_history.mat']); save(file,'out','changed','job','-v7');
    end
end
dpWriteJson(fullfile(c.output,'pilot.json'),struct('complete',true,'records',[records{:}],'scope','Development only; failures retained.'));
end
