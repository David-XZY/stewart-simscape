function report=ch5SpeedAffinePriorAuditV46()
[c,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'affine_prior_audit_v46');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve all experiments.');mkdir(directory);
teacher=load(fullfile(base,'teacher_candidate_audit_v39','audit.mat'),'report');
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'completed',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepareV45(4,mode);r.c.H=60;r.c.version=[c.version,'-speed-reduced-v1'];
        r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV4(r);
        indices=find(cellfun(@(v)strcmp(v.mode,mode),teacher.report.rows));
        for index=indices
            sample=teacher.report.rows{index};context=sample.context;x=context(1:r.plant.n);previous=context(31:36);goal=context(37:42);o=reshape(context(43:218),22,8);
            [anchor,anchorCheck,~,anchorCost]=ch5ReducedAuditV23(x,previous,sample.teacher_U,goal,o,r,k,zeros(6,60));
            if ~anchorCheck.accepted
                report.rows{end+1}=struct('mode',char(mode),'window_index',sample.window_index,'anchor_accepted',false,'anchor_check',anchorCheck);write();continue;
            end
            for seed=5101:5105
                r.seed=seed;r.weights=load(fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed)));
                for startTime=[99,24]
                    setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',seed+sample.window_index+660000));
                    [candidates,info]=ch5ReducedAffinePriorV46(x,previous,goal,o,anchor,r,k,startTime);
                    row=struct('mode',char(mode),'window_index',sample.window_index,'seed',seed,'start_t',startTime, ...
                        'anchor_accepted',true,'anchor_cost',anchorCost,'candidates',{candidates},'info',info);report.rows{end+1}=row;
                    fprintf('Affine prior window%d seed%d t%d raw%d final%d/4 improved%d %.1fms\n',sample.window_index,seed,startTime, ...
                        sum(cellfun(@(v)v.raw_check.accepted,candidates)),sum(cellfun(@(v)v.check.accepted,candidates)), ...
                        sum(cellfun(@(v)v.check.accepted&&v.cost<anchorCost-1e-10,candidates)),1000*info.seconds);
                end
            end
            write();
        end
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
        for j=1:numel(summary.rows)
            if ~isfield(summary.rows{j},'candidates'),continue;end
            for ci=1:numel(summary.rows{j}.candidates),summary.rows{j}.candidates{ci}=rmfield(summary.rows{j}.candidates{ci},{'U','X'});end
        end
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
    end
end
