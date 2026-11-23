function report=ch5SpeedProjectedPriorAuditV40()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'projected_prior_audit_v40');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve past attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'completed',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV4(r);
        source=load(fullfile(base,'closed_generic_v33',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        for time=[.6,9.3,13]
            index=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{index};
            anchor=source.predictions{index}.prediction.candidates{1}.U;
            [anchor,check]=ch5ReducedAuditV23(sample.x,sample.previous_u,anchor,sample.goal,sample.observed,r,k,zeros(size(anchor)));
            usedBrake=false;
            if ~check.accepted
                brake=ch5ReducedBrakeSequence(r);anchor=full(brake(sample.x,sample.previous_u));
                [anchor,check]=ch5ReducedAuditV23(sample.x,sample.previous_u,anchor,sample.goal,sample.observed,r,k,zeros(size(anchor)));usedBrake=true;
            end
            if ~check.accepted
                report.rows{end+1}=struct('mode',char(mode),'time',time,'anchor_accepted',false,'anchor_check',check,'used_brake',usedBrake);write();continue;
            end
            for startTime=[9,24,49]
                generator=RandStream('mt19937ar','Seed',640000+r.seed+round(100*time)+startTime);setappdata(0,'ch5Random',generator);
                [candidates,info]=ch5ReducedProjectedPriorV40(sample.x,sample.previous_u,sample.goal,sample.observed,anchor,r,k,startTime);
                row=struct('mode',char(mode),'time',time,'seed',r.seed,'anchor_accepted',true,'used_brake',usedBrake, ...
                    'start_t',startTime,'candidates',{candidates},'info',info);report.rows{end+1}=row;
                fprintf('Projected prior %s t%.1f noise%d valid%d/4 improved%d changes %.6fN cost %.5g %.1fms\n',mode,time,startTime, ...
                    sum(cellfun(@(v)v.check.accepted,candidates)),sum(cellfun(@(v)v.improved,candidates)), ...
                    max(cellfun(@(v)v.force_rms_change,candidates)),info.anchor_cost-min(cellfun(@(v)v.cost,candidates)),1000*info.seconds);
                write();
            end
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
