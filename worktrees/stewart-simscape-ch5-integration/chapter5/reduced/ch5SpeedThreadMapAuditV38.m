function report=ch5SpeedThreadMapAuditV38()
% Independent stage maps: compare values and interleave warmed timings.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'thread_map_audit_v38');
assert(~isfolder(directory),'ch5:ExistingAudit','Retain old attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'rows',{{}},'accepted',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('factor_derivatives',false);k=ch5ReducedKernelsV4(r);
        source=load(fullfile(base,'closed_generic_v33',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        maps={k.stage.map(r.c.H),k.stage.map(r.c.H,'thread',2),k.stage.map(r.c.H,'thread',4)};
        for time=[0,.6,4.6,9.3,13]
            index=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{index};
            U=source.predictions{index}.prediction.candidates{1}.U;nodes=full(k.nodes(sample.x,U));states=nodes(:,1:r.c.H);
            [y,A,B]=maps{1}(states,U);reference={full(y),full(A),full(B)};errors=zeros(3,3);seconds=zeros(7,3);
            for variant=1:3
                [yy,aa,bb]=maps{variant}(states,U);values={full(yy),full(aa),full(bb)};
                for v=1:3,errors(variant,v)=norm(values{v}-reference{v},inf)/max(1,norm(reference{v},inf));end
            end
            for rep=1:7
                order=circshift(1:3,mod(rep,3));
                for variant=order
                    timer=tic;[yy,aa,bb]=maps{variant}(states,U);full(yy);full(aa);full(bb);seconds(rep,variant)=toc(timer);
                end
            end
            row=struct('mode',char(mode),'time',time,'threads',[1,2,4],'relative_errors',errors,'seconds',seconds, ...
                'accepted',max(errors,[],'all')<1e-10);report.rows{end+1}=row;
            fprintf('Thread maps %s t%.1f serial/2/4 %.2f %.2f %.2fms exact%d\n',mode,time,1000*median(seconds),row.accepted);
            write();
        end
    end
    report.accepted=all(cellfun(@(v)v.accepted,report.rows));
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();assert(report.accepted,'ch5:ThreadMismatch','Thread mapping changed a derivative.');
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
