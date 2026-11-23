function report=ch5SpeedSensitivityAuditV26()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'sensitivity_audit_v26');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'completed',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('factor_derivatives',false,'reuse_sensitivities',true,'sensitivity_maximum_age',.50000001,'sensitivity_relative_error',.001);k=ch5ReducedKernelsV4(r);
        source=load(fullfile(base,'closed_generic_v22',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        times=cellfun(@(v)v.time,source.records);refresh=cellfun(@(v)v.prediction.refresh&&logical(v.prediction.chosen),source.predictions);
        indices=find(refresh&((times<=2)|(times>=4.5&times<=5.6)|(times>=9&times<=9.8)));
        scale=[.1*ones(3,1);deg2rad(10)*ones(3,1);.1*ones(3,1);.2*ones(3,1);160*ones(r.plant.n-12,1)];
        for bounds=[.005,.02;.002,.01]'
            if isappdata(0,'ch5ReducedSensitivityV26'),rmappdata(0,'ch5ReducedSensitivityV26');end
            rr=r;rr.reduced.sensitivity_state_distance=bounds(1);rr.reduced.sensitivity_input_distance=bounds(2);
            cachedSeconds=[];freshSeconds=[];errors=[];reused=[];fresh=[];
            for idx=indices
                sample=source.records{idx};U=source.predictions{idx}.prediction.candidates{1}.U;nodes=full(k.nodes(sample.x,U));
                timer=tic;[A,B,info]=ch5ReducedJacobianV26(nodes,U,sample.goal,sample.time,rr,k);cachedSeconds(end+1)=toc(timer);
                timer=tic;[~,FA,FB]=k.stages(nodes(:,1:r.c.H),U);freshSeconds(end+1)=toc(timer);FA=full(FA);FB=full(FB);maximum=0;
                for j=1:r.c.H
                    rows=r.plant.n*(j-1)+(1:r.plant.n);cols=6*(j-1)+(1:6);
                    a=(A(:,rows)-FA(:,rows)).*scale'./scale;b=(B(:,cols)-FB(:,cols))*160./scale;
                    fa=FA(:,rows).*scale'./scale;fb=FB(:,cols)*160./scale;
                    maximum=max(maximum,max(norm(a,inf)/max(1,norm(fa,inf)),norm(b,inf)/max(1,norm(fb,inf))));
                end
                errors(end+1)=maximum;reused(end+1)=info.reused_stages;fresh(end+1)=info.fresh_stages;
            end
            row=struct('mode',char(mode),'parameters',rr.reduced,'times',times(indices),'cached_seconds',cachedSeconds,'fresh_seconds',freshSeconds, ...
                'weighted_relative_jacobian_errors',errors,'reused',reused,'fresh',fresh,'accepted',max(errors)<=.01);report.rows{end+1}=row;
            fprintf('Branch reuse %s state%.4f input%.3f median %.1fms vs %.1fms maxerr %.4g reuse %.1f%% accepted%d\n',mode,bounds(1),bounds(2), ...
                1000*median(cachedSeconds),1000*median(freshSeconds),max(errors),100*sum(reused)/(r.c.H*numel(indices)),row.accepted);
        end
        write(directory,report);
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
