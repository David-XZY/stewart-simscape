function report=ch5SpeedHybridQpAuditV41()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'hybrid_qp_audit_v41');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve all attempts.');mkdir(directory);
source=load(fullfile(base,'restructure_audit_v24','audit.mat'),'report');
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'accepted',false,'error','');
try
    r=struct('reduced',struct('trust_radius',1,'qp_row_scaling',true,'qp_max_iterations',400));
    for index=1:numel(source.report.solver_rows)
        sample=source.report.solver_rows{index};linear=sample.linear;
        [reference,refInfo]=ch5ReducedSolveV36(linear,r);[delta,info]=ch5ReducedSolveV41(linear,r);
        seconds=zeros(7,2);
        for rep=1:7
            for variant=circshift(1:2,mod(rep,2))
                timer=tic;if variant==1,ch5ReducedSolveV36(linear,r);else,ch5ReducedSolveV41(linear,r);end
                seconds(rep,variant)=toc(timer);
            end
        end
        relativeCost=abs(info.objective-refInfo.objective)/max(1,abs(refInfo.objective));difference=norm(delta-reference,inf);
        row=struct('mode',sample.mode,'time',sample.time,'reference',refInfo,'hybrid',info,'seconds',seconds, ...
            'delta_difference',difference,'relative_objective_error',relativeCost, ...
            'accepted',info.solver_success&&info.original_linear_feasible&&relativeCost<1e-8&&difference<1e-6);
        report.rows{end+1}=row;
        fprintf('Hybrid QP %s t%.1f firstflag%d retry%d finalflag%d %.2f vs %.2fms difference %.3g accepted%d\n', ...
            sample.mode,sample.time,info.first_attempt.exitflag,info.used_row_scaled_retry,info.exitflag,1000*median(seconds),difference,row.accepted);
    end
    report.accepted=all(cellfun(@(v)v.accepted,report.rows));
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();assert(report.accepted,'ch5:HybridMismatch','Do not admit a changed or inaccurate QP solution.');
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
