function report=ch5SpeedProjectionAuditV25()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'projection_audit_v25');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
data=load(fullfile(base,'failure_replay_v23','audit.mat'),'report');report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'accepted',true,'error','');
try
    for mi=1:numel(data.report.rows)
        item=data.report.rows{mi};r=ch5ReducedPrepare(4,string(item.mode));r.reduced=settings();k=ch5ReducedKernelsV4(r);s=item.replay;
        for reuse=[false,true]
            if isappdata(0,'ch5ReducedSensitivityV24'),rmappdata(0,'ch5ReducedSensitivityV24');end
            rr=r;rr.reduced.reuse_sensitivities=reuse;U=s.warm;trials={};
            for iteration=1:2
                if iteration==2,rr.reduced.feasibility_projection=true;rr.reduced.trust_radius=.1;end
                timer=tic;[proposal,qp]=ch5ReducedStepV25(s.x,s.previous,U,s.goal,s.observed,s.prior,rr,k,.6);
                [U,check,trajectory,cost,audit]=ch5ReducedAuditV23(s.x,s.previous,proposal,s.goal,s.observed,rr,k,s.prior);seconds=toc(timer);
                trials{end+1}=struct('solver',qp,'check',check,'cost',cost,'seconds',seconds,'audit',audit,'U',U,'X',trajectory.X);
                fprintf('Projection %s reuse%d iteration%d accepted%d violation %.6fmm %.1fms flag%d reuse%d\n',item.mode,reuse,iteration,check.accepted, ...
                    1000*check.inequality_violation,1000*seconds,qp.exitflag,qp.sensitivity.reused_stages);
                if check.accepted,break;end
            end
            independent=ch5ReducedAuditGuardedV10(s.x,s.previous,U,s.goal,s.observed,rr,k,s.prior);
            row=struct('mode',item.mode,'reuse_enabled',reuse,'trials',{trials},'accepted',check.accepted&&independent.accepted);report.rows{end+1}=row;
            report.accepted=report.accepted&&row.accepted;
        end
        write(directory,report);
    end
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function s=settings()
s=struct('iterations',2,'backtracking',2,'linearization_collision_reserve',.0002,'velocity_weight',1,'factor_derivatives',false, ...
    'reuse_sensitivities',true,'sensitivity_maximum_age',.10000001,'sensitivity_relative_error',.01,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
for j=1:numel(summary.rows)
    for t=1:numel(summary.rows{j}.trials),summary.rows{j}.trials{t}=rmfield(summary.rows{j}.trials{t},{'U','X'});end
end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
