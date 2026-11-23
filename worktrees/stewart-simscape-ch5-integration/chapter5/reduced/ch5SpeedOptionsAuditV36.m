function report=ch5SpeedOptionsAuditV36()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'options_audit_v36');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
data=load(fullfile(base,'restructure_audit_v24','audit.mat'),'report');report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'accepted',true,'error','');
try
    r=struct('reduced',struct('qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1));
    for j=1:numel(data.report.solver_rows)
        item=data.report.solver_rows{j};linear=item.linear;ch5ReducedSolveV36(linear,r);oldTimes=[];newTimes=[];
        for repeat=1:5
            timer=tic;[oldD,oldInfo]=ch5ReducedSolveV24(linear,r);oldTimes(end+1)=toc(timer);
            timer=tic;[newD,newInfo]=ch5ReducedSolveV36(linear,r);newTimes(end+1)=toc(timer);
        end
        row=struct('mode',item.mode,'time',item.time,'old_seconds',oldTimes,'new_seconds',newTimes,'old_solver',oldInfo,'new_solver',newInfo, ...
            'delta_difference',norm(oldD-newD,inf),'accepted',isequal(oldD,newD)&&oldInfo.exitflag==newInfo.exitflag&&newInfo.original_linear_feasible&&newInfo.relative_stationarity<1e-5);
        report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
        fprintf('Option cache %s t%.1f old %.2fms new %.2fms exact%d\n',item.mode,item.time,1000*median(oldTimes),1000*median(newTimes),row.accepted);
    end
    tail=jsondecode(fileread(fullfile(base,'tail_audit_v23','summary.json')));report.accepted=report.accepted&&tail.accepted;
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);assert(report.accepted,'ch5:OptionsMismatch','Cached options must preserve solver outputs.');
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
