function report=ch5SpeedTailAuditV23()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'tail_audit_v23');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve every attempt.');mkdir(directory);
data=load(fullfile(base,'failure_replay_v23','audit.mat'),'report');report=struct('accepted',true,'pid',feature('getpid'),'rows',{{}},'identity',ch5ReducedIdentity(),'error','');
try
    for mi=1:numel(data.report.rows)
        item=data.report.rows{mi};r=ch5ReducedPrepare(4,string(item.mode));r.reduced=struct('factor_derivatives',false,'velocity_weight',1);k=ch5ReducedKernelsV4(r);s=item.replay;
        for variant=1:3
            U=s.proposal;if variant==2,U(:,end-9:end)=U(:,end-9:end)+2;elseif variant==3,U=s.warm;end
            nodes=full(k.nodes(s.x,U));ch5ReducedTerminalV23(s.x,U,r,k,nodes);oldTimes=zeros(1,5);newTimes=oldTimes;
            for repeat=1:5
                timer=tic;[oldU,oldInfo]=ch5ReducedTerminal(s.x,U,r,k);oldTimes(repeat)=toc(timer);
                timer=tic;[newU,newInfo,newNodes]=ch5ReducedTerminalV23(s.x,U,r,k,nodes);newTimes(repeat)=toc(timer);
            end
            primal=full(k.nodes(s.x,newU));errors=[norm(oldU-newU,inf),norm(primal-newNodes,inf),abs(oldInfo.residual-newInfo.residual)];
            row=struct('mode',item.mode,'variant',variant,'errors',errors,'old_seconds',oldTimes,'new_seconds',newTimes, ...
                'iterations',newInfo.iterations,'accepted',max(errors)<1e-8&&oldInfo.iterations==newInfo.iterations&&oldInfo.finite==newInfo.finite);
            report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
            fprintf('Tail equivalence %s variant%d maxerr %.3g old %.1fms new %.1fms\n',item.mode,variant,max(errors),1000*median(oldTimes),1000*median(newTimes));
        end
        [oldU,oldC,oldT,oldCost]=ch5ReducedAuditV22(s.x,s.previous,s.proposal,s.goal,s.observed,r,k,s.prior);
        [newU,newC,newT,newCost]=ch5ReducedAuditV23(s.x,s.previous,s.proposal,s.goal,s.observed,r,k,s.prior);
        errors=[norm(oldU-newU,inf),norm(oldT.X-newT.X,inf),abs(oldCost-newCost),abs(oldC.inequality_violation-newC.inequality_violation)];
        row=struct('mode',item.mode,'variant','complete_gate','errors',errors,'accepted',max(errors)<1e-8&&oldC.accepted==newC.accepted);
        report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
    end
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);assert(report.accepted,'ch5:TailMismatch','Retain failed tail equivalence.');
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
