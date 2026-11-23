function report=ch5SpeedBlockingAuditV50()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'blocking_audit_v50');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve all experiments.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'accepted',true,'error','', ...
    'scope','Exact subspace embedding, unchanged 60-step physics, frozen local QP timing; no closed-loop guarantee');
try
    P=zeros(72,48);selected=[1,2,4,6,8,10,11,12];
    for block=1:8,P(6*(selected(block)-1)+(1:6),6*(block-1)+(1:6))=eye(6);end
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false, ...
            'reuse_sensitivities',false,'linearization_collision_reserve',.0002,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
        k=ch5ReducedKernelsV4(r);data=load(fullfile(base,'closed_generic_v33',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        for time=[.6,9.3,13]
            [~,index]=min(abs(cellfun(@(v)v.time,data.records)-time));sample=data.records{index};prediction=data.predictions{index}.prediction;
            x=sample.x;previous=sample.previous_u;goal=sample.goal;o=sample.observed;U=prediction.candidates{1}.U;
            [U,warmCheck,trajectory,warmCost]=ch5ReducedAuditV23(x,previous,U,goal,o,r,k,zeros(6,60));
            [~,old]=ch5ReducedStepV33(x,previous,U,goal,o,zeros(6,60),r,k,time,true);
            [~,small]=ch5ReducedStepV50(x,previous,U,goal,o,zeros(6,60),r,k,time,true);
            oldRestricted=struct('Q',P'*old.Q*P,'f',P'*old.f,'A',old.A*P,'b',old.b,'E',old.E*P,'e',old.e);
            errors=[relative(small.Q,oldRestricted.Q),relative(small.f,oldRestricted.f),relative(small.A,oldRestricted.A), ...
                relative(small.b,oldRestricted.b),relative(small.E,oldRestricted.E),relative(small.e,oldRestricted.e),relative(small.M,old.M*P)];
            D=reshape(sin(1:48)*.001,6,8);expanded=zeros(6,12);expanded(:,selected)=D;
            gain=ch5ReducedFeedbackGain(x,r,k);large=ch5ReducedFeedbackRollout(r);blocked=ch5ReducedFeedbackRolloutV50(r);
            [largeU,largeX]=large(x,old.nodes,U,gain,expanded);[smallU,smallX]=blocked(x,small.nodes,U,gain,D);
            errors=[errors,relative(full(largeU),full(smallU)),relative(full(largeX),full(smallX))];
            assert(all(isfinite(errors))&&max(errors)<1e-9,'ch5:BlockingMismatch','Blocked feedback and QP must equal the specified original subspace.');
            largeLinear=struct('Q',old.Q,'f',old.f,'A',old.A,'b',old.b,'E',old.E,'e',old.e);
            smallLinear=struct('Q',small.Q,'f',small.f,'A',small.A,'b',small.b,'E',small.E,'e',small.e);
            oldTimes=[];newTimes=[];
            for repeat=1:5
                if mod(repeat,2)==1
                    timer=tic;[~,oldSolve]=ch5ReducedSolveV24(largeLinear,r);oldTimes(end+1)=toc(timer);
                    timer=tic;[~,newSolve]=ch5ReducedSolveV24(smallLinear,r);newTimes(end+1)=toc(timer);
                else
                    timer=tic;[~,newSolve]=ch5ReducedSolveV24(smallLinear,r);newTimes(end+1)=toc(timer);
                    timer=tic;[~,oldSolve]=ch5ReducedSolveV24(largeLinear,r);oldTimes(end+1)=toc(timer);
                end
            end
            [proposal,step]=ch5ReducedStepV50(x,previous,U,goal,o,zeros(6,60),r,k,time);checks={};accepted=false;
            for line=0:5
                [~,check,~,cost]=ch5ReducedAuditV23(x,previous,U+2^(-line)*(proposal-U),goal,o,r,k,zeros(6,60));
                checks{end+1}=struct('line',line,'check',check,'cost',cost);
                if check.accepted&&(~warmCheck.accepted||cost<warmCost),accepted=true;break;end
            end
            row=struct('mode',char(mode),'time',time,'embedding_errors',errors,'accepted',true,'warm_check',warmCheck, ...
                'warm_cost',warmCost,'old_seconds',oldTimes,'new_seconds',newTimes,'old_solver',oldSolve,'new_solver',newSolve, ...
                'step',step,'trials',{checks},'improving_valid_proposal',accepted,'optimization_variables',48,'prediction_intervals',60);
            report.rows{end+1}=row;
            fprintf('Blocking %s t%.1f exact %.2g QP %.1f/%.1fms flags%d/%d valid-improving%d\n',mode,time,max(errors), ...
                1000*median(oldTimes),1000*median(newTimes),oldSolve.exitflag,newSolve.exitflag,accepted);
        end
    end
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();assert(report.accepted,'ch5:BlockingMismatch','Do not enter Simscape before the independent contract passes.');
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
function error=relative(a,b)
error=norm(a(:)-b(:),inf)/max(1,norm(b(:),inf));
end
