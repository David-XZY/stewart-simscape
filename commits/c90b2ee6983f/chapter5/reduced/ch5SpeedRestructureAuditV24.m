function report=ch5SpeedRestructureAuditV24()
% Saved-state audits precede Simscape validation; every hypothesis is retained.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'restructure_audit_v24');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
data=load(fullfile(base,'failure_replay_v23','audit.mat'),'report');tail=jsondecode(fileread(fullfile(base,'tail_audit_v23','summary.json')));assert(tail.accepted,'ch5:TailUnverified','Tail equivalence must pass.');
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'relinearization',{{}},'solver_rows',{{}},'reuse_rows',{{}},'completed',false,'accepted',true,'error','');
try
    for mi=1:numel(data.report.rows)
        item=data.report.rows{mi};r=ch5ReducedPrepare(4,string(item.mode));r.reduced=settings();k=ch5ReducedKernelsV4(r);s=item.replay;
        clearSensitivity();U=s.warm;trials={};
        for iteration=1:2
            timer=tic;[proposal,qp]=ch5ReducedStepV24(s.x,s.previous,U,s.goal,s.observed,s.prior,r,k,.6);
            [U,check,trajectory,cost,audit]=ch5ReducedAuditV23(s.x,s.previous,proposal,s.goal,s.observed,r,k,s.prior);seconds=toc(timer);
            trials{end+1}=struct('solver',qp,'check',check,'cost',cost,'seconds',seconds,'audit',audit,'U',U,'X',trajectory.X);
            fprintf('Relinearize %s iteration%d accepted%d collision %.6fmm %.1fms flag%d reuse%d\n',item.mode,iteration,check.accepted, ...
                1000*check.internal_check.collision_violation,1000*seconds,qp.exitflag,qp.sensitivity.reused_stages);
            if check.accepted,break;end
        end
        independent=ch5ReducedAuditGuardedV10(s.x,s.previous,U,s.goal,s.observed,r,k,s.prior);
        row=struct('mode',item.mode,'time',.6,'trials',{trials},'accepted',check.accepted&&independent.accepted);report.relinearization{end+1}=row;
        report.accepted=report.accepted&&row.accepted;
        source=load(fullfile(base,'closed_generic_v22',sprintf('004_%s.mat',strrep(item.mode,'-','_'))),'records','predictions');
        for time=[.6,4.6,9.3,9.4]
            idx=find(cellfun(@(v)abs(v.time-time)<1e-8,source.records),1);sample=source.records{idx};
            if time==.6,U=s.warm;else,U=reconstructWarm(source,idx,r,k);end
            exact=r;exact.reduced.reuse_sensitivities=false;
            [~,linear]=ch5ReducedStepV24(sample.x,sample.previous_u,U,sample.goal,sample.observed,s.prior,exact,k,time,true);
            [oldD,oldInfo]=ch5ReducedSolveV19(linear,"whiten-active");[newD,newInfo]=ch5ReducedSolveV24(linear,r);
            error=inf;if oldInfo.finite&&newInfo.finite,error=norm(oldD-newD,inf);end
            row=struct('mode',item.mode,'time',time,'old_solver',oldInfo,'new_solver',newInfo,'delta_difference',error,'linear',linear, ...
                'accepted',newInfo.original_linear_feasible&&newInfo.relative_stationarity<1e-5&&newInfo.dual_violation<1e-6);
            report.solver_rows{end+1}=row;
            fprintf('QP rows %s t%.1f oldflag%d/%dit %.1fms newflag%d/%dit %.1fms KKT %.3g\n',item.mode,time,oldInfo.exitflag,oldInfo.iterations, ...
                oldInfo.solver_seconds*1000,newInfo.exitflag,newInfo.iterations,newInfo.solver_seconds*1000,newInfo.relative_stationarity);
        end
        times=cellfun(@(v)v.time,source.records);refresh=cellfun(@(v)v.prediction.refresh&&logical(v.prediction.chosen),source.predictions);
        indices=find(refresh&((times<=2)|(times>=4.5&times<=5.6)|(times>=9&times<=9.8)));
        scale=[.1*ones(3,1);deg2rad(10)*ones(3,1);.1*ones(3,1);.2*ones(3,1);160*ones(r.plant.n-12,1)];
        for age=[.10000001,.50000001]
            clearSensitivity();cachedSeconds=[];freshSeconds=[];errors=[];reused=[];fresh=[];
            for idx=indices
                sample=source.records{idx};U=source.predictions{idx}.prediction.candidates{1}.U;nodes=full(k.nodes(sample.x,U));
                rr=r;rr.reduced.sensitivity_maximum_age=age;timer=tic;[A,B,info]=ch5ReducedJacobianV24(nodes,U,sample.goal,sample.time,rr,k);cachedSeconds(end+1)=toc(timer);
                timer=tic;[~,FA,FB]=k.stages(nodes(:,1:r.c.H),U);freshSeconds(end+1)=toc(timer);FA=full(FA);FB=full(FB);
                maximum=0;
                for j=1:r.c.H
                    rows=r.plant.n*(j-1)+(1:r.plant.n);cols=6*(j-1)+(1:6);
                    a=(A(:,rows)-FA(:,rows)).*scale'./scale;b=(B(:,cols)-FB(:,cols))*160./scale;
                    fa=FA(:,rows).*scale'./scale;fb=FB(:,cols)*160./scale;
                    maximum=max(maximum,max(norm(a,inf)/max(1,norm(fa,inf)),norm(b,inf)/max(1,norm(fb,inf))));
                end
                errors(end+1)=maximum;reused(end+1)=info.reused_stages;fresh(end+1)=info.fresh_stages;
            end
            row=struct('mode',item.mode,'maximum_age',age,'times',times(indices),'cached_seconds',cachedSeconds,'fresh_seconds',freshSeconds, ...
                'weighted_relative_jacobian_errors',errors,'reused',reused,'fresh',fresh,'accepted',max(errors)<=.01);
            report.reuse_rows{end+1}=row;
            fprintf('Reuse audit %s age%.1f median %.1fms vs %.1fms max weighted err %.4g reused %.1f%%\n',item.mode,age, ...
                1000*median(cachedSeconds),1000*median(freshSeconds),max(errors),100*sum(reused)/(r.c.H*numel(indices)));
        end
        write(directory,report);
    end
    report.completed=true;
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function U=reconstructWarm(source,index,r,k)
last=find(cellfun(@(s)s.prediction.refresh,source.predictions(1:index-1)),1,'last');p=source.predictions{last}.prediction;sample=source.records{last};
assert(p.chosen,'ch5:ReplayAnchor','Replay requires an accepted previous refresh.');old=p.candidates{1};
saved=struct('time',sample.time,'refresh_time',sample.time,'trajectory_time',sample.time,'goal',sample.goal,'observed',sample.observed, ...
    'anchor',sample.x(1:6),'trajectory',old);
for j=last+1:index
    s=source.records{j};saved.trajectory=ch5ReducedWarmV23(s.x,s.previous_u,saved,r,k,s.time);saved.time=s.time;saved.trajectory_time=s.time;
end
U=saved.trajectory.U;
end
function s=settings()
s=struct('iterations',2,'backtracking',2,'linearization_collision_reserve',.0002,'velocity_weight',1,'factor_derivatives',false, ...
    'reuse_sensitivities',true,'sensitivity_maximum_age',.50000001,'sensitivity_relative_error',.01,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
end
function clearSensitivity()
if isappdata(0,'ch5ReducedSensitivityV24'),rmappdata(0,'ch5ReducedSensitivityV24');end
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
for j=1:numel(summary.solver_rows),summary.solver_rows{j}=rmfield(summary.solver_rows{j},'linear');end
for j=1:numel(summary.relinearization)
    for t=1:numel(summary.relinearization{j}.trials),summary.relinearization{j}.trials{t}=rmfield(summary.relinearization{j}.trials{t},{'U','X'});end
end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
