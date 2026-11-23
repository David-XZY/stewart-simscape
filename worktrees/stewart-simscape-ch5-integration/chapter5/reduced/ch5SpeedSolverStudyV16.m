function report=ch5SpeedSolverStudyV16()
clear ch5ReducedSolveV16
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'solver_study_v16');
assert(~isfolder(directory),'ch5:ExistingStudy','Preserve solver-study attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'matrices',{{}});
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002, ...
        'velocity_weight',1,'factor_derivatives',false,'retain_numerical',false,'restoration_attempts',0);
    k=ch5ReducedKernelsV4(r);source=load(fullfile(base,'closed_warm_v11',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
    for index=[1,31,191,276,471]
        sample=source.records{index};saved=reconstruct(source,index,r,k);count=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)));
        install(saved,r,count);U=[];
        if ~isempty(saved)&&~isempty(saved.trajectory)
            old=saved.trajectory;shift=round((sample.time-saved.trajectory_time)/r.c.Ts);
            U=[old.U(:,shift+1:end),repmat(old.artificial_force,1,shift)];steady=[old.artificial_pose;zeros(6,1)];
            if r.plant.n>12,steady=[steady;repmat(old.artificial_force,(r.plant.n-12)/6,1)];end
            warmNodes=[old.X(:,shift+1:end),repmat(steady,1,shift)];feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(sample.x,r,k);
            U=full(feedback(sample.x,warmNodes,U,gain,zeros(6,r.c.H/5)));
        end
        if isempty(U),brake=ch5ReducedBrakeSequence(r);U=full(brake(sample.x,sample.previous_u));end
        [prior,~]=ch5ReducedPrior(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);
        U=ch5ReducedAuditV12(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
        phase=tic;[~,linear]=ch5ReducedStepV9(sample.x,sample.previous_u,U,sample.goal,sample.observed,prior,r,k,true);linearSeconds=toc(phase);
        report.matrices{end+1}=struct('mode',char(mode),'time',sample.time,'linear',linear);
        for backend=["active","null-active"]
            times=[];info=[];
            for repeat=0:1
                [delta,info]=ch5ReducedSolveV16(linear,backend);if repeat>0,times(end+1)=info.total_seconds;end
            end
            proposal=U;
            if info.finite
                feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(sample.x,r,k);
                proposal=full(feedback(sample.x,linear.nodes,U,gain,reshape(delta,6,r.c.H/5)));
            end
            [~,check,~,cost]=ch5ReducedAuditV12(sample.x,sample.previous_u,proposal,sample.goal,sample.observed,r,k,prior);
            row=struct('mode',char(mode),'time',sample.time,'backend',char(backend),'linear_seconds',linearSeconds, ...
                'seconds',times,'solve',info,'nonlinear_accepted',check.accepted,'nonlinear_violation',check.inequality_violation,'cost',cost);
            report.rows{end+1}=row;
            fprintf('QP study %s t%.1f %s %.1fms flag%d iter%d linear%d nonlinear%d obj%.4g\n', ...
                mode,sample.time,backend,1000*median(times),info.exitflag,info.iterations,info.original_linear_feasible,check.accepted,info.objective);
            write(directory,report);
        end
    end
end
end

function saved=reconstruct(source,index,r,k)
saved=[];if index==1,return;end
last=find(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)),1,'last');
p=source.predictions{last}.prediction;sample=source.records{last};trajectory=[];
anchorIndex=find(cellfun(@(p)p.prediction.chosen,source.predictions(1:last)),1,'last');
anchor=r.task.q0;if ~isempty(anchorIndex),anchor=source.records{anchorIndex}.x(1:6);end
if p.chosen,trajectory=p.candidates{1};end
saved=struct('time',sample.time,'refresh_time',sample.time,'trajectory_time',sample.time, ...
    'goal',sample.goal,'observed',sample.observed,'anchor',anchor,'trajectory',trajectory);
for j=last+1:index-1
    sample=source.records{j};p=source.predictions{j}.prediction;
    if ~isempty(saved.trajectory)
        warm=ch5ReducedWarmV11(sample.x,sample.previous_u,saved,r,k,sample.time);
        if p.chosen,saved.trajectory=warm;saved.anchor=sample.x(1:6);else,saved.trajectory=[];end
    end
    saved.time=sample.time;saved.trajectory_time=sample.time;saved.goal=sample.goal;saved.observed=sample.observed;
end
end
function install(saved,r,refreshCount)
if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
if ~isempty(saved),setappdata(0,'ch5ReducedExecution',saved);end
generator=RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index));
for j=1:refreshCount,randn(generator,6*r.c.H,r.c.candidates);end
setappdata(0,'ch5Random',generator);
end
function write(directory,report)
save(fullfile(directory,'study.mat'),'report','-v7');summary=rmfield(report,'matrices');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
