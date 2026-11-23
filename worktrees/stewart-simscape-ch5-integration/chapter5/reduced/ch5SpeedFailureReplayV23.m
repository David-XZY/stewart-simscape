function report=ch5SpeedFailureReplayV23()
% Locate saved nonlinear failures without rerunning the Simscape episode.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');
directory=fullfile(base,'failure_replay_v23');assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'formal_ready',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
        source=load(fullfile(base,'closed_generic_v22',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
        index=find(cellfun(@(s)abs(s.time-.6)<1e-9,source.records),1);sample=source.records{index};p=source.predictions{index}.prediction;
        row=struct('mode',char(mode),'time',sample.time,'saved_trials',{{}});
        for ti=1:numel(p.denoise_attempts{1}.trials)
            trial=p.denoise_attempts{1}.trials{ti};row.saved_trials{end+1}=locate(sample.x,trial.U,sample.observed,r,k);
        end
        saved=reconstruct(source,index,r,k);shift=round((sample.time-saved.trajectory_time)/r.c.Ts);old=saved.trajectory;
        U=[old.U(:,shift+1:end),repmat(old.artificial_force,1,shift)];
        steady=[old.artificial_pose;zeros(6,1)];if r.plant.n>12,steady=[steady;repmat(old.artificial_force,(r.plant.n-12)/6,1)];end
        warmNodes=[old.X(:,shift+1:end),repmat(steady,1,shift)];feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(sample.x,r,k);
        U=full(feedback(sample.x,warmNodes,U,gain,zeros(6,r.c.H/5)));
        generator=RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index));
        count=sum(cellfun(@(s)s.prediction.refresh,source.predictions(1:index-1)));
        for j=1:count,randn(generator,6*r.c.H,r.c.candidates);end;setappdata(0,'ch5Random',generator);
        prior=ch5ReducedPriorV21(sample.x,sample.previous_u,sample.goal,sample.observed,r,k);
        [U,incumbent]=ch5ReducedAuditV22(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
        row.warm_accepted=incumbent.accepted;row.warm=locate(sample.x,U,sample.observed,r,k);
        exact=r;exact.reduced.reuse_sensitivities=false;
        [proposal,qp]=ch5ReducedStepV20(sample.x,sample.previous_u,U,sample.goal,sample.observed,prior,exact,k,sample.time);
        row.fresh_qp=qp;row.before_terminal=locate(sample.x,proposal,sample.observed,r,k);
        timer=tic;[after,terminal]=ch5ReducedTerminal(sample.x,proposal,r,k);row.terminal_seconds=toc(timer);
        row.terminal=terminal;row.after_terminal=locate(sample.x,after,sample.observed,r,k);
        row.repair_prefix_difference=max(abs(after(:,1:r.c.H-10)-proposal(:,1:r.c.H-10)),[],'all');
        [nodes,values,trace]=k.preview(sample.x,proposal);row.replay=struct('x',sample.x,'previous',sample.previous_u,'goal',sample.goal,'observed',sample.observed, ...
            'proposal',proposal,'nodes',full(nodes),'values',full(values),'trace',full(trace),'prior',prior,'warm',U);
        fprintf('Failure replay %s before %.6fmm at %.6fs stage%d point%d; after %.6fmm at %.6fs; Newton %.1fms\n', ...
            mode,1000*row.before_terminal.violation,row.before_terminal.relative_time,row.before_terminal.stage,row.before_terminal.trace_point, ...
            1000*row.after_terminal.violation,row.after_terminal.relative_time,1000*row.terminal_seconds);
        report.rows{end+1}=row;write(directory,report);
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function detail=locate(x,U,o,r,k)
[~,~,trace]=k.preview(x,U);trace=full(trace);times=reshape(r.plant.traceTime(:)+(0:r.c.H-1)*r.c.Ts,1,[]);
detail=struct('minimum_gap',inf,'required_gap',r.c.safe_distance+r.c.sample_hold_margin,'violation',0, ...
    'relative_time',nan,'stage',nan,'trace_point',nan,'obstacle',nan,'terminal_suffix_start',(r.c.H-10)*r.c.Ts);
for oi=find(o(22,:)>.5)
    future=repmat(o(:,oi),1,numel(times));future(1:3,:)=future(1:3,:)+o(16:18,oi)*times;
    mapped=r.collision.single.map(numel(times));gaps=full(mapped(trace(1:6,:),future));[gap,idx]=min(gaps);
    if gap<detail.minimum_gap
        detail.minimum_gap=gap;detail.relative_time=times(idx);detail.stage=floor((idx-1)/numel(r.plant.traceTime))+1;
        detail.trace_point=mod(idx-1,numel(r.plant.traceTime))+1;detail.obstacle=oi;
    end
end
detail.violation=max(0,detail.required_gap-detail.minimum_gap);
end
function saved=reconstruct(source,index,r,k)
last=find(cellfun(@(s)s.prediction.refresh,source.predictions(1:index-1)),1,'last');p=source.predictions{last}.prediction;sample=source.records{last};
assert(p.chosen,'ch5:ReplayAnchor','Replay requires an accepted preceding refresh.');
saved=struct('time',sample.time,'refresh_time',sample.time,'trajectory_time',sample.time,'goal',sample.goal,'observed',sample.observed, ...
    'anchor',sample.x(1:6),'trajectory',p.candidates{1});
for j=last+1:index-1
    s=source.records{j};saved.trajectory=ch5ReducedWarmV11(s.x,s.previous_u,saved,r,k,s.time);saved.time=s.time;saved.trajectory_time=s.time;
end
end
function s=settings()
s=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002,'velocity_weight',1,'factor_derivatives',false,'reuse_sensitivities',true);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
for j=1:numel(summary.rows),summary.rows{j}=rmfield(summary.rows{j},'replay');end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
