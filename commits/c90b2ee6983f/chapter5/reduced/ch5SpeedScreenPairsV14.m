function report=ch5SpeedScreenPairsV14()
% Interleaved same-state core timing; no Simscape preparation/audit in timer.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'screen_pairs_v14');
assert(~isfolder(directory),'ch5:ExistingPairs','Preserve all timing attempts.');mkdir(directory);
report=struct('accepted',true,'pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'timing_scope','controller_core_same_snapshot');
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=settings();r.reduced.retain_numerical=false;r.reduced.restoration_attempts=0;k=ch5ReducedKernelsV4(r);
    source=load(fullfile(base,'closed_warm_v11',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
    for index=[26,276,321,401]
        sample=source.records{index};before=reconstruct(source,index,r,k);count=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)));
        old=[];fresh=[];errors=[];screens=[];
        for repetition=0:4
            for which=[1,2,2,1]
                install(before,r,count);timer=tic;
                if which==1
                    [u,~,p]=ch5ReducedControlV11(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);
                else
                    [u,~,p]=ch5ReducedControlV13(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);
                end
                seconds=toc(timer);
                if which==1,reference=u;end
                if repetition>0
                    if which==1,old(end+1)=seconds;else,fresh(end+1)=seconds;errors(end+1)=norm(u-reference,inf);screens(end+1)=p.objective_screens;end
                end
            end
        end
        row=struct('mode',char(mode),'time',sample.time,'reference_seconds',old,'screened_seconds',fresh, ...
            'force_errors',errors,'screen_counts',screens,'accepted',max(errors)<1e-8);
        report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
        fprintf('Interleaved screen %s t%.1f %.1f -> %.1fms errors %.3g\n',mode,sample.time,1000*median(old),1000*median(fresh),max(errors));
    end
end
write(directory,report);assert(report.accepted,'ch5:PairsDisagree','Preserve timing and numerical disagreement.');
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
function options=settings()
options=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
    'factor_derivatives',false,'retain_numerical',true,'restoration_attempts',1);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
