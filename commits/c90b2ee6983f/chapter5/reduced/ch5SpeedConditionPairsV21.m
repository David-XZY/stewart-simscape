function report=ch5SpeedConditionPairsV21()
% Same observed states / RNG; interleave both cores in one current process.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'condition_pairs_v21');
assert(~isfolder(directory),'ch5:ExistingPairs','Preserve timing attempts.');mkdir(directory);
report=struct('accepted',true,'pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}}, ...
    'timing_scope','controller_core_same_snapshot_with_restored_previous_sensitivity_cache');
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=settings();k=ch5ReducedKernelsV4(r);
    source=load(fullfile(base,'closed_warm_v11',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
    for index=[31,191,276,471]
        previousIndex=index-5;sample=source.records{previousIndex};before=reconstruct(source,previousIndex,r,k);
        count=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:previousIndex-1)));install(before,r,count);clearSensitivity();
        ch5ReducedControlV21(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);
        cached=[];if isappdata(0,'ch5ReducedSensitivity'),cached=getappdata(0,'ch5ReducedSensitivity');end
        sample=source.records{index};before=reconstruct(source,index,r,k);count=sum(cellfun(@(p)p.prediction.refresh,source.predictions(1:index-1)));
        old=[];fresh=[];errors=[];reused=[];oldFailures=[];newFailures=[];accepted=true;
        for repetition=0:3
            for which=[1,2,2,1]
                install(before,r,count);clearSensitivity();if ~isempty(cached),setappdata(0,'ch5ReducedSensitivity',cached);end
                timer=tic;
                if which==1,[u,d,p]=ch5ReducedControlV13(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);
                else,[u,d,p]=ch5ReducedControlV21(sample.x,sample.previous_u,sample.goal,sample.observed,r,sample.time);end
                seconds=toc(timer);
                if which==1,reference=u;end
                if repetition>0
                    if which==1,old(end+1)=seconds;oldFailures(end+1)=~p.denoise_attempts{1}.solver_success;
                    else
                        fresh(end+1)=seconds;errors(end+1)=norm(u-reference,inf);reused(end+1)=p.denoise_attempts{1}.sensitivity.reused_stages;
                        newFailures(end+1)=~p.denoise_attempts{1}.solver_success;
                        if p.chosen,gate=ch5ReducedAuditGuardedV10(sample.x,sample.previous_u,p.candidates{1}.U,sample.goal,sample.observed,r,k,zeros(6,r.c.H));
                        else,gate=ch5ReducedHoldAudit(sample.x,sample.previous_u,u,sample.observed,r);end
                        accepted=accepted&&gate.accepted;
                    end
                end
            end
        end
        row=struct('mode',char(mode),'time',sample.time,'reference_seconds',old,'conditioned_seconds',fresh,'force_errors',errors, ...
            'reused_stages',reused,'reference_solver_failures',oldFailures,'conditioned_solver_failures',newFailures,'accepted',accepted);
        report.rows{end+1}=row;report.accepted=report.accepted&&accepted;
        fprintf('Interleaved condition %s t%.1f %.1f -> %.1fms physical%d reuse%d\n',mode,sample.time,1000*median(old),1000*median(fresh),accepted,median(reused));
    end
end
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:PairPhysicalGate','Keep rejected timing candidates.');
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
    'factor_derivatives',false,'retain_numerical',false,'restoration_attempts',0,'reuse_sensitivities',true);
end
function clearSensitivity()
if isappdata(0,'ch5ReducedSensitivity'),rmappdata(0,'ch5ReducedSensitivity');end
end
