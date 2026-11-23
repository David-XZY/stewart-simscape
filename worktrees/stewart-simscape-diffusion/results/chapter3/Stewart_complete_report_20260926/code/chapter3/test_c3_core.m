function test_c3_core(runName)
if nargin<1, runName='chapter3_tests'; end
c=c3Setup(runName); checks={};
assert(c3SolverGate(true,'Solve_Succeeded',0,0,0,true,true,true));
args={true,'Solve_Succeeded',0,0,0,true,true,true};
for k=[1,6,7,8], a=args; a{k}=false; assert(~c3SolverGate(a{:})); end
for k=3:5, a=args; a{k}=1.01e-6; assert(~c3SolverGate(a{:})); a{k}=NaN; assert(~c3SolverGate(a{:})); end
args{2}='Maximum_Iterations_Exceeded'; assert(~c3SolverGate(args{:})); checks{end+1}='solver_status_residual_bounds_finite_budget_physics';
data=dir(fullfile(c.source,'attempts','simple_*','start_01.mat')); z=[];
for k=1:numel(data)
    trial=load(fullfile(data(k).folder,data(k).name));
    if trial.record.passed, z=trial; break; end
end
assert(~isempty(z)); task=z.task; bundle=struct('kind','spline','cp',z.record.cp,'T',task.T);
r=sgValidate(bundle,task,c); gold=sgAudit(bundle,task,c); assert(r.passed && gold.passed);
bad=bundle; bad.cp(1,6)=NaN; r=sgValidate(bad,task,c); assert(~r.passed);
limited=task; limited.model.actuator.forceMax(:)=1; limited.model.actuator.forceMin(:)=-1;
r=sgValidate(bundle,limited,c); assert(~r.passed); checks{end+1}='physical_negative_and_independent';
changed=task; changed.T=task.T+1; changed.qGoal(1)=task.qGoal(1)+.01;
transport=c3TransportWarm(z.result.bundle,changed,[0,changed.T]);
assert(max(abs(transport.q(:,1)-changed.q0))<1e-8 && max(abs(transport.q(:,end)-changed.qGoal))<1e-8);
assert(max(abs(transport.v(:)))<1e-7 && max(abs(transport.a(:)))<1e-7);
checks{end+1}='warm_endpoint_and_time_transport';
q=c3Quality(bundle,task,c); standard=struct('scales',max(q.values,1e-12),'groups',{c.groups}, ...
    'weights',c.weights,'primary',1,'hash','test-only','limits',struct());
lim=struct('groups',2*ones(1,4),'detours',2*[q.positionDetour,q.rotationDetour]+1e-8,'costs',2*ones(1,3));
standard.limits.simple=lim; standard.limits.hood=lim;
s=c3Score(q,task.family,standard); assert(s.qualityPassed && abs(s.primaryScore-50)<1e-8);
q2=q; q2.values(1)=q2.values(1)*10; s2=c3Score(q2,task.family,standard); assert(~s2.qualityPassed && s2.primaryScore<s.primaryScore);
q2=q; q2.rotationDetour=inf; badScore=c3Score(q2,task.family,standard); assert(~badScore.qualityPassed);
checks{end+1}='independent_score_and_no_compensated_extreme';
assetConfig=c; assetConfig.modelFile=fullfile(c.source,'training','model.json'); assetConfig.datasetFile=fullfile(c.source,'dataset.json');
asset=sgAssets(assetConfig);
[one,~]=c3Sample(task,1,908,asset); [many,~]=c3Sample(task,1:8,908,asset); [tail,~]=c3Sample(task,5:8,908,asset);
assert(max(abs(one(:)-reshape(many(:,:,1),[],1)))<1e-12);
assert(max(abs(tail(:)-reshape(many(:,:,5:8),[],1)))<1e-12); checks{end+1}='incremental_candidate_prefixes';
base=dpBaseline(task); delta=z.record.cp(:,4:13)-base(:,4:13);
fake=asset; fake.trainY=repmat(delta(:).',80,1); fake.trainNormalized=zeros(80,numel(task.context));
fake.model.inputMean=task.context; fake.model.inputScale=ones(size(task.context));
job=struct('method','retrieve_fast','count',8,'seed',908,'budget',600,'repairEnabled',false,'fallbackEnabled',false);
out=c3Plan(task,job,c,fake,standard); assert(out.passed && out.candidateCount==1 && numel(out.attempts)==1);
assert(abs(out.onlineSeconds-out.generationSeconds-out.evaluationSeconds)<1e-8 && abs(out.timingClosureError)<1e-8);
job.fullSweep=true; sweep=c3Plan(task,job,c,fake,standard); assert(sweep.passed && sweep.candidateCount==8 && numel(sweep.attempts)==8);
checks{end+1}='actual_early_return_and_exclusive_timers';
broken=task; broken.model.actuator.forceMax(:)=1; broken.model.actuator.forceMin(:)=-1;
out=c3Plan(broken,job,c,fake,standard); assert(~out.passed && out.candidateCount==0); checks{end+1}='boundary_return';
short=c; short.repairIterations=1; short.repairBudget=10; clock=tic;
[fixed,info]=c3Repair(task,bundle.cp,short,standard,clock);
assert(all(isfinite(fixed(:))) && isequal(fixed(:,[1:3,14:16]),bundle.cp(:,[1:3,14:16])) && info.iterations<=1);
checks{end+1}='quality_repair_fixed_endpoints';
strict=standard; strict.limits.(task.family).groups(:)=.01;
repairJob=job; repairJob.fullSweep=false; repairJob.count=1; repairJob.repairEnabled=true;
repaired=c3Plan(task,repairJob,short,fake,strict);
assert(~repaired.passed && repaired.physicalPassed && any(cellfun(@(a)strcmp(a.stage,'repair'),repaired.attempts)));
checks{end+1}='physically_valid_low_quality_triggers_repair';
repairJob.budget=0; expired=c3Plan(task,repairJob,short,fake,standard); assert(~expired.passed && strcmp(expired.reason,'wall_budget'));
checks{end+1}='wall_budget_not_a_success';
staticTask=task; staticTask.qGoal=task.q0; staticTask.T=.02;
staticBundle=struct('kind','spline','cp',repmat(task.q0,1,16),'T',.02);
R=rpy2rotmZYX(task.q0(4:6)); obj=task.scene.objectCylinder; center=task.q0(1:3)+R*obj.center_P;
for axis=1:2
    support=obj.radius; if axis==1, support=obj.length/2; end
    for gap=[-.02,.01-2e-8,.01,.01+2e-8]
        o=task.scene.hood.obstacles(1); o.R_S=R; o.halfSize=[.1;.1;.1]; o.center_S=center+R(:,axis)*(support+.1+gap);
        staticTask.scene.hood.obstacles=o; fast=sgValidate(staticBundle,staticTask,c); independent=sgAudit(staticBundle,staticTask,c);
        assert(fast.passed==independent.passed && fast.passed==(gap>=.01-1e-8));
    end
end
checks{end+1}='collision_threshold_both_cylinder_axes';
if isfile(c.modelFile), auditedAsset=c3Assets(c); assert(~isempty(auditedAsset.trainY)); checks{end+1}='audited_model_dataset_and_history_isolation'; end
% Same stationary curve in spline and IHS representations must have same score.
stationary=task; stationary.qGoal=task.q0; N=60; M=2*N+1;
a=struct('kind','spline','cp',repmat(task.q0,1,16),'T',task.T);
b=struct('kind','ihsid','T',task.T,'h',task.T/N,'intervals',N,'tNode',linspace(0,task.T,N+1), ...
    'X',repmat([task.q0;zeros(6,1)],1,M),'A',zeros(6,M),'F',zeros(6,M));
qa=c3Quality(a,stationary,c); qb=c3Quality(b,stationary,c);
assert(max(abs(qa.values-qb.values))<1e-7); checks{end+1}='representation_invariance';
dpWriteJson(fullfile(c.output,'core_tests.json'),struct('passed',true,'checks',{checks},'count',numel(checks),'matlab',version));
fprintf('C3_CORE_PASSED %d\n',numel(checks));
end
