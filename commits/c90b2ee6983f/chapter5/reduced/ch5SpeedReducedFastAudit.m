function report=ch5SpeedReducedFastAudit()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','fast_matrix_transient_audit');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'),'identity',ch5ReducedIdentity());
for mode=["ideal-force","nonideal-force"]
    for task=[1,4]
        r=ch5ReducedPrepare(task,mode);k=ch5ReducedKernels(r);[goal,o]=ch5Observe(r.task,2);
        x=[r.task.q0;r.task.qd0];previous=r.initialForce;if r.plant.n>12,x=[x;repmat(previous,3,1)];end
        x(7)=.01;x(10)=.005;brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));prior=zeros(6,r.c.H);
        timer=tic;[~,reference]=ch5ReducedStepReference(x,previous,U,goal,o,prior,r,k);referenceSeconds=toc(timer);
        timer=tic;[~,fast]=ch5ReducedStepFast(x,previous,U,goal,o,prior,r,k,true);fastSeconds=toc(timer);
        errors=struct();valid=true;
        for name={'Q','f','A','b','E','e','M','nodes'}
            delta=norm(fast.(name{1})-reference.(name{1}),'inf');relative=delta/(1+norm(reference.(name{1}),'inf'));
            errors.(name{1})=struct('maximum',delta,'relative',relative);valid=valid&&relative<1e-10;
        end
        row=struct('mode',char(mode),'task',task,'reference_seconds',referenceSeconds,'fast_seconds',fastSeconds,'errors',errors,'accepted',valid);
        report.rows{end+1}=row;fprintf('Vectorized QP %s task%d identical=%d old %.4fs new %.4fs\n',mode,task,valid,referenceSeconds,fastSeconds);
        save(fullfile(directory,sprintf('%d_%s.mat',task,strrep(mode,'-','_'))),'reference','fast','row','-v7');
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
saved=load(fullfile(root,'chapter5','artifacts','reduced_probe','closed_latest_v2','003_nonideal_force.mat'),'records');
r=ch5ReducedPrepare(3,"nonideal-force");k=ch5ReducedKernels(r);record=saved.records{8};
timer=tic;extra=ch5ReducedTransient(record.x,record.u,record.observed,r);transientBuildSeconds=toc(timer);
timer=tic;extra=ch5ReducedTransient(record.x,record.u,record.observed,r);transientSeconds=toc(timer);
[u,recovery]=ch5ReducedRecoveryFast(record.x,record.previous_u,record.observed,r,k,record.time,record.q);
report.transient=extra;report.transient_seconds=transientSeconds;report.transient_build_seconds=transientBuildSeconds;report.recovery=recovery;
fprintf('Missed transient replay: peak %.6f violation %.6f rejected=%d hot %.6fs recovery valid=%d\n',extra.maximum_acceleration,extra.maximum_violation,~extra.accepted,transientSeconds,recovery.interval_valid);
save(fullfile(directory,'transient_replay.mat'),'record','extra','recovery','u','report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(all(cellfun(@(s)s.accepted,report.rows)),'ch5:MatrixMismatch','Batched formulation changed the QP.');
assert(~extra.accepted&&extra.maximum_acceleration>1.2,'ch5:TransientMiss','Additional points must reject the previously missed physical peak.');
end
