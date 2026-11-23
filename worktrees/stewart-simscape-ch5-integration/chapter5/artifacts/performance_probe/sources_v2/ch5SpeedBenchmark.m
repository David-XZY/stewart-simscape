function report=ch5SpeedBenchmark()
[~,root]=ch5Init();addpath(fullfile(root,'chapter5','performance'));out=fullfile(root,'chapter5','artifacts','performance_probe');
if ~isfolder(out),mkdir(out);end
report=struct('pid',feature('getpid'),'controller_identity',ch5SourceIdentity(),'rows',{{}},'scope','Independent development timing; no formal outcomes');
for mode=["ideal-force","nonideal-force"]
    for polish=[0,3]
        label=sprintf('one_sqp_%d_polish',polish);r=ch5SpeedPrepare(1,mode,label,polish);
        [goal,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];if mode=="nonideal-force",x=[x;repmat(r.initialForce,3,1)];end
        build=tic;preview=ch5NativePreview(r);ch5NativeNodes(r);ch5PredictiveOptimizer(r,find(o(22,:)>.5));
        if polish>0,ch5PredictiveOptimizer(r,find(o(22,:)>.5),true);end
        [~,~,trace]=preview(x,repmat(r.initialForce,1,r.c.H));ch5NativeCollisionAudit(full(trace),o,r);
        buildSeconds=toc(build);previous=r.initialForce;
        for sample=1:3
            time=(sample-1)*.1;if sample==3,goal(1)=goal(1)+.02;end
            timer=tic;[u,diagnostic,prediction]=ch5SpeedDiffusionControl(x,previous,goal,o,r,time);seconds=toc(timer);
            row=struct('mode',mode,'label',label,'sample',sample,'seconds',seconds,'build_seconds',buildSeconds, ...
                'budget_seconds',.1,'deadline_met',seconds<=.1,'fallback',diagnostic.usedFallback, ...
                'position_error',norm(x(1:3)-goal(1:3)),'terminal_position_error',norm(prediction.candidates{1}.X(1:3,end)-goal(1:3)), ...
                'prediction',prediction,'state',x,'u',u);
            report.rows{end+1}=row;
            if prediction.chosen>0,U=prediction.candidates{1}.U;else,U=repmat(u,1,r.c.H);end
            for j=1:5,x=full(r.plant.fd(x,U(:,j),zeros(6,1)));previous=U(:,j);end
            fprintf('Speed %s %s #%d: %.6fs fallback=%d terminal error %.6fm\n',mode,label,sample,seconds,diagnostic.usedFallback,row.terminal_position_error);
            save(fullfile(out,'benchmark.mat'),'report','-v7');
        end
        clear ch5PredictiveOptimizer ch5NativePreview ch5NativeNodes
    end
end
report.completed=true;save(fullfile(out,'benchmark.mat'),'report','-v7');
fid=fopen(fullfile(out,'benchmark.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
